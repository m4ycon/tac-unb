#include <algorithm>
#include <cassert>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <getopt.h>
#include <iostream>
#include <map>
#include <vector>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/properties.hpp>
#include <boost/graph/graph_utility.hpp>

#include "utils/cwa.hpp"
#include "utils/domain.hpp"
#include "utils/output.hpp"
#include "utils/parametersplitting.hpp"
#include "utils/parsetree.hpp"
#include "utils/plan.hpp"
#include "utils/sortexpansion.hpp"
#include "utils/typeof.hpp"
#include "utils/util.hpp"
#include "utils/verify.hpp"
#include "utils/properties.hpp"
#include "utils/ihtn_generator_utils.hpp"
#include "hddl/hddl.hpp"
#include "tdg/tdg.hpp"
#include "knowledgemanager/knowledgemanager.hpp"
#include "knowledgemanager/fileknowledgemanager.hpp"
#include "knowledgemanager/knowledgemanagerfactory.hpp"
#include "config/config.hpp"
#include "atmanager/at_manager.hpp"
#include "annotmanager/annotmanager.hpp"
#include "missiondecomposer/missiondecomposer.hpp"
#include "outputgenerator/outputgenerator.hpp"
#include "outputgenerator/xmloutputgenerator.hpp"
#include "outputgenerator/fileoutputgeneratorfactory.hpp"
#include "configchecker/configchecker.hpp"
#include "ihtngenerator/ihtngenerator.hpp"

using namespace std;

const string verbose_command = "-v";
const string pretty_print_command = "-p";
const string ihtn_output_command = "-h";

// declare parser function manually
void run_parser_on_file(FILE* f, char* filename);

// parsed domain data structures
bool has_typeof_predicate = false;
vector<sort_definition> sort_definitions;
vector<string> rewards_definitions;
vector<string> capabilities_definitions;
vector<predicate_definition> predicate_definitions;
vector<pair<predicate_definition,string>> parsed_functions;
vector<parsed_task> parsed_primitive;
vector<parsed_task> parsed_abstract;
map<string,vector<parsed_method> > parsed_methods;
string metric_target = dummy_function_type;

map<string,set<string>> sorts;
set<string> robot_related_sorts;
map<string,set<string>> robot_related_sorts_map;
vector<method> methods;
vector<task> primitive_tasks;
vector<task> abstract_tasks;

map<string, task> task_name_map;

map<string, variant<pair<string,string>,pair<vector<string>,string>>> gm_var_map;

bool mdp = false; //Variable for checking if mdp constructs are found
bool has_forall = false;
bool has_when = false;
bool has_capabilities_definitions = false;
bool verbose = false;
bool pretty_print = false;
bool ihtn_output = false;

int main(int argc, char** argv) {
	cin.sync_with_stdio(false);
	cout.sync_with_stdio(false);
	int dfile = -1;
	int jsonfile = -1;
	int configfile = -1;
	vector<int> options;

	for (int i = optind; i < argc; i++) {
		if (dfile == -1) dfile = i;
		else if (jsonfile == -1) jsonfile = i;
		else if (configfile == -1) configfile = i;
	}

	for(int i = configfile+1; i < argc; i++) {
		options.push_back(i);
	}

	bool option_not_found = false;
	for(int option : options) {
		string opt(argv[option]);

		if(opt == verbose_command) {
			verbose = true;
		} else if(opt == pretty_print_command) {
			pretty_print = true;
		} else if(opt == ihtn_output_command) {
			ihtn_output = true;
		} else {
			if(!option_not_found) {
				std::cout << std::endl;
			}
			std::cout << "Unknown option [" + opt + "]" << std::endl;
			option_not_found = true;
		}
	}

	if(verbose && pretty_print) {
		std::cout << "Options -v and -p cannot be used together!" << std::endl;

		return 1;
	}

	if(option_not_found) {
		std::cout << std::endl;
	}

	if(dfile == -1){
		cout << "You need to provide a domain file as input." << endl;
		return 1;
	}
	if(jsonfile == -1) {
		cout << "You need to provide a Goal Model JSON file as input." << endl;
		return 1;
	}
	if(configfile == -1) {
		cout << "You need to provide a configuration file as input." << endl;
		return 1;
	}

	FILE *domain_file = fopen(argv[dfile], "r");
	FILE *json_file = fopen(argv[jsonfile], "r");
	FILE *config_file;

	config_file = fopen(argv[configfile], "r");

	if (!domain_file) {
		cout << "I can't open " << argv[dfile] << "!" << endl;
		return 2;
	}
	if (!json_file) {
		cout << "I can't open " << argv[jsonfile] << "!" << endl;
		return 2;
	}
	if(!config_file) {
		cout << "I can't open " << argv[configfile] << "!" << endl;
		return 2;
	}

	if (has_forall) {
		cout << "MDP version does not support the FORALL operator yet." << endl;
		return 11;
	}

	namespace pt = boost::property_tree;

	pt::ptree json_root;
	pt::read_json(argv[jsonfile], json_root);

	// Generate configuration map from configuration file
	map<string, variant<map<string,string>, vector<string>, vector<SemanticMapping>, vector<VariableMapping>, pair<string,string>>> cfg;

	ConfigManager cfg_manager;
	cfg = cfg_manager.parse_configuration_file(argv[configfile]);

	map<string,string> type_mapping = std::get<map<string,string>>(cfg[type_mapping_config_key]);

	vector<VariableMapping> variable_mapping = std::get<vector<VariableMapping>>(cfg[var_mapping_config_key]);
	vector<SemanticMapping> semantic_mapping = std::get<vector<SemanticMapping>>(cfg[semantic_mapping_config_key]);
	
	vector<string> high_level_loc_types;
	if(cfg.find(location_types_config_key) != cfg.end()) {
		high_level_loc_types = std::get<vector<string>>(cfg[location_types_config_key]);
	}
	vector<string> high_level_agent_types;
	if(cfg.find(agent_types_config_key) != cfg.end()) {
		high_level_agent_types = std::get<vector<string>>(cfg[agent_types_config_key]);
	}
	vector<string> output_info = std::get<vector<string>>(cfg[output_config_key]);

	//Generate Knowledge Bases and Knowledge Manager
	KnowledgeManagerFactory k_manager_factory;
	shared_ptr<KnowledgeManager> knowledge_manager = k_manager_factory.create_knowledge_manager(cfg, type_mapping, "world_db");
	knowledge_manager->construct_knowledge_base(cfg);

	//Parse HDDL Domain file
	run_parser_on_file(domain_file, argv[dfile]);
		
	/*
		Add native sorts
	*/
	bool added_native_sorts = false; 
	bool has_object_sort = false;
	int cnt = 0;
	sort_definition aux;
	for(auto s : sort_definitions) {
		if(s.has_parent_sort && s.parent_sort == "object") {
			has_object_sort = true;
			aux = s;
			break;
		}
		cnt++;
	}

	if(has_object_sort) {
		aux.declared_sorts.push_back(hddl_robot_type);
		aux.declared_sorts.push_back(hddl_robotteam_type);
		added_native_sorts = true;

		sort_definitions.erase(sort_definitions.begin()+cnt);
		sort_definitions.insert(sort_definitions.begin()+cnt,aux);
	}

	if(!added_native_sorts) {
		sort_definition robot_sort;
		robot_sort.declared_sorts.push_back(hddl_robot_type);
		robot_sort.has_parent_sort = false;

		sort_definition robotteam_sort;
		robotteam_sort.declared_sorts.push_back(hddl_robotteam_type);
		robotteam_sort.has_parent_sort = false;

		sort_definitions.push_back(robot_sort);
		sort_definitions.push_back(robotteam_sort);
	}

	robot_related_sorts_map[hddl_robot_type] = set<string>();
	robot_related_sorts_map[hddl_robotteam_type] = set<string>();
	if(sort_definitions.size() > 0) {
		vector<sort_definition>::iterator sort_def_it = sort_definitions.begin();
		sort_definition current_sort = *sort_def_it;

		while(sort_def_it != sort_definitions.end()) {
			bool next = false;

			if(current_sort.has_parent_sort && (current_sort.parent_sort == hddl_robot_type || current_sort.parent_sort == hddl_robotteam_type)) {
				for(string sort : current_sort.declared_sorts) {
					robot_related_sorts.insert(sort);
					robot_related_sorts_map[current_sort.parent_sort].insert(sort);
				}

				next = true;
			} else if(current_sort.has_parent_sort && current_sort.parent_sort != hddl_robot_type && current_sort.parent_sort != hddl_robotteam_type) {
				bool found_def = false;
				
				sort_definition aux;
				for(sort_definition s : sort_definitions) {
					if(std::find(s.declared_sorts.begin(), s.declared_sorts.end(), current_sort.parent_sort) != s.declared_sorts.end()) {
						aux = s;
						found_def = true;

						break;
					}
				}

				if(found_def) {
					current_sort = aux;
				} else {
					next = true;
				}
			} else {
				next = true;
			}

			if(next) {
				sort_def_it++;

				if(sort_def_it != sort_definitions.end()) {
					current_sort = *sort_def_it;
				}
			}
		}
	}

	expand_sorts();

	flatten_mdp_tasks();
	parsed_method_to_data_structures(false, false, false);

	/*
		Goal Model parsing and generation of Abstract tasks instances
	*/

	GMGraph gm;
	gm = graph_from_property_tree(json_root);

	check_config(variable_mapping, type_mapping, gm, abstract_tasks, semantic_mapping, high_level_loc_types, predicate_definitions);

	check_gm_validity(gm);

	if(verbose) {
		print_gm(gm);

		print_gm_nodes_info(gm);
	}

	check_undefined_number_of_robots(gm, abstract_tasks, sort_definitions);

	ATManagerFactory at_manager_factory;
	shared_ptr<ATManager> at_manager_ptr = at_manager_factory.create_at_manager(knowledge_manager, abstract_tasks, gm, high_level_loc_types);

	map<string,vector<AbstractTask>> at_instances;

	if(at_manager_ptr->get_at_manager_type() == ATFILE) {
		FileKnowledgeATManager* at_manager = dynamic_cast<FileKnowledgeATManager*>(at_manager_ptr.get());

		FileKnowledgeManager* aux = dynamic_cast<FileKnowledgeManager*>(knowledge_manager.get());
		at_manager->set_fk_manager(aux);

		at_instances = at_manager->generate_at_instances(gm_var_map,variable_mapping);
	}

	if(verbose) {
		print_at_instances_info(at_instances);

		print_gm_var_map_info(gm_var_map);
	}

	/*
		Tasks generated by methods preconditions (which are preceded by __method_precondition_) contain in the vars attribute
		only variables used in the preconditions of the method. They come first in the method's task list.

		Example:
		Method

		(:method robot-sanitization
        	:parameters (?r - CleanerRobot ?srm - room ?rloc - location ?c - capability)
        	:task (RobotSanitization ?r ?srm ?rloc ?c)
        	:precondition (and
            	(hascapability ?r ?c)
       		)
        	:subtasks (and
            	(sanitize-robot ?r ?srm ?rloc)
        	)
		)

		generates primitive task

		(:action
			:parameters (?r - CleanerRobot ?c - capability)
			:precondition (and
				(hascapability ?r ?c)
			)
			:effect ()
		)
	*/

	map<string,vector<DecompositionPath>> at_decomposition_paths;
	map<string,vector<CompleteDecompositionPath>> at_complete_decomposition_paths;

	for(task at : abstract_tasks) {
		TDG t(at, abstract_tasks, primitive_tasks, methods, verbose);

		at_decomposition_paths[at.name] = t.retrieve_possible_decompositions();
		if(ihtn_output) {
			at_complete_decomposition_paths[at.name] = t.retrieve_possible_complete_decompositions();
		}
	}

	if(verbose) {
		print_at_paths_info(at_decomposition_paths);
		print_complete_at_paths_info(at_complete_decomposition_paths);
	}

	knowledge_manager->initialize_objects(sorts, high_level_loc_types, at_instances);
	knowledge_manager->initialize_world_state(init, init_functions, semantic_mapping, sorts);

	if(verbose) {
		print_world_state(init,init_functions);
	}

	AnnotManagerFactory annot_manager_factory;
	shared_ptr<AnnotManager> annot_manager_ptr = annot_manager_factory.create_annot_manager(knowledge_manager, gm, high_level_loc_types, at_instances);

	general_annot* gmannot = NULL;

	if(annot_manager_ptr->get_annot_manager_type() == FILEANNOTMANAGER) {
		FileKnowledgeAnnotManager* annot_manager = dynamic_cast<FileKnowledgeAnnotManager*>(annot_manager_ptr.get());
		
		FileKnowledgeManager* aux = dynamic_cast<FileKnowledgeManager*>(knowledge_manager.get());
		annot_manager->set_fk_manager(aux);

		gmannot = annot_manager->retrieve_gm_annot();
	}

	if(gmannot == NULL) {
		string gmannot_could_not_be_generated_error = "Could not generate Runtime Annotation Tree";

		throw std::runtime_error(gmannot_could_not_be_generated_error);
	}

	rename_at_instances_in_runtime_annot(gmannot, at_instances, gm);

	if(verbose) {
		print_runtime_annot_from_general_annot(gmannot);
	}

	MissionDecomposerFactory mission_decomposer_factory;
	shared_ptr<MissionDecomposer> mission_decomposer_ptr = mission_decomposer_factory.create_mission_decomposer(knowledge_manager, init, init_functions, at_decomposition_paths, at_instances, gmannot, gm, verbose, pretty_print);
	
	ATGraph mission_decomposition;

	if(mission_decomposer_ptr->get_mission_decomposer_type() == FILEMISSIONDECOMPOSER) {
		FileKnowledgeMissionDecomposer* mission_decomposer = dynamic_cast<FileKnowledgeMissionDecomposer*>(mission_decomposer_ptr.get());

		FileKnowledgeManager* aux = dynamic_cast<FileKnowledgeManager*>(knowledge_manager.get());
		mission_decomposer->set_fk_manager(aux);

		mission_decomposition = mission_decomposer->build_at_graph(gm_var_map, semantic_mapping);
	}

	if(verbose) {
		print_mission_decomposition(mission_decomposition); 
	}

	if(!ihtn_output) {
		if(output_info.at(0) == "FILE") {
			FileOutputGeneratorFactory output_gen_factory;

			pair<string,string> file_output_data = std::make_pair(output_info.at(1),output_info.at(2));
			std::shared_ptr<FileOutputGenerator> output_generator_ptr = output_gen_factory.create_file_output_generator(gm, mission_decomposition, init, init_functions, file_output_data, verbose, pretty_print);

			if(output_generator_ptr->get_file_output_generator_type() == XMLFILEOUTGEN) {
				XMLOutputGenerator* output_generator = dynamic_cast<XMLOutputGenerator*>(output_generator_ptr.get());

				output_generator->generate_instances_output(semantic_mapping, sorts, sort_definitions, predicate_definitions, gm_var_map, robot_related_sorts);
			} else if(output_generator_ptr->get_file_output_generator_type() == JSONFILEOUTGEN) {
				JSONOutputGenerator* output_generator = dynamic_cast<JSONOutputGenerator*>(output_generator_ptr.get());

				output_generator->generate_instances_output(semantic_mapping, sorts, sort_definitions, predicate_definitions, gm_var_map, robot_related_sorts);
			}
		}
	} else {
		map<string,CompleteDecompositionPath> decomposition_mapping;
		decomposition_mapping = map_complete_decompositions(mission_decomposition, at_complete_decomposition_paths);

		map<string,string> hddl_to_ocl_type_mapping;
		
		map<string,string>::iterator t_mapping_it;
		for(t_mapping_it = type_mapping.begin(); t_mapping_it != type_mapping.end(); t_mapping_it++) {
			hddl_to_ocl_type_mapping[t_mapping_it->second] = t_mapping_it->first;
		}

		IHTNGenerator ihtn_gen(gm, mission_decomposition, verbose, pretty_print, init, init_functions, high_level_loc_types, high_level_agent_types, hddl_to_ocl_type_mapping, decomposition_mapping);

		ihtn_gen.generate_ihtn(semantic_mapping, gm_var_map, robot_related_sorts_map);
	}
}