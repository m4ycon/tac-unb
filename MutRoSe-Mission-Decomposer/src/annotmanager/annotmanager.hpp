#ifndef __ANNOTMANAGER
#define __ANNOTMANAGER

#include <map>
#include <string>
#include <vector>

#include "../utils/annotmanagerutils.hpp"
#include "../knowledgemanager/knowledgemanager.hpp"
#include "../knowledgemanager/fileknowledgemanager.hpp"

enum annot_manager_type {FILEANNOTMANAGER};

class AnnotManager {
    public:
        virtual general_annot* retrieve_gm_annot() = 0;
        
        virtual void recursive_gm_annot_generation(general_annot* node_annot, std::vector<int> &vctr,  pt::ptree worlddb, int current_node, std::map<int,AchieveCondition> valid_forAll_conditions) = 0;

        void set_annot_manager_type(annot_manager_type amt);
        void set_gm(GMGraph g);
        void set_high_level_loc_types(std::vector<std::string> hllt);
        void set_at_instances(std::map<std::string,std::vector<AbstractTask>> atinst);
        void set_knowledge_unique_id(std::string knowledge_unique_id);

        void expand_node_vector(std::vector<int>& vctr, int current, int generated_instances);
        void expand_forall_annot(general_annot* node_annot, int generated_instances, std::string iterated_var, std::string iteration_var, std::vector<int>& vctr, int current, pt::ptree worlddb, std::map<int,AchieveCondition> valid_forAll_conditions);

        bool forall_goal_resolution(general_annot* node_annot, int current, int depth, std::map<int,AchieveCondition> valid_forAll_conditions, std::vector<int>& vctr, pt::ptree worlddb);
        virtual bool goal_node_resolution(general_annot* node_annot, int current_node, int depth, std::map<int,AchieveCondition>& valid_forAll_conditions, pt::ptree worlddb) = 0;

        void expand_annotation(general_annot* node_annot, int current_node, bool is_forAll_goal, int depth, std::vector<int>& vctr, std::map<int,AchieveCondition> valid_forAll_conditions, pt::ptree worlddb);

        annot_manager_type get_annot_manager_type();
    
    protected: 
        general_annot* gmannot;
        GMGraph gm;
        std::vector<std::string> high_level_loc_types;
        std::map<std::string,std::vector<AbstractTask>> at_instances;
        std::map<int,int> node_depths;
        std::map<std::string,pair<std::string,std::vector<pt::ptree>>> valid_variables;
        std::string knowledge_unique_id;

    private:
        annot_manager_type am_type;
};

class FileKnowledgeAnnotManager : public AnnotManager {
    public:
        general_annot* retrieve_gm_annot();

        void recursive_gm_annot_generation(general_annot* node_annot, std::vector<int> &vctr,  pt::ptree worlddb, int current_node, std::map<int,AchieveCondition> valid_forAll_conditions);
        
        bool goal_node_resolution(general_annot* node_annot, int current_node, int depth, std::map<int,AchieveCondition>& valid_forAll_conditions, pt::ptree worlddb);

        void set_fk_manager(FileKnowledgeManager* manager);

    private:
        FileKnowledgeManager* fk_manager;
};

class AnnotManagerFactory {
    public:
        std::shared_ptr<AnnotManager> create_annot_manager(std::shared_ptr<KnowledgeManager> k_manager, GMGraph gm, std::vector<std::string> high_level_loc_types, std::map<std::string,std::vector<AbstractTask>> at_instances);
};

general_annot* retrieve_runtime_annot(std::string id);

void recursive_fill_up_runtime_annot(general_annot* rannot, VertexData gm_node);
void recursive_child_replacement(general_annot* copy, general_annot* original);
void rename_at_instances_in_runtime_annot(general_annot* gmannot, std::map<std::string,std::vector<AbstractTask>> at_instances, GMGraph gm);
void recursive_at_instances_renaming(general_annot* rannot, std::map<std::string,int>& at_instances_counter, bool in_forAll, map<string,vector<AbstractTask>> at_instances, GMGraph gm);
void print_runtime_annot_from_general_annot(general_annot* rt);
void solve_query_statement(pt::ptree queried_tree, QueriedProperty q, GMGraph gm, int node_id, std::map<std::string,std::pair<std::string,std::vector<pt::ptree>>>& valid_variables);

std::string recursive_rt_annot_build(general_annot* rt);

std::map<std::string,std::variant<std::pair<std::string,std::string>,std::pair<std::vector<std::string>,std::string>>> get_annot_var_maps(std::map<std::string,pair<std::string,std::vector<pt::ptree>>> valid_variables, std::string knowledge_unique_id);

#endif