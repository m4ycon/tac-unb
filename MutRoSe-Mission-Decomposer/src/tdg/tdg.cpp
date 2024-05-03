#include "tdg.hpp"

#include <iostream>
#include <set>
#include <variant>

#include <boost/foreach.hpp>
#include <boost/algorithm/string.hpp>

#include "../utils/math_utils.hpp"

using namespace std;

/*
    Function: TDG
    Objective: Constructor for the TDG object. Here we generate a TDG and deal with the cycles 
    problem only when generating possible decompositions

    @ Input 1: The root abstract task of the TDG, which needs to be decomposed
    @ Input 2: The abstract tasks defined in HDDL
    @ Input 3: The primitive tasks defined in HDDL
    @ Input 4: The methods defined in HDDL
    @ Input 5: The verbose flag
    @ Output: void. The TDG object
*/ 
TDG::TDG(task root_abstract_task, vector<task> a_tasks, vector<task> p_tasks, vector<method> ms, bool verbose) {
    abstract_tasks = a_tasks;
    primitive_tasks = p_tasks;
    methods = ms;

    NodeData n;

    n.type = AT;
    n.t = root_abstract_task;
    n.parent = -1;

    vertex_t id = boost::add_vertex(n,tdg);

    root = id;
    tdg[id].id = id;
    n.id = id;

    add_task_path(n);

    this->verbose = verbose; 
}

/*
    Function: retrieve possible decompositions
    Objective: Find all possible decompositions of the root abstract task

    @ Output: A vector of the possible decompositions of the task. One must note that these are not necessarily valid in the current world state

    NOTE: Need to deal with cycles. Checking predicates is needed.
        - For each path starting in method m1 we need to have as initial state of the world the methods preconditions (if any).
            * If the method originating some path does not contain preconditions, we assume nothing about what was not told to us. This way
            if we find some predicate in action preconditions, we assume them to be true if they were not set by previous actions effects, since
            this means no harm because we are not generating a plan (just decomposing the task)
*/ 
vector<DecompositionPath> TDG::retrieve_possible_decompositions() {
    vector<int> depth_first_nodes = DFS_visit();

    vector<DecompositionPath> paths;

    vector<pair<string,string>> initial_vars = tdg[root].t.vars;
    vector<literal> world_state; //Empty since root is an AT and doesn't introduce preconditions and effects
    vector<pair<string,string>> variable_mapping; //Empty since we only have original root vars

    paths = decomposition_recursion(depth_first_nodes,0, initial_vars, world_state, variable_mapping);

    for(DecompositionPath& path : paths) {
        std::sort(path.fragments_to_expand.begin(), path.fragments_to_expand.end(), [](auto &left, auto &right) {
            return left.first.first < right.first.first;
        });
    }

    return paths;
}

/*
    Function: decomposition_recursion
    Objective: Recursively generate the decomposition based on a previously generated DFS nodes vector. World state is
    created based on previous preconditions and effects and variables are mapped accordingly to the original root abstract
    task variables.

    @ Input 1: The vector containing the TDG nodes visited using DFS
    @ Input 2: An index to the current element in the vector being considered
    @ Input 3: The original variables from the original root task
    @ Input 4: The world state generated until the moment
    @ Input 5: The variable mappings with respect to the original root abstract task variables
    @ Output: A vector of the decompositions generated at the current recursion depth

    NOTE: Every variable involved in the recursion must come from the root task
            - In this sense, variable substition must happen 
            - Inside every method we know which method variable refers to which original variable
            - Inside every action we also know which variable refers to which original variable
*/ 
vector<DecompositionPath> TDG::decomposition_recursion(vector<int> dfs_nodes, int current_pos, vector<pair<string,string>> original_vars, 
                                                    vector<literal>& world_state, vector<pair<string,string>> variable_mapping) {
    int node = dfs_nodes.at(current_pos);

    NodeData n = tdg[node];

    vector<DecompositionPath> generated_paths;
    if(n.type == M) {
        /*
            If any task in the methods subtasks cannot be executed, the method can't be used.
                - This needs checking in order to skip
                - This only works if we have an and decomposition of subtasks (we are assuming we only have and)
                - Also, we are assuming totally ordered methods
        */
        vector<vector<int>> possible_orderings = find_method_possible_orderings(n.m,n.children);

        if(verbose) {
            print_method_possible_orderings(possible_orderings, n);
        }

        bool expansion_needed = false;
        literal expansion_pred;
        variant<int,float> expansion_pred_sum = 0;
        int children_num = -1;

        vector<vector<DecompositionPath>> child_paths;
        for(vector<int> ordering : possible_orderings) {
            bool ordering_exec = true;
            vector<literal> world_state_copy = world_state;

            unsigned int ordering_index = 0;
            set<int> considered_tasks;
            for(int c : ordering) {
                vector<int>::iterator it = std::find(dfs_nodes.begin(),dfs_nodes.end(),c);
                int c_pos = std::distance(dfs_nodes.begin(),it);

                task child_task = tdg[c].t;
                vector<pair<string,string>> child_var_mapping;

                int plan_step_index = 0;
                for(auto subtask : n.m.ps) {
                    if(subtask.task == child_task.name && considered_tasks.find(plan_step_index) == considered_tasks.end()) {
                        int index = 0;
                        for(string var : subtask.args) {
                            string aux;
                            for(auto mapping : variable_mapping) {
                                if(mapping.first == var) { //Found mapping from methods
                                    aux = mapping.second;
                                    child_var_mapping.push_back(make_pair(child_task.vars.at(index).first,aux));

                                    break;
                                }
                            }

                            index++;
                        }

                        considered_tasks.insert(plan_step_index);
                        break;
                    }

                    plan_step_index++;
                }

                /*
                    If we have an action, check for preconditions based on the current world state and only call decomposition if they are met
                */             
                pair<bool,pair<literal,bool>> checking_result;
                checking_result = check_predicates(child_task, n.m, child_var_mapping, variable_mapping, c, world_state_copy, make_pair(ordering,ordering_index));

                bool executable = checking_result.first;
                
                if(!expansion_needed) {
                    expansion_needed = checking_result.second.second;
                    
                    if(expansion_needed) {
                        if(child_task.name.find(method_precondition_action_name) != string::npos) {
                            bool hasDecrease = false;
                            for(unsigned int i = ordering_index+1; i < ordering.size(); i++) {
                                task ch = tdg[i].t;

                                for(literal ceff : ch.costExpression) {
                                    if(ceff.isCostChangeExpression && !ceff.isAssignCostChangeExpression) {
                                        hasDecrease = true;
                                    }
                                }
                            }

                            if(hasDecrease) {
                                children_num = n.m.ps.size();

                                expansion_pred = checking_result.second.first;
                            } else {
                                expansion_needed = false;
                            }
                        } else {
                            expansion_needed = false;
                        }
                    }

                    if(expansion_needed) {
                        /*
                            Check if expansion is possible. For this to be true, the sum of decreases and increases of the expansion predicate in the possible ordering must be
                            negative (since we only have greater than preconditions for now)
                        */
                        for(unsigned int i = ordering_index+1; i < ordering.size(); i++) {
                            task ch = tdg[i].t;

                            vector<pair<string,string>> ch_var_mapping;
                            for(auto subtask : n.m.ps) {
                                if(subtask.task == ch.name) {
                                    int index = 0;
                                    for(string var : subtask.args) {
                                        string aux;
                                        for(auto mapping : variable_mapping) {
                                            if(mapping.first == var) { //Found mapping from methods
                                                aux = mapping.second;
                                                ch_var_mapping.push_back(make_pair(ch.vars.at(index).first,aux));
                                                break;
                                            }
                                        }

                                        index++;
                                    }

                                    break;
                                }
                            }

                            variant<int,float> ch_sum = check_expansion_predicate_assignments(ch, ch_var_mapping, expansion_pred);

                            if(holds_alternative<int>(expansion_pred_sum)) {
                                int current_sum = std::get<int>(expansion_pred_sum);

                                if(holds_alternative<int>(ch_sum)) {
                                    expansion_pred_sum = current_sum + std::get<int>(ch_sum);
                                } else {
                                    expansion_pred_sum = static_cast<float>(current_sum) + std::get<float>(ch_sum);
                                }
                            } else {
                                float current_sum = std::get<float>(expansion_pred_sum);

                                if(holds_alternative<int>(ch_sum)) {
                                    expansion_pred_sum = current_sum + static_cast<float>(std::get<int>(ch_sum));
                                } else {
                                    expansion_pred_sum = current_sum + std::get<float>(ch_sum);
                                }
                            }
                        }

                        bool expansion_error = false;
                        if(holds_alternative<int>(expansion_pred_sum)) {
                            if(std::get<int>(expansion_pred_sum) >= 0) expansion_error = true;
                        } else {
                            float exp_sum = std::get<float>(expansion_pred_sum);

                            if(greater_than_floats(exp_sum, 0.0) || compare_floats(0.0, exp_sum)) expansion_error = true;
                        }

                        if(expansion_error) {
                            string infinite_expansion_error = "Infinite expansion generated with method [" + n.m.name + "]";

                            throw std::runtime_error(infinite_expansion_error); 
                        }
                    }
                }

                if(executable) {
                    vector<DecompositionPath> aux = decomposition_recursion(dfs_nodes,c_pos,original_vars,world_state_copy,child_var_mapping);

                    child_paths.push_back(aux);
                } else {
                    ordering_exec = false;
                    break;
                }

                ordering_index++;
            }

            // Here is one place where we need to check for expansion_needed flags
            vector<DecompositionPath> ordering_paths;
            if(ordering_exec) {
                for(auto aux : child_paths) {
                    if(aux.size() > 0) {
                        vector<DecompositionPath> g_paths_temp = ordering_paths;
                        ordering_paths.clear();

                        for(auto p : aux) {
                            if(g_paths_temp.size() > 0) {
                                for(auto g_pt : g_paths_temp) {
                                    DecompositionPath p_temp = p;
                                    p_temp.decomposition.insert(p_temp.decomposition.begin(),g_pt.decomposition.begin(),g_pt.decomposition.end());

                                    // Adjust indexes of task fragments that will need expansion
                                    if(p_temp.needs_expansion) {
                                        int index_diff = g_pt.decomposition.size()-1;

                                        for(auto& fragment : p_temp.fragments_to_expand) {
                                            fragment.first.first += index_diff;
                                            fragment.first.second += index_diff;
                                        }
                                    }

                                    ordering_paths.push_back(p_temp);
                                }
                            } else {
                                ordering_paths.push_back(p);
                            }
                        }
                    }
                }
            }

            /*
                In order for an expansion to be necessary, the method needs to be in a cycle

                -> Right now this cycle needs to be with its parent, how do we generalize this?
                    - Also, do we need to generalize this?
            */
            NodeData parent = tdg[n.parent];
            if(parent.belongs_to_cycles) {
                bool has_cycle_with_parent = false;
                for(int link : parent.cycle_links) {
                    if(link == n.id) {
                        has_cycle_with_parent = true;
                        break;
                    }
                }

                if(!has_cycle_with_parent) {
                    expansion_needed = false;
                }        
            } else {
                expansion_needed = false;
            }

            if(expansion_needed) {
                for(DecompositionPath& path : ordering_paths) {
                    path.needs_expansion = expansion_needed;
                    path.fragments_to_expand.insert(path.fragments_to_expand.begin(), make_pair(make_pair(0,children_num-2),expansion_pred));
                    path.expansion_decrease = expansion_pred_sum;
                }
            }

            generated_paths.insert(generated_paths.end(),ordering_paths.begin(),ordering_paths.end());

            child_paths.clear();
        }
    } else if(n.type == AT) {
        vector<literal> initial_world_state = world_state;
        for(int c : n.children) {
            vector<int>::iterator it = std::find(dfs_nodes.begin(),dfs_nodes.end(),c);
            int c_pos = std::distance(dfs_nodes.begin(),it);

            method child_method = tdg[c].m;
            vector<pair<string,string>> child_var_mapping;
            if(n.id == root) { //If we are dealing with the root
                int index = 0;
                for(string arg : child_method.atargs) {
                    child_var_mapping.push_back(make_pair(arg,original_vars.at(index).first));
                    index++; 
                }
            } else {
                int index = 0;
                for(string arg : child_method.atargs) {
                    child_var_mapping.push_back(make_pair(arg,variable_mapping.at(index).second)); //Mapping to original vars
                    index++;
                }
            }

            /*
                Methods preconditions are transformed into actions, so we only check for preconditions when checking methods children
            */
            vector<DecompositionPath> aux = decomposition_recursion(dfs_nodes,c_pos,original_vars,world_state,child_var_mapping);
            generated_paths.insert(generated_paths.end(),aux.begin(),aux.end());

            world_state = initial_world_state;
        }
    } else if(n.type == PT) {
        change_world_state(n.t,world_state,variable_mapping);
        
        DecompositionPath t;
        
        task nt = n.t;
        variable_renaming(nt,variable_mapping);
        t.decomposition.push_back(nt);

        generated_paths.push_back(t);
    }

    return generated_paths;
}

vector<CompleteDecompositionPath> TDG::retrieve_possible_complete_decompositions() {
    vector<int> depth_first_nodes = DFS_visit();

    vector<CompleteDecompositionPath> paths;

    vector<pair<string,string>> initial_vars = tdg[root].t.vars;
    vector<literal> world_state; //Empty since root is an AT and doesn't introduce preconditions and effects
    vector<pair<string,string>> variable_mapping; //Empty since we only have original root vars

    paths = complete_decomposition_recursion(depth_first_nodes, 0, initial_vars, world_state, variable_mapping, -1, 0);

    return paths;
}

/*
    Function: complete_decomposition_recursion
    Objective: Recursive function to generate complete decomposition paths. Complete decomposition paths are HTN decomposed paths which contain methods and abstract tasks
    in addition to actions

    @ Input 1: The TDG nodes in a DFS order
    @ Input 2: The current node position in the DFS nodes vector
    @ Input 3: The variables of the abstract task that is the root of the TDG
    @ Input 4: The current non-ground world state
    @ Input 5: The variable mappings with the original variables
    @ Input 6: The node's parent in the TDG
    @ Input 7: The current node's index in the TDG
    @ Output: The vector of the complete decomposition paths of the TDG
*/ 
vector<CompleteDecompositionPath> TDG::complete_decomposition_recursion(vector<int> dfs_nodes, int current_pos, vector<pair<string,string>> original_vars, vector<literal>& world_state, vector<pair<string,string>> variable_mapping, int parent, int current_index) {
    int node = dfs_nodes.at(current_pos);

    NodeData n = tdg[node];

    int node_index = current_index;

    vector<CompleteDecompositionPath> generated_paths;
    if(n.type == M) {
        /*
            If any task in the methods subtasks cannot be executed, the method can't be used.
                - This needs checking in order to skip
                - This only works if we have an and decomposition of subtasks (we are assuming we only have and)
                - Also, we are assuming totally ordered methods
        */
        vector<vector<int>> possible_orderings = find_method_possible_orderings(n.m,n.children);

        vector<vector<CompleteDecompositionPath>> child_paths;
        for(vector<int> ordering : possible_orderings) {
            bool ordering_exec = true;
            vector<literal> world_state_copy = world_state;

            unsigned int ordering_index = 0;
            set<int> considered_tasks;
            int child_index = current_index+1;
            for(int c : ordering) {
                vector<int>::iterator it = std::find(dfs_nodes.begin(),dfs_nodes.end(),c);
                int c_pos = std::distance(dfs_nodes.begin(),it);

                task child_task = tdg[c].t;
                vector<pair<string,string>> child_var_mapping;

                int plan_step_index = 0;
                for(auto subtask : n.m.ps) {
                    if(subtask.task == child_task.name && considered_tasks.find(plan_step_index) == considered_tasks.end()) {
                        int index = 0;
                        for(string var : subtask.args) {
                            string aux;
                            for(auto mapping : variable_mapping) {
                                if(mapping.first == var) { //Found mapping from methods
                                    aux = mapping.second;
                                    child_var_mapping.push_back(make_pair(child_task.vars.at(index).first,aux));

                                    break;
                                }
                            }

                            index++;
                        }

                        considered_tasks.insert(plan_step_index);
                        break;
                    }

                    plan_step_index++;
                }

                /*
                    If we have an action, check for preconditions based on the current world state and only call decomposition if they are met
                */             
                pair<bool,pair<literal,bool>> checking_result;
                checking_result = check_predicates(child_task, n.m, child_var_mapping, variable_mapping, c, world_state_copy, make_pair(ordering,ordering_index));

                bool executable = checking_result.first;

                if(executable) {
                    vector<CompleteDecompositionPath> aux = complete_decomposition_recursion(dfs_nodes,c_pos,original_vars,world_state_copy,child_var_mapping, node_index, child_index);

                    child_paths.push_back(aux);
                } else {
                    ordering_exec = false;
                    break;
                }

                ordering_index++;
                child_index++;
            }

            // Here is one place where we need to check for expansion_needed flags
            vector<CompleteDecompositionPath> ordering_paths;
            if(ordering_exec) {
                for(auto aux : child_paths) {
                    if(aux.size() > 0) {
                        vector<CompleteDecompositionPath> g_paths_temp = ordering_paths;
                        ordering_paths.clear();

                        for(auto p : aux) {
                            if(g_paths_temp.size() > 0) {
                                for(auto g_pt : g_paths_temp) {
                                    CompleteDecompositionPath p_temp = p;
                                    p_temp.decomposition.insert(p_temp.decomposition.begin(),g_pt.decomposition.begin(),g_pt.decomposition.end());

                                    ordering_paths.push_back(p_temp);
                                }
                            } else {
                                ordering_paths.push_back(p);
                            }
                        }
                    }
                }
            }

            generated_paths.insert(generated_paths.end(),ordering_paths.begin(),ordering_paths.end());
            for(CompleteDecompositionPath& child_path : generated_paths) {
                method m = n.m;
                method_variable_renaming(m, variable_mapping);

                DecompositionNode m_node;
                m_node.content = m;
                m_node.id = node_index;
                m_node.parent = parent;
                child_path.decomposition.insert(child_path.decomposition.begin(), m_node);
            }

            child_paths.clear();
        }
    } else if(n.type == AT) {
        vector<literal> initial_world_state = world_state;

        int child_index = current_index+1;
        for(int c : n.children) {
            vector<int>::iterator it = std::find(dfs_nodes.begin(),dfs_nodes.end(),c);
            int c_pos = std::distance(dfs_nodes.begin(),it);

            method child_method = tdg[c].m;
            vector<pair<string,string>> child_var_mapping;
            if(n.id == root) { //If we are dealing with the root
                int index = 0;
                for(string arg : child_method.atargs) {
                    child_var_mapping.push_back(make_pair(arg,original_vars.at(index).first));
                    index++; 
                }
            } else {
                int index = 0;
                for(string arg : child_method.atargs) {
                    child_var_mapping.push_back(make_pair(arg,variable_mapping.at(index).second)); //Mapping to original vars
                    index++;
                }
            }

            /*
                Methods preconditions are transformed into actions, so we only check for preconditions when checking methods children
            */
            vector<CompleteDecompositionPath> aux = complete_decomposition_recursion(dfs_nodes,c_pos,original_vars,world_state,child_var_mapping,node_index,child_index);
            generated_paths.insert(generated_paths.end(),aux.begin(),aux.end());

            world_state = initial_world_state;
            child_index++;
        }

        for(CompleteDecompositionPath& path : generated_paths) {
            task t = n.t;
            variable_renaming(t, variable_mapping);

            DecompositionNode at_node;
            at_node.content = t;
            at_node.id = node_index;
            at_node.parent = parent;
            at_node.is_primitive_task_node = false;
            
            path.decomposition.insert(path.decomposition.begin(), at_node);
        }
    } else if(n.type == PT) {
        change_world_state(n.t,world_state,variable_mapping);
        
        CompleteDecompositionPath t;
        
        task nt = n.t;
        variable_renaming(nt,variable_mapping);

        DecompositionNode pt;
        pt.content = nt;
        pt.id = node_index;
        pt.parent = parent;
        pt.is_primitive_task_node = true;

        t.decomposition.push_back(pt);

        generated_paths.push_back(t);
    }

    return generated_paths;
}

/*
    Function: add_method_path
    Objective: Add method path in TDG by iterating through method decomposition tasks
    and further adding these tasks paths to the TDG

    @ Input: The method node being considered
    @ Output: void. The path will be added to the TDG object calling the function
*/ 
void TDG::add_method_path(NodeData m) {
    for(plan_step ps : m.m.ps) {
        NodeData t_node;
        task n_task;

        bool primitive = true;

        for(task at : abstract_tasks) {
            if(at.name == ps.task) {
                primitive = false;
                n_task = at;
                break;
            }
        }

        if(primitive) {
            for(task pt : primitive_tasks) {
                if(pt.name == ps.task) {
                    n_task = pt;
                    break;
                }
            }
            t_node.type = PT;
        } else {
            t_node.type = AT;
        }

        t_node.t = n_task;

        pair<bool,int> cycle = make_pair(false,-1);
        if(!primitive) {
            cycle = check_cycle(m.id,t_node);
        }

        if(!cycle.first) {
            vertex_t id = boost::add_vertex(t_node,tdg);

            tdg[id].id = id;

            t_node.id = id;

            add_edge(m.id, tdg[id].id);

            add_task_path(t_node);
        } else {
            add_edge(m.id, cycle.second);
        }
    }
}

/*
    Function: add_task_path
    Objective: Add a task (primitive or abstract) to the TDG. Find methods that decompose the task 
    and from them generate the complete path resulting from their decomposition

    @ Input: The task node being considered
    @ Output: void. The path will be added to the TDG object calling the function
*/ 
void TDG::add_task_path(NodeData t) {
    if(t.type != PT) {
        for(method m : methods) {
            if(m.at == t.t.name) {
                NodeData m_node;
                m_node.type = M;
                m_node.m = m;
                
                vertex_t id = boost::add_vertex(m_node,tdg);

                tdg[id].id = id;

                m_node.id = id;

                add_edge(t.id, tdg[id].id);

                add_method_path(m_node);
            }
        }
    }
}

/*
    Function: add_edge
    Objective: Add an edge between TDG nodes

    @ Input 1: The source node ID
    @ Input 2: The target node ID
    @ Output: void. The edge will be added in the TDG
*/ 
void TDG::add_edge(int s_id, int t_id) {
    EData edge;

    edge.source = s_id;
    edge.target = t_id;

    if(tdg[s_id].type == M) {
        edge.type = OOR;
    } else {
        edge.type = AAND;
    }

    tdg[s_id].children.push_back(t_id);
    tdg[t_id].parent = s_id;

    boost::add_edge(s_id,t_id,edge,tdg);
}

/*
    Function: DFS_visit
    Objective: Perform a Depth-First visit in the TDG and return a vector of node ID's
    generated through this visit

    @ Output: The vector of node ID's
*/ 
vector<int> TDG::DFS_visit() {
    auto indexmap = boost::get(boost::vertex_index, tdg);
    auto colormap = boost::make_vector_property_map<boost::default_color_type>(indexmap);

    TDGDFSVisitor vis;
    boost::depth_first_search(tdg, vis, colormap, 0);

    vector<int> vctr = vis.GetVector();

    return vctr;
}

/*
    Function: print_edges
    Objective: Print TDG edges

    @ Output: void. The edges will be printed in a terminal
*/ 
void TDG::print_edges() {
    boost::graph_traits<TDGraph>::edge_iterator it, end;
	
	for(tie(it,end) = boost::edges(tdg);it != end;++it) {
        NodeData s = tdg[boost::source(*it, tdg)];
        NodeData t = tdg[boost::target(*it, tdg)];

        if(s.type != M) {
		    std::cout << s.t.name << " -> ";
        } else {
            std::cout << s.m.name << " -> ";
        }

        if(t.type != M) {
		    std::cout << t.t.name << endl;
        } else {
            std::cout << t.m.name << endl;
        }
	}
}

/*
    Function: print_method_possible_orderings
    Objective: Print all the possible orderings for a method

    @ Input 1: The possible orderings for the method
    @ Input 2: The method node being considered 
    @ Output: void. The possible orderings will be printed in a terminal
*/ 
void TDG::print_method_possible_orderings(vector<vector<int>> possible_orderings, NodeData n) {
    cout << "Possible orderings for method " << n.m.name << ":" << endl;
    for(vector<int> ordering : possible_orderings) {
        for(int t : ordering) {
            cout << tdg[t].t.name << " ";
        }
        cout << endl;
    }
}

/*
    Function: add_task_path
    Objective: Here we have a method that is already in the graph and a task that is to be introduced to the graph
    if it is not already in it. In case it is, return true since we have a cycle.

    @ Input 1: The method ID in the TDG
    @ Input 2: The task node in the TDG
    @ Output: A pair containing a flag if we have a cycle or not and the ID of the task to which this cycle refers to.

    NOTE: Task t must be a parent in some degree of m in order for us to consider a cycle
*/ 
pair<bool,int> TDG::check_cycle(int m_id, NodeData t) {
    pair<bool,int> cycle = make_pair(false,-1);
    NodeData m = tdg[m_id];

    NodeData* current_node = &tdg[m_id];
    bool at_root = false;
    while(at_root == false) {
        if(current_node->parent == -1) at_root = true;

        if(current_node->type == AT) {
            if(current_node->t.name == t.t.name) {
                current_node->belongs_to_cycles = true;
                current_node->cycle_links.push_back(m.id);
                cycle.first = true;
                cycle.second = current_node->id;

                break;
            }
        }

        if(current_node->parent != -1) {
            current_node = &tdg[current_node->parent];
        }
    }

    return cycle;
}

/*
    Function: check_predicates
    Objective: Here we check if all predicates for a task hold in the current world state. One must note that since
    the world state is generated based on the root abstract task of the TDG and we must rename variables.

    @ Input 1: The task to be evaluated
    @ Input 2: The variable mapping between the task and the TDG's root abstract task
    @ Input 3: The task ID in the TDG
    @ Input 4: The world state
    @ Input 5: The task index in the TDG
    @ Output: A boolean flag indicating if predicates hold
*/ 
pair<bool,pair<literal,bool>> TDG::check_predicates(task t, method parent_method, vector<pair<string,string>> t_var_mapping, vector<pair<string,string>> global_var_mapping, int t_id, vector<literal>& world_state, pair<vector<int>,int> ordering_info) {
    vector<literal> t_precs, precs_to_add;

    t_precs = t.prec;

    //Rename predicates variables
    for(literal& prec : t_precs) {
        for(string& arg : prec.arguments) {
            for(pair<string,string>& arg_mapping : t_var_mapping) {
                if(arg_mapping.first == arg) {
                    arg = arg_mapping.second;
                    break;
                }
            }
        }
    }

    bool executable = true;
    bool expansion_needed = false;
    literal expansion_pred;
    if(tdg[t_id].type == PT) {
        for(literal& prec : t_precs) {
            bool found_prec = false;
            for(literal& state : world_state) {
                bool same_predicate = is_same_predicate(prec, state);

                if(same_predicate) { //Dealing with same predicate with same arguments
                    found_prec = true;
                    if(prec.isComparisonExpression) {
                        string comparison_op = prec.comparison_op_and_value.first;
                        variant<int,float> comparison_value = prec.comparison_op_and_value.second;

                        if(comparison_op == equal_comparison_op) {
                            if(holds_alternative<int>(state.costValue)) {
                                int state_value = std::get<int>(state.costValue);

                                if(holds_alternative<int>(comparison_value)) {
                                    executable = (state_value == std::get<int>(comparison_value));
                                } else {
                                    executable = compare_int_and_float(state_value, std::get<float>(comparison_value));
                                }
                            } else {
                                float state_value = std::get<float>(state.costValue);

                                if(holds_alternative<int>(comparison_value)) {
                                    executable = compare_int_and_float(std::get<int>(comparison_value), state_value);
                                } else {
                                    executable = compare_floats(state_value, std::get<float>(comparison_value));
                                }
                            }
                        } else if(comparison_op == greater_comparison_op) {
                            if(holds_alternative<int>(state.costValue)) {
                                int state_value = std::get<int>(state.costValue);

                                if(holds_alternative<int>(comparison_value)) {
                                    executable = (state.costValue > comparison_value);
                                } else {
                                    executable = greater_than_int_and_float(state_value, std::get<float>(comparison_value));
                                }
                            } else {
                                float state_value = std::get<float>(state.costValue);

                                if(holds_alternative<int>(comparison_value)) {
                                    executable = greater_than_float_and_int(std::get<int>(comparison_value), state_value);
                                } else {
                                    executable = greater_than_floats(state_value, std::get<float>(comparison_value));
                                }
                            }
                        }
                    } else {
                        if(!((prec.positive && state.positive) || (!prec.positive && !state.positive))) {
                            executable = false;
                        }
                    }

                    break;
                }
            }

            if(!found_prec) {
                if(prec.isComparisonExpression) {
                    literal l = prec;
                    
                    string comparison_op = l.comparison_op_and_value.first;
                    variant<int,float> comparison_value = l.comparison_op_and_value.second;
                    if(l.isComparisonExpression) {
                        if(comparison_op == equal_comparison_op) {
                            l.costValue = comparison_value;
                        } else if(comparison_op == greater_comparison_op) {
                            expansion_needed = true;
                            expansion_pred = l;

                            variant<int,float> expansion_offset = 0;
                            for(unsigned int i = ordering_info.second+1; i < ordering_info.first.size(); i++) {
                                task ch = tdg[i].t;

                                vector<pair<string,string>> ch_var_mapping;
                                for(auto subtask : parent_method.ps) {
                                    if(subtask.task == ch.name) {
                                        int index = 0;
                                        for(string var : subtask.args) {
                                            string aux;
                                            for(auto mapping : global_var_mapping) {
                                                if(mapping.first == var) { //Found mapping from methods
                                                    aux = mapping.second;
                                                    ch_var_mapping.push_back(make_pair(ch.vars.at(index).first,aux));
                                                    break;
                                                }
                                            }

                                            index++;
                                        }

                                        break;
                                    }
                                }

                                variant<int,float> ch_sum = check_predicate_assignments(ch, ch_var_mapping, expansion_pred, expansion_offset, world_state);

                                if(holds_alternative<int>(expansion_offset)) {
                                    int current_sum = std::get<int>(expansion_offset);

                                    if(holds_alternative<int>(ch_sum)) {
                                        expansion_offset = current_sum + std::get<int>(ch_sum);
                                    } else {
                                        expansion_offset = static_cast<float>(current_sum) + std::get<float>(ch_sum);
                                    }
                                } else {
                                    float current_sum = std::get<float>(expansion_offset);

                                    if(holds_alternative<int>(ch_sum)) {
                                        expansion_offset = current_sum + static_cast<float>(std::get<int>(ch_sum));
                                    } else {
                                        expansion_offset = current_sum + std::get<float>(ch_sum);
                                    }
                                }
                            }

                            if(holds_alternative<int>(comparison_value)) {
                                int comp_val = std::get<int>(comparison_value);

                                if(holds_alternative<int>(expansion_offset)) {
                                    l.costValue = comp_val + std::get<int>(expansion_offset)*(-1);
                                } else {
                                    l.costValue = static_cast<float>(comp_val) + std::get<float>(expansion_offset)*(-1);
                                }
                            } else {
                                float comp_val = std::get<float>(comparison_value);
                                
                                if(holds_alternative<int>(expansion_offset)) {
                                    l.costValue = comp_val + static_cast<float>(std::get<int>(expansion_offset))*(-1);
                                } else {
                                    l.costValue = comp_val + std::get<float>(expansion_offset)*(-1);
                                }
                            }
                        }
                    }

                    l.isCostChangeExpression = true;

                    precs_to_add.push_back(l);
                } else {
                    precs_to_add.push_back(prec);
                }
            }
        }

        if(executable) {
            for(literal prec : precs_to_add) {
                world_state.push_back(prec);
            }
        }
    }

    return make_pair(executable,make_pair(expansion_pred,expansion_needed));
}

/*
    Function: change_world state
    Objective: Here we change the world state based on the effects of a task. We must consider the variable mapping
    with respect to the TDG's root abstract task

    @ Input 1: The task to be evaluated
    @ Input 2: A reference to the world state
    @ Input 3: The variable mapping with respect to the TDG's root abstract task
    @ Output: Void. The reference to the world state is changed
*/ 
void TDG::change_world_state(task t, vector<literal>& world_state, vector<pair<string,string>> variable_mapping) {
    vector<literal> t_effs = t.eff;
    vector<literal> t_costexp = t.costExpression;

    //Rename predicates variables
    for(literal& eff : t_effs) {
        for(string& arg : eff.arguments) {
            for(pair<string,string>& arg_mapping : variable_mapping) {
                if(arg_mapping.first == arg) {
                    arg = arg_mapping.second;
                    break;
                }
            }
        }
    }
    for(literal& cexp : t_costexp) {
        for(string& arg : cexp.arguments) {
            for(pair<string,string>& arg_mapping : variable_mapping) {
                if(arg_mapping.first == arg) {
                    arg = arg_mapping.second;
                    break;
                }
            }
        }
    }

    for(literal& eff : t_effs) {
        bool found_pred = false;
        vector<literal>::iterator state;
        for(state = world_state.begin();state != world_state.end();++state) {
            bool same_predicate = is_same_predicate(*state, eff);

            if(same_predicate) { //Dealing with same predicate with same arguments
                found_pred = true;

                if(((eff.positive && !state->positive) || (!eff.positive && state->positive))) {
                    state->positive = eff.positive;
                }

                break;
            }
        }

        if(!found_pred) { // We don't need to change cost change expressions, we only care about their cost values
            world_state.push_back(eff);
        }
    }

    for(literal& cexp : t_costexp) {
        bool found_pred = false;
        vector<literal>::iterator state;
        for(state = world_state.begin();state != world_state.end();++state) {
            bool same_predicate = is_same_predicate(*state, cexp);

            if(same_predicate) { //Dealing with same predicate with same arguments
                found_pred = true;

                if(cexp.isAssignCostChangeExpression) {
                    state->costValue = cexp.costValue;
                } else {
                    if(holds_alternative<int>(state->costValue)) {
                        int state_value = std::get<int>(state->costValue);

                        if(holds_alternative<int>(cexp.costValue)) {
                            state->costValue = state_value + std::get<int>(cexp.costValue);
                        } else {
                            state->costValue = static_cast<float>(state_value) + std::get<float>(cexp.costValue);
                        }
                    } else {
                        float state_value = std::get<float>(state->costValue);

                        if(holds_alternative<int>(cexp.costValue)) {
                            state->costValue = state_value + static_cast<float>(std::get<int>(cexp.costValue));
                        } else {
                            state->costValue = state_value + std::get<float>(cexp.costValue);
                        }
                    }
                }

                state->isCostChangeExpression = true;

                break;
            } 
        }

        if(!found_pred) { // We don't need to change cost change expressions, we only care about their cost values
            world_state.push_back(cexp);
        }
    }
}

/*
    Function: variable_renaming
    Objective: Rename variables of a specific task

    @ Input 1: A reference to the task being considered
    @ Input 2: The variable mapping between the task and the TDG's root abstract task
    @ Output: Void. The reference to the task is modified
*/ 
void TDG::variable_renaming(task& t, vector<pair<string,string>> var_mapping) {
    //Rename preconditions
    for(literal& prec : t.prec) {
        for(string& arg : prec.arguments) {
            for(pair<string,string>& arg_mapping : var_mapping) {
                if(arg_mapping.first == arg) {
                    arg = arg_mapping.second;
                    break;
                }
            }
        }
    }

    //Rename effects
    for(literal& eff : t.eff) {
        for(string& arg : eff.arguments) {
            for(pair<string,string>& arg_mapping : var_mapping) {
                if(arg_mapping.first == arg) {
                    arg = arg_mapping.second;
                    break;
                }
            }
        }
    }

    for(literal& cexp : t.costExpression) {
        for(string& arg : cexp.arguments) {
            for(pair<string,string>& arg_mapping : var_mapping) {
                if(arg_mapping.first == arg) {
                    arg = arg_mapping.second;
                    break;
                }
            }
        }
    }

    //Rename task variables
    for(pair<string,string>& var : t.vars) {
        for(pair<string,string>& arg_mapping : var_mapping) {
            if(arg_mapping.first == var.first) {
                var.first = arg_mapping.second;
                break;
            }
        }
    }
}

void TDG::method_variable_renaming(method& m, vector<pair<string,string>> var_mapping) {
    //Rename method variables
    for(pair<string,string>& var : m.vars) {
        for(pair<string,string>& arg_mapping : var_mapping) {
            if(arg_mapping.first == var.first) {
                var.first = arg_mapping.second;
                break;
            }
        }
    }
}

/*
    Function: find_method_possible_orderings
    Objective: Find the possible orderings for a method's decomposition

    @ Input 1: The method being considered
    @ Input 2: The ID's of this method's children nodes in the TDG
    @ Output: The vector of the possible orderings by means of task ID's
*/ 
vector<vector<int>> TDG::find_method_possible_orderings(method m, vector<int> children) {
    map<string,int> plan_step_id_map;

    set<int> considered_children;
    for(plan_step ps : m.ps) {
        for(int c : children) {
            if(tdg[c].t.name == ps.task && considered_children.find(c) == considered_children.end()) {
                plan_step_id_map[ps.id] = c;
                considered_children.insert(c);
                break;
            }
        }
    }

    map<int,set<int>> precedences_map;
    for(pair<string,string> o : m.ordering) {
        if(precedences_map.find(plan_step_id_map[o.first]) == precedences_map.end()) {
            set<int> precedence;
            precedence.insert(plan_step_id_map[o.second]);
            precedences_map[plan_step_id_map[o.first]] = precedence;
        } else {
            precedences_map[plan_step_id_map[o.first]].insert(plan_step_id_map[o.second]);
        }
    }

    vector<vector<int>> possible_orderings;

    set<int> children_set;
    for(int c : children) {
        children_set.insert(c);
    }

    possible_orderings = recursive_method_possible_ordering(precedences_map, possible_orderings, children_set);

    return possible_orderings;
}

/*
    Function: recursive_method_possible_orderings
    Objective: Recursive generations of a method's possible orderings of decomposition

    @ Input 1: The map of precendence constraints given in HDDL
    @ Input 2: The orderings generated so far
    @ Input 3: The task ID's to be inserted
    @ Output: The possible orderings generated in this depth of the recursion
*/ 
vector<vector<int>> TDG::recursive_method_possible_ordering(map<int,set<int>> precedence_map, vector<vector<int>> current_orderings, set<int> values_to_insert) {
    vector<vector<int>> new_orderings;

    if(values_to_insert.size() > 0) {
        for(int val : values_to_insert) {
            bool can_insert_value = true;
            map<int,set<int>>::iterator precedences_iterator;
            for(precedences_iterator = precedence_map.begin();precedences_iterator != precedence_map.end();++precedences_iterator) {
                if(precedences_iterator->first != val) {
                    /*
                        If we have a value that is involved in a precedence constraint with the current value being checked and it was not inserted we
                        cannot insert the current value yet
                    */
                    if(values_to_insert.find(precedences_iterator->first) != values_to_insert.end()) {
                        if(precedences_iterator->second.find(val) != precedences_iterator->second.end()) {
                            can_insert_value = false;
                            break;
                        }
                    }
                }
            }

            if(can_insert_value) {
                set<int> updated_values_to_insert = values_to_insert;
                updated_values_to_insert.erase(val);

                vector<vector<int>> updated_current_orderings = current_orderings;

                if(current_orderings.size() > 0) {
                    for(vector<int>& ordering : updated_current_orderings) {
                        ordering.push_back(val);
                    }
                } else {
                    vector<int> first_ordering;
                    first_ordering.push_back(val);
                    updated_current_orderings.push_back(first_ordering);
                }

                vector<vector<int>> aux = recursive_method_possible_ordering(precedence_map,updated_current_orderings,updated_values_to_insert);
                for(vector<int> o : aux) {
                    new_orderings.push_back(o);
                }
            }
        }
    } else {
        new_orderings = current_orderings;
    }

    return new_orderings;
}

variant<int,float> TDG::check_expansion_predicate_assignments(task t, vector<pair<string,string>> var_mapping, literal expansion_pred) {
    variable_renaming(t, var_mapping);

    variant<int,float> sum_of_assignments = 0;

    for(literal eff : t.costExpression) {
        if(holds_alternative<int>(sum_of_assignments)) {
            int sum = std::get<int>(sum_of_assignments);

            if(eff.isCostChangeExpression && !eff.isAssignCostChangeExpression) {
                if(is_same_predicate(eff,expansion_pred)) {
                    if(holds_alternative<int>(eff.costValue)) {
                        sum_of_assignments = sum + std::get<int>(eff.costValue);
                    } else {
                        sum_of_assignments = static_cast<float>(sum) + std::get<float>(eff.costValue);
                    }
                }
            } else {
                string assignment_in_expansion_error = "Assignment expressions on expansion cycle are not allowed";

                throw std::runtime_error(assignment_in_expansion_error);
            }
        } else {
            float sum = std::get<float>(sum_of_assignments);

            if(eff.isCostChangeExpression) {
                if(is_same_predicate(eff,expansion_pred)) {
                    if(holds_alternative<int>(eff.costValue)) {
                        sum_of_assignments = sum + static_cast<float>(std::get<int>(eff.costValue));
                    } else {
                        sum_of_assignments = sum + std::get<float>(eff.costValue);
                    }
                }
            } else {
                string assignment_in_expansion_error = "Assignment expressions on expansion cycle are not allowed";

                throw std::runtime_error(assignment_in_expansion_error);
            }
        }
    }

    return sum_of_assignments;
}

std::variant<int,float> TDG::check_predicate_assignments(task t, vector<pair<string,string>> var_mapping, literal pred, variant<int,float> current_sum, vector<literal> world_state) {
    variable_renaming(t, var_mapping);

    variant<int,float> sum_of_assignments = 0;

    for(literal eff : t.costExpression) {
        if(holds_alternative<int>(sum_of_assignments)) {
            int sum = std::get<int>(sum_of_assignments);

            if(eff.isCostChangeExpression && !eff.isAssignCostChangeExpression) {
                if(is_same_predicate(eff,pred)) {
                    if(holds_alternative<int>(eff.costValue)) {
                        sum_of_assignments = sum + std::get<int>(eff.costValue);
                    } else {
                        sum_of_assignments = static_cast<float>(sum) + std::get<float>(eff.costValue);
                    }
                }
            } else {
                literal state;

                for(literal ws : world_state) {
                    if(is_same_predicate(ws,pred)) {
                        state = ws;
                    }
                }

                variant<int,float> state_val;
                if(state.predicate == pred.predicate) {
                    state_val = state.costValue;
                } else {
                    state_val = pred.costValue;
                }

                if(is_same_predicate(eff,pred)) {
                    if(holds_alternative<int>(sum_of_assignments)) {
                        int sum = std::get<int>(sum_of_assignments);

                        if(holds_alternative<int>(eff.costValue)) {
                            int eff_val = std::get<int>(eff.costValue);

                            if(holds_alternative<int>(state_val)) {
                                sum_of_assignments = sum + (std::get<int>(state_val) - sum - eff_val);
                            } else {
                                sum_of_assignments = static_cast<float>(sum) + (std::get<float>(state_val) - static_cast<float>(sum) - static_cast<float>(eff_val));
                            }
                        } else {
                            float eff_val = std::get<float>(eff.costValue);

                            if(holds_alternative<int>(state_val)) {
                                sum_of_assignments = static_cast<float>(sum) + (static_cast<float>(std::get<int>(state_val)) - static_cast<float>(sum) - eff_val);
                            } else {
                                sum_of_assignments = static_cast<float>(sum) + (std::get<float>(state_val) - static_cast<float>(sum) - eff_val);
                            }
                        }
                    } else {
                        float sum = std::get<float>(sum_of_assignments);

                        if(holds_alternative<int>(eff.costValue)) {
                            int eff_val = std::get<int>(eff.costValue);

                            if(holds_alternative<int>(state_val)) {
                                sum_of_assignments = sum + (static_cast<float>(std::get<int>(state_val)) - sum - static_cast<float>(eff_val));
                            } else {
                                sum_of_assignments = sum + (std::get<float>(state_val) - sum - static_cast<float>(eff_val));
                            }
                        } else {
                            float eff_val = std::get<float>(eff.costValue);

                            if(holds_alternative<int>(state_val)) {
                                sum_of_assignments = sum + (static_cast<float>(std::get<int>(state_val)) - sum - eff_val);
                            } else {
                                sum_of_assignments = sum + (std::get<float>(state_val) - sum - eff_val);
                            }
                        }
                    }
                }
            }
        } else {
            float sum = std::get<float>(sum_of_assignments);

            if(eff.isCostChangeExpression) {
                if(is_same_predicate(eff,pred)) {
                    if(holds_alternative<int>(eff.costValue)) {
                        sum_of_assignments = sum + static_cast<float>(std::get<int>(eff.costValue));
                    } else {
                        sum_of_assignments = sum + std::get<float>(eff.costValue);
                    }
                }
            } else {
                // TODO
            }
        }
    }

    return sum_of_assignments;
}