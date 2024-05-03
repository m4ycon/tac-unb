#include "query_utils.hpp"

#include <regex>

#include <boost/foreach.hpp>
#include <boost/algorithm/string.hpp>

using namespace std;

/*
    Function: get_query_ptree
    Objective: Based on a queried property, find the ptree that corresponds to the object being queried

    @ Input 1: The Goal Model as a GMGraph object
    @ Input 2: The Query goal node ID
    @ Input 3: The valid variables map
    @ Input 4: The valid forAll conditions map
    @ Input 5: The world knowledge ptree
    @ Input 6: The knowledge unique ID name
    @ Output: The ptree corresponding to the queried object
*/
pt::ptree get_query_ptree(GMGraph gm, int node_id, map<string,pair<string,vector<pt::ptree>>> valid_variables, map<int,AchieveCondition> valid_forAll_conditions, pt::ptree world_tree, string knowledge_unique_id) {
	pt::ptree queried_tree;
	QueriedProperty q = std::get<QueriedProperty>(gm[node_id].custom_props[queried_property_prop]);

	if(q.queried_var == world_db_query_var) {
		queried_tree = world_tree;
	} else {
		bool valid_query = true;
		if(q.queried_var.find(".") == string::npos) {
			if(valid_variables.find(q.queried_var) != valid_variables.end()) {
				if(valid_variables[q.queried_var].second.size() != 1) {
					valid_query = false;
				} else {
					queried_tree = valid_variables[q.queried_var].second.at(0);
				}
			} else {
				valid_query = false;
			}
		} else {
			vector<string> query_attrs;
						
			string queried_var = q.queried_var;
			std::replace(queried_var.begin(), queried_var.end(),'.',' ');

			stringstream ss(queried_var);
			string temp;
			while(ss >> temp) {
				query_attrs.push_back(temp);
			}

			pt::ptree var_to_query;
			string var_type;

			bool found_var = false;
			int current_attr = 0;

			if(valid_variables.find(query_attrs.at(0)) != valid_variables.end()) {
				var_type = valid_variables[query_attrs.at(0)].first;
				found_var = true;
			}

			if(!found_var) {
				valid_query = false;
			}			
							
			if(valid_query) {
				BOOST_FOREACH(pt::ptree::value_type& child, world_tree) {
					if(child.first == var_type) {	
						if(child.second.get<string>(knowledge_unique_id) == valid_variables[query_attrs.at(0)].second.at(0).get<string>(knowledge_unique_id)) { //Doesn't work for collection variables
							boost::optional<pt::ptree&> attr = child.second.get_child_optional(query_attrs.at(1));
							if(!attr) {
								valid_query = false;
							} else {
								current_attr = 1;
								var_to_query = attr.get();
							}

							break;
						}
					}
				}

				while(current_attr < int(query_attrs.size())-1 && valid_query) {
					boost::optional<pt::ptree&> attr = var_to_query.get_child_optional(query_attrs.at(current_attr+1));
					if(!attr) {
						valid_query = false;
					} else {
						current_attr++;
						var_to_query = attr.get();
					}
				}

				if(valid_query) {
					string queried_var_type = std::get<vector<pair<string,string>>>(gm[node_id].custom_props[controls_prop]).at(0).second;
										
					string gm_var_type = parse_gm_var_type(queried_var_type);
					if(gm_var_type == "COLLECTION") {
                        size_t begin = queried_var_type.find("(")+1;
                        size_t end = queried_var_type.find(")");
						queried_var_type = queried_var_type.substr(begin,end-begin);
					}

					BOOST_FOREACH(pt::ptree::value_type& child, var_to_query) {
						if(child.first != queried_var_type) {
							valid_query = false;
							break;
						}
					}
				}
			}

			if(valid_query) {
				queried_tree = var_to_query;
			} else { 
				string invalid_query_error = "Invalid query in Goal " + get_node_name(gm[node_id].text);

				throw std::runtime_error(invalid_query_error);
			}	
		}
	}

	return queried_tree;
}

/*
    Function: solve_query_statement
    Objective: Solve a given QueriedProperty

	@ Input 1: The world knowledge tree that represents the queried variable
	@ Input 2: The QueriedProperty to be solved
	@ Input 3: The Goal Model as a GMGraph object
	@ Input 4: The current GM node ID
	@ Input 5: The valid GM variables map
	@ Input 6: The current variable mappings of the Goal Model
    @ Output: The world knowledge trees of the records that satisfied the queried property. In addition, a set of their unique knowledge ID's is given
*/
pair<vector<pt::ptree>,set<string>> solve_query_statement(pt::ptree queried_tree, QueriedProperty q, GMGraph gm, int node_id, map<string,pair<string,vector<pt::ptree>>>& valid_variables, string knowledge_unique_id) {
	if(holds_alternative<pair<Query*,Query*>>(q.query->query)) {
        pair<Query*,Query*> query_items = std::get<pair<Query*,Query*>>(q.query->query);

        QueriedProperty aux1;
        aux1.queried_var = q.queried_var;
        aux1.query_var = q.query_var;
        aux1.query = query_items.first;

        pair<vector<pt::ptree>,set<string>> valid_query1 = solve_query_statement(queried_tree, aux1, gm, node_id, valid_variables, knowledge_unique_id);

        QueriedProperty aux2;
        aux2.queried_var = q.queried_var;
        aux2.query_var = q.query_var;
        aux2.query = query_items.second;

        pair<vector<pt::ptree>,set<string>> valid_query2 = solve_query_statement(queried_tree, aux2, gm, node_id, valid_variables, knowledge_unique_id);

		pair<vector<pt::ptree>,set<string>> final_result = valid_query1;

        if(q.query->is_and) {
            set<string> aux;

            std::set_difference(valid_query1.second.begin(), valid_query1.second.end(), valid_query2.second.begin(), valid_query2.second.end(), std::inserter(aux, aux.end()));

            for(string elem : aux) {
                set<string>::iterator pos = final_result.second.find(elem);
                if(pos != final_result.second.end()) {
                    final_result.second.erase(pos);
                }
            }

            vector<pt::ptree>::iterator result_it;
            for(result_it = final_result.first.begin(); result_it != final_result.first.end(); ) {
                string result_val = result_it->get<string>(knowledge_unique_id);

                if(final_result.second.find(result_val) == final_result.second.end()) {
                    final_result.first.erase(result_it);
                } else {
                    result_it++;
                }
            }
        } else {
            set<string> aux;

            std::set_difference(valid_query1.second.begin(), valid_query1.second.end(), valid_query2.second.begin(), valid_query2.second.end(), std::inserter(aux, aux.end()));

            for(string elem : aux) {
                set<string>::iterator pos = final_result.second.find(elem);
                if(pos == final_result.second.end()) {
                    final_result.second.insert(elem);
                }
            }

			for(pt::ptree res : valid_query2.first) {
				if(aux.find(res.get<string>(knowledge_unique_id)) != aux.end()) {
					final_result.first.push_back(res);
				}
			}
        }

		return final_result;
    } else {
		vector<pt::ptree> aux;
		set<string> accepted_records;

		vector<string> query_item = std::get<vector<string>>(q.query->query);
					
		if(!queried_tree.empty()) {
			BOOST_FOREACH(pt::ptree::value_type& child, queried_tree) {
				if(child.first == q.query_var.second) {
					if(query_item.size() == 0) {
						aux.push_back(child.second);
						accepted_records.insert(child.second.get<string>(knowledge_unique_id));
					} else if(query_item.size() == 1) {
						if(query_item.at(0) != "") {
							string prop = query_item.at(0).substr(query_item.at(0).find('.')+1);

							boost::optional prop_val_opt = child.second.get_optional<string>(prop);

							if(prop_val_opt) {
								bool prop_val;

								istringstream(boost::to_lower_copy(prop_val_opt.get())) >> std::boolalpha >> prop_val;
								if(query_item.at(0).find('!') != string::npos) {
									prop_val = !prop_val;
								}
								if(prop_val) {
									aux.push_back(child.second);
									accepted_records.insert(child.second.get<string>(knowledge_unique_id));
								}
							}
						} else {
							aux.push_back(child.second);
							accepted_records.insert(child.second.get<string>(knowledge_unique_id));
						}
					} else {
						if(query_item.at(1) == ocl_equal || query_item.at(1) == ocl_different) {
							string prop = query_item.at(0).substr(query_item.at(0).find('.')+1);

							boost::optional prop_val_opt = child.second.get_optional<string>(prop);

							if(prop_val_opt) {
								string prop_val = prop_val_opt.get();

								bool result;
								if(query_item.at(1) == ocl_equal) {
									result = (prop_val == query_item.at(2));
								} else {
									result = (prop_val != query_item.at(2));
								}
								if(result) {
									aux.push_back(child.second);
									accepted_records.insert(child.second.get<string>(knowledge_unique_id));
								}
							}
						} else if(query_item.at(1) == ocl_in) {
							bool in_for_ownership = false;
							if(query_item.at(2).find(".") != std::string::npos) {
								string attr_to_search = query_item.at(2);
								std::replace(attr_to_search.begin(), attr_to_search.end(), '.', ' ');

								vector<string> split_query_attr;

								stringstream ss(attr_to_search);
								string tmp;
								while(ss >> tmp) {
									split_query_attr.push_back(tmp);
								}

								if(split_query_attr.at(0) == q.query_var.first) {
									in_for_ownership = true;
								}
							}

							if(in_for_ownership) {
								if(std::count(query_item.at(2).begin(),query_item.at(2).end(),'.') > 1) {
									string too_many_levels_for_ownership_in = "The in operator for ownership relations currently only accepts a variable direct attribute ([var].[attr])";

									throw std::runtime_error(too_many_levels_for_ownership_in);
								}

								string tree_prop = query_item.at(2).substr(query_item.at(2).find(".")+1);
								boost::optional actual_queried_tree_opt = child.second.get_child_optional(tree_prop);

								if(actual_queried_tree_opt) {
									pt::ptree actual_queried_tree = actual_queried_tree_opt.get();

									if(!actual_queried_tree.empty()) {
										BOOST_FOREACH(pt::ptree::value_type& actual_child, actual_queried_tree) {
											string prop = query_item.at(0).substr(query_item.at(0).find('.')+1);

											boost::optional prop_val_opt = actual_child.second.get_optional<string>(prop);

											if(prop_val_opt) {
												string prop_val = prop_val_opt.get();

												string attr_to_search = query_item.at(0);
												std::replace(attr_to_search.begin(), attr_to_search.end(), '.', ' ');
												
												vector<string> split_attr;
												
												stringstream ss(attr_to_search);
												string tmp;
												while(ss >> tmp) {
													split_attr.push_back(tmp);
												}

												pt::ptree attr_tree = valid_variables[split_attr.at(0)].second.at(0).get_child(split_attr.at(1));

												string attr_data = attr_tree.data();
												boost::trim(attr_data);
												if(attr_tree.empty() && attr_data != "") {
													vector<string> attr_values;

													size_t pos = 0;
													while((pos = attr_data.find(',')) != std::string::npos) {
                                                        string attr_val = attr_data.substr(0,pos);
                                                        boost::trim(attr_val);
														attr_values.push_back(attr_val);
														attr_data.erase(0, pos+1);
													}
													attr_values.push_back(attr_data);

													if(std::find(attr_values.begin(), attr_values.end(), prop_val) != attr_values.end()) {
														aux.push_back(child.second);
														accepted_records.insert(child.second.get<string>(knowledge_unique_id));
													}
												} else if(!attr_tree.empty() && attr_data == "") {
													BOOST_FOREACH(pt::ptree::value_type val, attr_tree) {
														if(prop_val == val.second.data()) {
															aux.push_back(child.second);
															accepted_records.insert(child.second.get<string>(knowledge_unique_id));

															break;
														}
													}
												} else {
													string bad_condition = "Cannot solve condition in QueriedProperty of Goal " + get_node_name(gm[node_id].text); 

													throw std::runtime_error(bad_condition);
												}
											}
										}
									}
								}
							} else {
								string prop = query_item.at(0).substr(query_item.at(0).find('.')+1);

								boost::optional prop_val_opt = child.second.get_optional<string>(prop);

								if(prop_val_opt) {
									string prop_val = prop_val_opt.get();

									string attr_to_search = query_item.at(2);
									std::replace(attr_to_search.begin(), attr_to_search.end(), '.', ' ');
									
									vector<string> split_attr;
									
									stringstream ss(attr_to_search);
									string tmp;
									while(ss >> tmp) {
										split_attr.push_back(tmp);
									}

									/*
										If we have [VAR].[ATTR] in [VAR].[ÁTTR] we search in the ptree

										If we have [VAR].[ATTR] in [VAR], where VAR is a collection variable, we search in the variable value
									*/
									if(split_attr.size() == 1) {
										vector<pt::ptree> var_value = valid_variables[split_attr.at(0)].second;
                                        
                                        bool found_attr = false;
                                        for(pt::ptree val : var_value) {
                                            if(val.get<string>(knowledge_unique_id) == prop_val) {
                                                found_attr = true;
                                                
                                                break;
                                            }
                                        }

                                        if(found_attr) {
                                            aux.push_back(child.second);
                                            accepted_records.insert(child.second.get<string>(knowledge_unique_id));
                                        }
									} else if(split_attr.size() == 2) {
										// Here we need to get the query ptree for the second attribute
										pt::ptree attr_tree = valid_variables[split_attr.at(0)].second.at(0).get_child(split_attr.at(1));

										string attr_data = attr_tree.data();
										boost::trim(attr_data);
										if(attr_tree.empty() && attr_data != "") {
											vector<string> attr_values;

											size_t pos = 0;
											while((pos = attr_data.find(',')) != std::string::npos) {
												string attr_val = attr_data.substr(0,pos);
												boost::trim(attr_val);
												attr_values.push_back(attr_val);
												attr_data.erase(0, pos+1);
											}
											attr_values.push_back(attr_data);

											if(std::find(attr_values.begin(), attr_values.end(), prop_val) != attr_values.end()) {
												aux.push_back(child.second);
												accepted_records.insert(child.second.get<string>(knowledge_unique_id));
											}
										} else if(!attr_tree.empty() && attr_data == "") {
											BOOST_FOREACH(pt::ptree::value_type val, attr_tree) {
												if(prop_val == val.second.data()) {
													aux.push_back(child.second);
													accepted_records.insert(child.second.get<string>(knowledge_unique_id));

													break;
												}
											}
										} else {
											string bad_condition = "Cannot solve condition in QueriedProperty of Goal " + get_node_name(gm[node_id].text); 

											throw std::runtime_error(bad_condition);
										}
									} else {
										string bad_condition = "Cannot solve condition in QueriedProperty of Goal " + get_node_name(gm[node_id].text); 

										throw std::runtime_error(bad_condition);
									}
								}
							}
						} else if(query_item.at(1) == ocl_gt || query_item.at(1) == ocl_lt || query_item.at(1) == ocl_geq || query_item.at(1) == ocl_leq) {
							string prop = query_item.at(0).substr(query_item.at(0).find('.')+1);

							boost::optional prop_val_opt = child.second.get_optional<string>(prop);

							if(prop_val_opt) {
								string prop_val = prop_val_opt.get();

								std::regex integer("[0-9]+");

								bool result = false;
								if(std::regex_match(query_item.at(2), integer)) {
									int q_val = stoi(query_item.at(2));

									if(prop_val.find(".") == string::npos) {
										int val = stoi(prop_val);

										if(query_item.at(1) == ocl_gt) {
											result = (val > q_val);
										} else if(query_item.at(1) == ocl_lt) {
											result = (val < q_val);
										} else if(query_item.at(1) == ocl_geq) {
											result = (val >= q_val);
										} else if(query_item.at(1) == ocl_leq) {
											result = (val <= q_val);
										}
									} else {
										float val = static_cast<float>(::atof(prop_val.c_str()));

										if(query_item.at(1) == ocl_gt) {
											result = greater_than_float_and_int(q_val, val);
										} else if(query_item.at(1) == ocl_lt) {
											result = greater_than_int_and_float(q_val, val);
										} else if(query_item.at(1) == ocl_geq) {
											result = !greater_than_int_and_float(q_val, val);
										} else if(query_item.at(1) == ocl_leq) {
											result = !greater_than_float_and_int(q_val, val);
										}
									}
								} else {
									float q_val = static_cast<float>(::atof(query_item.at(2).c_str()));

									if(prop_val.find(".") == string::npos) {
										int val = stoi(prop_val);

										if(query_item.at(1) == ocl_gt) {
											result = greater_than_int_and_float(val, q_val);
										} else if(query_item.at(1) == ocl_lt) {
											result = greater_than_float_and_int(val, q_val);
										} else if(query_item.at(1) == ocl_geq) {
											result = !greater_than_float_and_int(val, q_val);
										} else if(query_item.at(1) == ocl_leq) {
											result = !greater_than_int_and_float(val, q_val);
										}
									} else {
										float val = static_cast<float>(::atof(prop_val.c_str()));

										if(query_item.at(1) == ocl_gt) {
											result = greater_than_floats(val, q_val);
										} else if(query_item.at(1) == ocl_lt) {
											result = greater_than_floats(q_val, val);
										} else if(query_item.at(1) == ocl_geq) {
											result = !greater_than_floats(q_val, val);
										} else if(query_item.at(1) == ocl_leq) {
											result = !greater_than_floats(val, q_val);
										}
									}
								}

								if(result) {
									aux.push_back(child.second);
									accepted_records.insert(child.second.get<string>(knowledge_unique_id));
								}
							}
						}
					}
				}
			}
		}

		return make_pair(aux,accepted_records);
	}
}