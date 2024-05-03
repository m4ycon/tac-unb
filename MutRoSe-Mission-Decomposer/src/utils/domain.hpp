#ifndef __DOMAIN
#define __DOMAIN

#include <vector>
#include <map>
#include <set>
#include <string>
#include <variant>
#include "sortexpansion.hpp"

using namespace std;

const string dummy_equal_literal = "__equal";
const string dummy_comparison_literal = "__comparison";
const string dummy_ofsort_literal = "__ofsort";
const string dummy_function_type = "__none";
const string numeric_function_type = "number";
const string method_precondition_action_name = "__method_precondition_";

const string equal_comparison_op = "=";
const string greater_comparison_op = ">";

struct literal{
	bool positive;
	bool isConstantCostExpression;
	bool isCostChangeExpression;
	bool isAssignCostChangeExpression;
	bool isComparisonExpression;
	string predicate;
	vector<string> arguments;
	variant<int,float> costValue;
	pair<string,variant<int,float>> comparison_op_and_value;
};

struct conditional_effect {
	vector<literal> condition;
	literal effect;

	conditional_effect(vector<literal> cond, literal eff);
};

enum reward_type {INTEGER, FLOATINGPOINT, PREDICATE, NONE};

struct reward_change {
	bool isRewardChangeExpression;
	reward_type type;
	string name; 
	int value;
	float fvalue;
	literal predvalue;
	bool increase;
};

struct conditional_reward_change {
	vector<literal> condition;
	reward_change rwd;

	conditional_reward_change(vector<literal> cond, reward_change r);
};

struct task{
	string name;
	int number_of_original_vars; // the first N variables are original, i.e. exist in the HDDL input file. The rest is artificial and was added by this parser for compilation
	vector<pair<string,string>> vars;
	vector<string> required_capabilities;
	vector<literal> prec;
	vector<literal> eff;
	vector<conditional_effect> ceff;
	vector<reward_change> rwd;
	vector<conditional_reward_change> crwd;
	vector<literal> constraints;
	vector<literal> costExpression;
	string reliability;

	void check_integrity();
};

struct plan_step{
	string task;
	string id;
	vector<string> args;

    bool operator< (const plan_step& ps) const {
        return (id < ps.id);
    }
};

struct method{
	string name;
	vector<pair<string,string>> vars;
	string at;
	vector<string> atargs;
	vector<plan_step> ps;
	vector<literal> constraints;
	vector<pair<string,string>> ordering;
	
	void check_integrity();
};


// sort name and set of elements
extern map<string,set<string> > sorts;
extern vector<method> methods;
extern vector<task> primitive_tasks;
extern vector<task> abstract_tasks;
extern map<string, task> task_name_map;

void flatten_tasks(bool compileConditionalEffects, bool linearConditionalEffectExpansion, bool encodeDisjunctivePreconditionsInMethods);
void parsed_method_to_data_structures(bool compileConditionalEffects, bool linearConditionalEffectExpansion, bool encodeDisjunctivePreconditionsInMethods);
void reduce_constraints();
void clean_up_sorts();
void remove_unnecessary_predicates();
void flatten_mdp_tasks();

#endif
