#ifndef __CONSTRAINT_MANAGER
#define __CONSTRAINT_MANAGER

#include <vector>
#include <map>
#include <string>
#include <variant>
#include <stack>
#include <queue>
#include <set>
#include <iostream>

#include "../missiondecomposer/missiondecomposer.hpp"
#include "../gm/gm.hpp"
#include "../utils/constraint.hpp"

struct ConstraintTree {
    virtual void generate_constraints() = 0;
    virtual std::vector<Constraint> generate_constraints_from_child_contents(std::variant<std::pair<int,ATNode>,std::vector<Constraint>> left_val, std::variant<std::pair<int,ATNode>,std::vector<Constraint>> right_val) = 0;

    std::vector<std::variant<std::pair<int,ATNode>,ConstraintTree*>> children;
    
    std::vector<Constraint> constraints;

    std::map<int,set<int>> constraints_map; // Avoid duplicate constraints 
};

struct SequentialConstraintTree : ConstraintTree {
   void generate_constraints();

   std::vector<Constraint> generate_constraints_from_child_contents(std::variant<std::pair<int,ATNode>,std::vector<Constraint>> left_val, std::variant<std::pair<int,ATNode>,std::vector<Constraint>> right_val);
};

struct FallbackConstraintTree : ConstraintTree {
   void generate_constraints();

   std::vector<Constraint> generate_constraints_from_child_contents(std::variant<std::pair<int,ATNode>,std::vector<Constraint>> left_val, std::variant<std::pair<int,ATNode>,std::vector<Constraint>> right_val);
};

struct ParallelConstraintTree : ConstraintTree {
   void generate_constraints();

   std::vector<Constraint> generate_constraints_from_child_contents(std::variant<std::pair<int,ATNode>,std::vector<Constraint>> left_val, std::variant<std::pair<int,ATNode>,std::vector<Constraint>> right_val);
};

class ConstraintManager {
    public:
        ConstraintManager(GMGraph g, ATGraph md, bool verb, bool pretty);

        std::vector<Constraint> generate_mission_constraints();

        void generate_at_constraints(ATGraph trimmed_mission_decomposition);
        std::variant<std::pair<int,ATNode>,ConstraintTree*> recursive_constraint_tree_build(std::vector<int>& dfs_nodes, ATGraph trimmed_mission_decomposition);

        void transform_at_constraints();
        void generate_execution_constraints();       
        void trim_mission_constraints();
        void check_execution_constraints();
    
    private:
        ATGraph mission_decomposition;
        GMGraph gm;
        bool verbose;
        bool pretty_print;
        std::vector<Constraint> mission_constraints;
};

Constraint generate_constraint(std::pair<int,ATNode> n1, std::pair<int,ATNode> n2, constraint_type type);
bool is_new_constraint(Constraint c, map<int,set<int>>& constraints_map);

#endif