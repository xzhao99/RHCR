#pragma once
#include "SIPP.h"
#include "MAPFSolver.h"
#include <cstdint>
#include <ctime>
#include <string>
#include <utility>
#include <vector>

// WHCA* with random restart
class WHCAStar :
	public MAPFSolver
{
public:

    uint64_t num_expanded;
    uint64_t num_generated;
    uint64_t num_restarts;

    std::vector<Path> initial_solution;

    // Runs the algorithm until the problem is solved or time is exhausted
    bool run(const std::vector<State>& starts,
             const std::vector< std::vector<std::pair<int, int> > >& goal_locations,
             int time_limit);

	std::string get_name() const {return "WHCA"; }

    void save_results(const std::string &fileName, const std::string &instanceName) const;
	void save_search_tree(const std::string &fileName) const {}
	void save_constraints_in_goal_node(const std::string &fileName) const {}
	void clear();

    WHCAStar(const BasicGraph& G, SingleAgentSolver& path_planner);
    ~WHCAStar() {}


private:
    void print_results() const;
};
