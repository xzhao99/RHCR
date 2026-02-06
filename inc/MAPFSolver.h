#pragma once
#include "PBSNode.h"
#include "SIPP.h"
#include <ctime>
#include <list>
#include <string>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

// Base class for MAPF solvers
class MAPFSolver
{
public:
    int k_robust;
    int window;
	bool hold_endpoints;

	double runtime;
    int screen;

	bool solution_found;
	double solution_cost;
	double avg_path_length;
	double min_sum_of_costs;
    std::vector<Path> solution;

	// initial data
	ReservationTable initial_rt;
	std::vector<Path> initial_paths;
    std::list< std::tuple<int, int, int> > initial_constraints; // <agent, location, timestep>:
    // only this agent can stay in this location before this timestep.
	std::list<const Path*> initial_soft_path_constraints; // the paths that all agents try to avoid
	std::unordered_map<int, double> travel_times;


	SingleAgentSolver& path_planner;
	// Runs the algorithm until the problem is solved or time is exhausted 
    virtual bool run(const std::vector<State>& starts,
            const std::vector< std::vector<std::pair<int, int> > >& goal_locations,  // an ordered std::list of pairs of <location, release time>
            int time_limit) = 0;


	MAPFSolver(const BasicGraph& G, SingleAgentSolver& path_planner);
	~MAPFSolver();

	// Save results
	virtual void save_results(const std::string &fileName, const std::string &instanceName) const = 0;
	virtual void save_search_tree(const std::string &fileName) const = 0;
	virtual void save_constraints_in_goal_node(const std::string &fileName) const = 0;
	virtual void clear() = 0;

	virtual std::string get_name() const = 0;

    const BasicGraph& G;
    std::vector<State> starts;
    std::vector< std::vector<std::pair<int, int> > > goal_locations;
    int num_of_agents;
	int time_limit;

    // validate
    bool validate_solution();
    void print_solution() const;
protected:
    std::vector<std::vector<bool> > cat; // conflict avoidance table
    std::vector<std::unordered_set<std::pair<int, int>, pair_hash> > constraint_table;
    ReservationTable rt;
};
