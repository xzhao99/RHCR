#pragma once
#include "MAPFSolver.h"
#include <cstdint>
#include <list>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>


class LRAStar: public MAPFSolver
{
public:
	int simulation_window;

    uint64_t num_wait_commands;
    uint64_t num_expanded;
    uint64_t num_generated;

    // Runs the algorithm until the problem is solved or time is exhausted
    void resolve_conflicts(const std::vector<Path>& paths); // current implementation can only deal with k_robust <= 1
    // TODO: implement for larger k_robust


    void save_results(const std::string &fileName, const std::string &instanceName) const;
	void save_search_tree(const std::string &fileName) const {}
	void save_constraints_in_goal_node(const std::string &fileName) const {}

    LRAStar(const BasicGraph& G, SingleAgentSolver& path_planner);

	bool run(const std::vector<State>& starts,
		const std::vector< std::vector<std::pair<int, int> > >& goal_locations,
		int time_limit);
	std::string get_name() const {return "LRA"; }
	void clear() {}

private:
    StateTimeAStar astar; // TODO: delete this
    std::unordered_map<int, int> curr_locations; // key = location, value = agent_id
    std::unordered_map<int, int> next_locations; // key = location, value = agent_id
    // std::vector<std::list<std::pair<int, int> > > trajectories;

    void print_results() const;
    void wait_command(int agent, int timestep,
                      std::vector<std::list<std::pair<int, int> >::const_iterator >& traj_pointers);
    void wait_command(int agent, int timestep, std::vector<int>& path_pointers);

	Path find_shortest_path(const State& start, const std::vector<std::pair<int, int> >& goal_location);
};
