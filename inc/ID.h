#include "MAPFSolver.h"
#include <ctime>
#include <string>
#include <utility>
#include <vector>
// Independence detection
// Designed for ECBS and PBS
class ID: public MAPFSolver
{
public:
	MAPFSolver& solver;

	// The first two parameters are useless, since they are already defined in mapf_solver.
	ID(const BasicGraph& G, SingleAgentSolver& path_planner, MAPFSolver& mapf_solver):
		MAPFSolver(G, path_planner), solver(mapf_solver) {}

	bool run(const std::vector<State>& starts,
		const std::vector< std::vector<std::pair<int, int> > >& goal_locations,
		int time_limit);
	void clear() { solution.clear(); }
	void save_results(const std::string &fileName, const std::string &instanceName) const;
	void save_search_tree(const std::string &fileName) const {}
	void save_constraints_in_goal_node(const std::string &fileName) const {}
	std::string get_name() const {return "ID+" + solver.get_name(); }
	void print_results() const;
private:
	clock_t start_time;
	std::vector<int> group_ids;
	bool plan_paths_for_group(int group_id);
	bool has_conflicts(const Path& path1, const Path& path2) const;
};
