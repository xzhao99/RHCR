#pragma once
#include "BasicGraph.h"
#include <string>
#include <utility>
#include <vector>


class BeeGraph :
	public BasicGraph
{
public:
	std::vector<int> flowers;
	std::vector<int> flower_demands;
	std::vector<int> flower_costs;
	std::vector<std::pair<int, int> > flower_time_windows;
	std::vector<int> initial_locations;
	int bee_capacity;
	int entrance; // vertex collisions at home will be ignored
	int num_of_bees;
	int max_timestep;
	int move_cost; // cost of a move action
	int wait_cost; // cost of a wait action
	bool load_map(std::string fname);
	bool load_Nathan_map(std::string fname);
	void preprocessing(std::string fname, bool consider_rotation); // compute heuristics
	double loading_time = 0; // time for loading the map from files
	double preprocessing_time = 0; // in seconds
};
