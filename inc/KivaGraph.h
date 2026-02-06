#pragma once
#include "BasicGraph.h"
#include <string>
#include <vector>


class KivaGrid :
	public BasicGraph
{
public:
	std::vector<int> endpoints;
	std::vector<int> agent_home_locations;

    bool load_map(std::string fname);
    void preprocessing(bool consider_rotation); // compute heuristics
private:
    bool load_weighted_map(std::string fname);
    bool load_unweighted_map(std::string fname);
};
