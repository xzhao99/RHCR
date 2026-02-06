#pragma once
#include "BasicGraph.h"
#include <string>
#include <vector>


class OnlineGrid :
	public BasicGraph
{
public:
	std::vector<int> entries;
	std::vector<int> exits;
    bool load_map(std::string fname);
    void preprocessing(bool consider_rotation); // compute heuristics
};
