#pragma once
#include "BasicGraph.h"
#include <list>
#include <string>
#include <unordered_map>


class SortingGrid :
	public BasicGraph
{
public:
    std::unordered_map<std::string, int> inducts;
    std::unordered_map<std::string, std::list<int> > ejects; // one eject station could have multiple eject fiducials

    bool load_map(std::string fname);
    void preprocessing(bool consider_rotation); // compute heuristics
};
