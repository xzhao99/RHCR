#pragma once
#include "States.h"
#include <list>
#include <memory>
#include <unordered_map>
#include <utility>
#include <vector>


class PathTable
{
public:
    PathTable(const std::vector<Path*>& paths, int window, int k_robust);

    void remove(const Path* old_path, int agent);
    std::list<std::shared_ptr<Conflict> > add(const Path* new_path, int agent);


private:

    std::unordered_map<int, std::list<std::pair<int, int>>> PT; // key: location; value: std::list of time-agent std::pair
    int window;
    int k_robust;
    int num_of_agents;


};
