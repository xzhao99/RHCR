#pragma once
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include "common.h"
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>


class PriorityGraph
{
public:
    double runtime;

    void clear();
    bool empty() const {return G.empty(); }
    void copy(const PriorityGraph& other);
    void copy(const PriorityGraph& other, const std::vector<bool>& excluded_nodes);
    void add(int from, int to); // from is lower than to
    void remove(int from, int to); // from is lower than to
    bool connected(int from, int to) const;
    std::unordered_set<int> get_reachable_nodes(int root);

    void save_as_digraph(std::string fname) const;
    typedef std::unordered_map<int, std::unordered_set<int> > PGraph_t;

    void update_number_of_lower_nodes(std::vector<int>& lower_nodes, int node) const;

    PGraph_t G;

    // TODO:  connected components
};
