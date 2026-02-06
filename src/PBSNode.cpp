#include "PBSNode.h"
#include <iostream>

void PBSNode::clear()
{
    conflicts.clear();
    priorities.clear();
}


void PBSNode::print_priorities() const
{
    std::cout << "Priorities: ";
    for (auto row : priorities.G)
    {
        std::cout << row.first << " < (";
        for (auto a : row.second)
            std::cout << a << ", ";
        std::cout << "); ";
    }
    std::cout << std::endl;
}