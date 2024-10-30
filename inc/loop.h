#ifndef LOOP_H
#define LOOP_H

#include <vector>
#include <tuple>
#include <string>
#include "edge.h"

class Loop {
public:
    std::vector<std::tuple<Edge, std::string>> edges;  
    bool isOuter;

    Loop();

    bool operator==(const Loop& other) const;
};

#endif 
