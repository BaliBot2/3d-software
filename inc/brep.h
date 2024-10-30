#ifndef BREP_H
#define BREP_H

#include <vector>
#include <iostream>
#include <unordered_map>
#include <unordered_set>
#include <stack>
#include <queue>
#include <algorithm>
#include <map>
#include "vertex.h"
#include "edge.h"
#include "loop.h"
#include "faces.h"
#include "../lib/nlohmann/json.hpp"

using json = nlohmann::json;

class BREP {
public:
    std::vector<Vertex> vertex;
    std::vector<Edge> edges;
    std::vector<Loop> loops;
    std::vector<Faces> faces;

    BREP();

    BREP getCopy() const;

    bool validateEulerFormula();

    bool validateEdgeUsage() const;

    static BREP fromJSON(const json& j);

    json toJSON() const;

    void display() const;

    bool validateLoopsAreClosed() const;

    bool validateConnectivity() const;

    int calculateGenus() const;

    bool validateEulerFormulaWithGenus() const;

private:
    bool isLoopClosed(const Loop& loop) const;

    bool areFacesAdjacent(const Faces& face1, const Faces& face2) const;

    int calculateConnectedComponents() const;
};

#endif // BREP_H
