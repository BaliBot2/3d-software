#include "brep.h"
#include <iostream>
#include <unordered_map>
#include <unordered_set>
#include <queue>
#include <algorithm>
#include <stdexcept>
#include <cmath>

// Constructor
BREP::BREP() {}

BREP BREP::getCopy() const {
    BREP copy;
    copy.vertex = this->vertex;
    copy.edges = this->edges;
    copy.loops = this->loops;
    copy.faces = this->faces;
    return copy;
}

bool BREP::validateEulerFormula() {
    int V = vertex.size();
    int E = edges.size();
    int F = faces.size();
    int G = 0;  // Assuming genus G = 0 for simplicity (no holes)

    return (V - E + F == 2 - 2 * G);
}

bool BREP::validateEdgeUsage() const {
    std::unordered_map<int, int> edgeUsage;

    // Count edge usage in all loops
    for (const auto& loop : loops) {
        for (const auto& edgeTuple : loop.edges) {
            const Edge& edge = std::get<0>(edgeTuple);
            auto it = std::find(edges.begin(), edges.end(), edge);
            if (it != edges.end()) {
                int edgeIndex = std::distance(edges.begin(), it);
                edgeUsage[edgeIndex]++;
            }
        }
    }

    // Check if each edge is used exactly twice
    for (const auto& entry : edgeUsage) {
        int edgeIndex = entry.first;
        int count = entry.second;
        if (count != 2) {
            std::cout << "Validation failed: Edge " << edgeIndex
                      << " is used " << count << " times." << std::endl;
            return false;
        }
    }

    // Also check for edges not used at all
    for (size_t i = 0; i < edges.size(); ++i) {
        if (edgeUsage.find(i) == edgeUsage.end()) {
            std::cout << "Validation failed: Edge " << i
                      << " is not used in any loop." << std::endl;
            return false;
        }
    }

    return true;
}
BREP BREP::fromJSON(const json& j) {
    BREP brep;

    // Parse vertices
    if (j.contains("vertices") && j["vertices"].is_array()) {
        for (const auto& vertexJson : j["vertices"]) {
            float x = vertexJson.at("x").get<float>();
            float y = vertexJson.at("y").get<float>();
            float z = vertexJson.at("z").get<float>();
            brep.vertex.push_back(Vertex(x, y, z));
        }
    } else {
        throw std::invalid_argument("JSON does not contain valid 'vertices' array.");
    }

    // Parse edges
    if (j.contains("edges") && j["edges"].is_array()) {
        for (const auto& edgeJson : j["edges"]) {
            int startIdx = edgeJson.at("start").get<int>();
            int endIdx = edgeJson.at("end").get<int>();
            int style = edgeJson.at("style").get<int>();
            brep.edges.push_back(Edge(startIdx, endIdx, style));
        }
    } else {
        throw std::invalid_argument("JSON does not contain valid 'edges' array.");
    }

    // Parse loops
    if (j.contains("loops") && j["loops"].is_array()) {
        for (const auto& loopJson : j["loops"]) {
            Loop loop;
            loop.isOuter = loopJson.at("isOuter").get<bool>();
            if (loopJson.contains("edges") && loopJson["edges"].is_array()) {
                for (const auto& edgeIdx : loopJson["edges"]) {
                    int idx = edgeIdx.get<int>();
                    // Assuming you have a method to get the edge by index
                    Edge edge = brep.edges.at(idx);
                    loop.edges.push_back(std::make_tuple(edge, ""));
                }
            } else {
                throw std::invalid_argument("Loop does not contain valid 'edges' array.");
            }
            brep.loops.push_back(loop);
        }
    } else {
        throw std::invalid_argument("JSON does not contain valid 'loops' array.");
    }

    // Parse faces
    if (j.contains("faces") && j["faces"].is_array()) {
        for (const auto& faceJson : j["faces"]) {
            Faces face;
            int outerLoopIdx = faceJson.at("outerLoop").get<int>();
            face.outerLoop = brep.loops.at(outerLoopIdx);

            if (faceJson.contains("innerLoops") && faceJson["innerLoops"].is_array()) {
                for (const auto& innerLoopIdx : faceJson["innerLoops"]) {
                    int idx = innerLoopIdx.get<int>();
                    face.innerLoops.push_back(brep.loops.at(idx));
                }
            }
            brep.faces.push_back(face);
        }
    } else {
        throw std::invalid_argument("JSON does not contain valid 'faces' array.");
    }

    return brep;
}

json BREP::toJSON() const {
    json j;
    j["vertices"] = json::array();
    for (const auto& v : vertex) {
        j["vertices"].push_back({
            {"x", v.x},
            {"y", v.y},
            {"z", v.z}
        });
    }
    j["edges"] = json::array();
    for (const auto& e : edges) {
        j["edges"].push_back({
            {"start", e.startVertexIndex},
            {"end", e.endVertexIndex},
            {"style", e.style}
        });
    }
    j["loops"] = json::array();
    for (const auto& loop : loops) {
        json loop_json;
        loop_json["isOuter"] = loop.isOuter;
        loop_json["edges"] = json::array();
        for (const auto& edge_tuple : loop.edges) {
            const Edge& edge = std::get<0>(edge_tuple);
            auto it = std::find(edges.begin(), edges.end(), edge);
            if (it != edges.end()) {
                int edge_index = std::distance(edges.begin(), it);
                loop_json["edges"].push_back(edge_index);
            }
        }
        j["loops"].push_back(loop_json);
    }
    j["faces"] = json::array();
    for (const auto& face : faces) {
        json face_json;
        // Find the index of the outer loop
        auto outerIt = std::find(loops.begin(), loops.end(), face.outerLoop);
        if (outerIt != loops.end()) {
            int outerLoopIndex = std::distance(loops.begin(), outerIt);
            face_json["outerLoop"] = outerLoopIndex;
        }
        // Find indices of inner loops
        face_json["innerLoops"] = json::array();
        for (const auto& innerLoop : face.innerLoops) {
            auto innerIt = std::find(loops.begin(), loops.end(), innerLoop);
            if (innerIt != loops.end()) {
                int innerLoopIndex = std::distance(loops.begin(), innerIt);
                face_json["innerLoops"].push_back(innerLoopIndex);
            }
        }
        j["faces"].push_back(face_json);
    }

    return j;
}

void BREP::display() const {
    std::cout << "BREP Details:\n";
    std::cout << "Vertices:\n";
    for (size_t i = 0; i < vertex.size(); ++i) {
        const auto& v = vertex[i];
        std::cout << "  " << i << ": (" << v.x << ", " << v.y << ", " << v.z << ")\n";
    }
    std::cout << "Edges:\n";
    for (size_t i = 0; i < edges.size(); ++i) {
        const auto& e = edges[i];
        const Vertex& startV = vertex[e.startVertexIndex];
        const Vertex& endV = vertex[e.endVertexIndex];
        std::cout << "  " << i << ": Start: (" << startV.x << ", " << startV.y << ", " << startV.z << ")";
        std::cout << "  End: (" << endV.x << ", " << endV.y << ", " << endV.z << ")\n";
    }
}

bool BREP::validateLoopsAreClosed() const {
    for (const auto& loop : loops) {
        if (!isLoopClosed(loop)) {
            std::cout << "Validation failed: Loop is not closed." << std::endl;
            return false;
        }
    }
    return true;
}

bool BREP::validateConnectivity() const {
    if (faces.empty()) {
        std::cout << "Validation failed: No faces in the BREP." << std::endl;
        return false;
    }

    std::vector<bool> visited(faces.size(), false);
    std::queue<int> queue;
    queue.push(0);  // Start from the first face
    visited[0] = true;

    while (!queue.empty()) {
        int currentFaceIdx = queue.front();
        queue.pop();

        const Faces& currentFace = faces[currentFaceIdx];

        // Find adjacent faces
        for (size_t i = 0; i < faces.size(); ++i) {
            if (!visited[i] && areFacesAdjacent(currentFace, faces[i])) {
                visited[i] = true;
                queue.push(i);
            }
        }
    }

    // Check if all faces were visited
    for (size_t i = 0; i < visited.size(); ++i) {
        if (!visited[i]) {
            std::cout << "Validation failed: Face " << i << " is isolated." << std::endl;
            return false;
        }
    }

    return true;
}

int BREP::calculateGenus() const {
    int V = vertex.size();
    int E = edges.size();
    int F = faces.size();

    int L = loops.size();
    int C = calculateConnectedComponents();

    // Extended Euler's formula: V - E + F - (L - F) = 2 * (C - G)
    // Solving for G (genus):
    int G = ((E - V - F - (L - F) + 2 * C) / 2);

    return G;
}

bool BREP::validateEulerFormulaWithGenus() const {
    int V = vertex.size();
    int E = edges.size();
    int F = faces.size();
    int L = loops.size();
    int S = 1;  // Assuming a single solid shell

    // Calculate Euler characteristic with loops
    int chi = V - E + F - (L - F);

    // Calculate genus
    double G = (2.0 * S - chi) / 2.0;

    // Check if G is a non-negative integer
    if (G < 0 || std::abs(G - std::round(G)) > 1e-6) {
        std::cout << "Validation failed: Invalid genus calculated." << std::endl;
        return false;
    }

    // Check if Euler's formula is satisfied
    if (std::abs(chi - 2 * (S - G)) > 1e-6) {
        std::cout << "Validation failed: Euler's formula not satisfied." << std::endl;
        return false;
    }

    std::cout << "Euler's formula validated with Genus = " << G << "." << std::endl;
    return true;
}

bool BREP::isLoopClosed(const Loop& loop) const {
    if (loop.edges.empty()) {
        return false;
    }

    // Build adjacency list for the loop
    std::unordered_map<int, std::vector<int>> adjacency;
    for (const auto& edgeTuple : loop.edges) {
        const Edge& edge = std::get<0>(edgeTuple);
        adjacency[edge.startVertexIndex].push_back(edge.endVertexIndex);
        adjacency[edge.endVertexIndex].push_back(edge.startVertexIndex); // For undirected graph
    }

    // Use DFS to check if the loop is connected and forms a single cycle
    std::unordered_set<int> visited;
    std::stack<int> stack;

    // Start from any vertex in the loop
    int startVertex = std::get<0>(loop.edges[0]).startVertexIndex;
    stack.push(startVertex);

    while (!stack.empty()) {
        int current = stack.top();
        stack.pop();

        if (visited.find(current) != visited.end()) {
            continue;
        }

        visited.insert(current);

        for (int neighbor : adjacency[current]) {
            if (visited.find(neighbor) == visited.end()) {
                stack.push(neighbor);
            }
        }
    }

    // Check if all vertices in the loop are visited
    std::size_t totalVerticesInLoop = adjacency.size();
    return (visited.size() == totalVerticesInLoop && loop.edges.size() == totalVerticesInLoop);
}


bool BREP::areFacesAdjacent(const Faces& face1, const Faces& face2) const {
    // Store the edge indices of face1
    std::unordered_set<int> face1Edges;

    // Add edges from face1's outerLoop
    for (const auto& edgeTuple : face1.outerLoop.edges) {
        const Edge& edge = std::get<0>(edgeTuple);
        auto it = std::find(edges.begin(), edges.end(), edge);
        if (it != edges.end()) {
            int edgeIndex = std::distance(edges.begin(), it);
            face1Edges.insert(edgeIndex);
        }
    }

    // Add edges from face1's innerLoops
    for (const auto& loop : face1.innerLoops) {
        for (const auto& edgeTuple : loop.edges) {
            const Edge& edge = std::get<0>(edgeTuple);
            auto it = std::find(edges.begin(), edges.end(), edge);
            if (it != edges.end()) {
                int edgeIndex = std::distance(edges.begin(), it);
                face1Edges.insert(edgeIndex);
            }
        }
    }

    // Check if face2 shares any edge with face1
    // First, get edges from face2
    std::unordered_set<int> face2Edges;

    // Add edges from face2's outerLoop
    for (const auto& edgeTuple : face2.outerLoop.edges) {
        const Edge& edge = std::get<0>(edgeTuple);
        auto it = std::find(edges.begin(), edges.end(), edge);
        if (it != edges.end()) {
            int edgeIndex = std::distance(edges.begin(), it);
            face2Edges.insert(edgeIndex);
        }
    }

    // Add edges from face2's innerLoops
    for (const auto& loop : face2.innerLoops) {
        for (const auto& edgeTuple : loop.edges) {
            const Edge& edge = std::get<0>(edgeTuple);
            auto it = std::find(edges.begin(), edges.end(), edge);
            if (it != edges.end()) {
                int edgeIndex = std::distance(edges.begin(), it);
                face2Edges.insert(edgeIndex);
            }
        }
    }

    // Now check for shared edges between face1 and face2
    for (const int edgeIndex : face2Edges) {
        if (face1Edges.find(edgeIndex) != face1Edges.end()) {
            return true;  // Found a shared edge
        }
    }

    return false;
}

int BREP::calculateConnectedComponents() const {
    // For simplicity, assume the model is a single connected component
    return 1;
}
