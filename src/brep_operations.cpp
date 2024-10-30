#include "brep_operations.h"
#include <iostream>
#include <algorithm>
#include <unordered_set>
#include <stack>
#include <queue>
#include <fstream>
#include <cmath>

BREPOperations::BREPOperations() {}

void BREPOperations::loadBREP(const BREP& brep) {
    // Validation steps
    if (!brep.validateLoopsAreClosed()) {
        throw std::invalid_argument("BREP validation failed: Loops are not closed.");
    }
    if (!brep.validateEdgeUsage()) {
        throw std::invalid_argument("BREP validation failed: Edge usage incorrect.");
    }
    if (!brep.validateConnectivity()) {
        throw std::invalid_argument("BREP validation failed: Faces are not fully connected.");
    }
    if (!brep.validateEulerFormulaWithGenus()) {
        throw std::invalid_argument("BREP validation failed: Euler's formula not satisfied.");
    }

    currentBREP = brep;
    history.clear();
    redoStack.clear();
    history.push_back({currentBREP, "load"});
    std::cout << "BREP loaded and validated successfully." << std::endl;
}

void BREPOperations::rotate(const std::string& axis, float theta) {
    if (!currentBREP.validateEulerFormula()) {
        throw std::invalid_argument("Invalid BREP: Does not satisfy Euler's formula.");
    }
    Matrix R = mathUtils.rotationMatrix(axis, theta);
    for (auto& v : currentBREP.vertex) {
        v = R.transform(v);
    }
    history.push_back({currentBREP, "rotate"});
    redoStack.clear();
}

void BREPOperations::translate(const std::string& axis, float units) {
    if (!currentBREP.validateEulerFormula()) {
        throw std::invalid_argument("Invalid BREP: Does not satisfy Euler's formula.");
    }
    Matrix T = mathUtils.translationMatrix(axis, units);
    for (auto& v : currentBREP.vertex) {
        v = T.transform(v);
    }
    history.push_back({currentBREP, "translate"});
    redoStack.clear();
}

void BREPOperations::orthogonalProjection(const std::string& view) {
    if (!currentBREP.validateEulerFormula()) {
        throw std::invalid_argument("Invalid BREP: Does not satisfy Euler's formula.");
    }

    // Create a temporary copy of the current BREP
    BREP tempBREP = currentBREP;

    Matrix P = mathUtils.projectionMatrix(view);
    for (auto& v : tempBREP.vertex) {
        v = P.transform(v);
    }

    // Display the result to the user
    std::cout << "Projection applied on a temporary BREP." << std::endl;
    tempBREP.display();

    // Prompt the user to save the temporary BREP
    std::cout << "Do you want to save the projected BREP? (yes/no): ";
    std::string response;
    std::getline(std::cin, response);
    if (response == "yes" || response == "y") {
        // Save the temporary BREP as the current BREP
        currentBREP = tempBREP;
        history.push_back({currentBREP, "project"});
        redoStack.clear();
        std::cout << "Projected BREP saved." << std::endl;
    } else {
        std::cout << "Projected BREP discarded." << std::endl;
    }
}
void BREPOperations::render(const std::string& filename) {
    // Use the JSON filename from the current BREP instance
    std::string jsonFilename = "json_files/" + filename + ".json";

    // Save the current BREP as a JSON file in json_files
    std::ofstream file(jsonFilename);
    if (file.is_open()) {
        json j = currentBREP.toJSON();
        file << j.dump(4);
        file.close();
        std::cout << "BREP data saved to " << jsonFilename << "." << std::endl;
    } else {
        std::cerr << "Error: Unable to save JSON to file." << std::endl;
        return;
    }

    // Construct the command to run the render Makefile
    std::string command = "cd render && make render JSON_FILE=../" + jsonFilename + " OUTPUT_IMAGE=../" + filename + ".png";
    int returnCode = std::system(command.c_str()); // Run the make command

    if (returnCode != 0) {
        std::cerr << "Error: Render command failed with return code " << returnCode << "." << std::endl;
    }
}
void BREPOperations::importBREP(const std::string& filename) {
    // Open the JSON file from the json_files directory
    std::ifstream file("json_files/" + filename);
    if (!file.is_open()) {
        throw std::runtime_error("Unable to open the JSON file: " + filename);
    }

    json j;
    file >> j;
    file.close();

    // Parse the JSON data and load the BREP
    try {
        BREP brep = BREP::fromJSON(j);
        loadBREP(brep);
        std::cout << "BREP imported and loaded successfully from " << filename << "." << std::endl;
    } catch (const std::exception& e) {
        throw std::runtime_error("Error loading BREP: " + std::string(e.what()));
    }
}

void BREPOperations::save(const std::string& filename) {
    json j = currentBREP.toJSON();
    // Save to json_files directory
    std::ofstream file("json_files/" + filename);
    if (file.is_open()) {
        file << j.dump(4);  // Pretty-print with 4-space indentation
        file.close();
        std::cout << "BREP saved to json_files/" << filename << std::endl;
    } else {
        std::cout << "Error: Unable to open file for writing." << std::endl;
    }
}

void BREPOperations::reset() {
    if (!history.empty()) {
        currentBREP = history.front().first;
        history.erase(history.begin() + 1, history.end());
        redoStack.clear();
    }
}

void BREPOperations::undo() {
    if (history.size() > 1) {
        redoStack.push_back(history.back());
        history.pop_back();
        currentBREP = history.back().first;
        std::cout << "Undo operation performed." << std::endl;
    } else {
        std::cout << "Nothing to undo." << std::endl;
    }
}

void BREPOperations::redo() {
    if (!redoStack.empty()) {
        history.push_back(redoStack.back());
        currentBREP = redoStack.back().first;
        redoStack.pop_back();
        std::cout << "Redo operation performed." << std::endl;
    } else {
        std::cout << "Nothing to redo." << std::endl;
    }
}

void BREPOperations::displayCurrentBREP() const {
    currentBREP.display();
}
void BREPOperations::loadBREPFromFile(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error: Unable to open the JSON file." << std::endl;
        return;
    }

    json j;
    file >> j;

    // Parse the JSON data and load the BREP
    try {
        BREP brep = BREP::fromJSON(j);
        loadBREP(brep);
        std::cout << "BREP loaded successfully from " << filename << "." << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "Error loading BREP: " << e.what() << std::endl;
    }

    file.close();
}

void BREPOperations::crossSection(const Plane& planeEqn, const std::string& sign) {
    if (!currentBREP.validateEulerFormula()) {
        throw std::invalid_argument("Invalid BREP: Does not satisfy Euler's formula.");
    }

    // Create a temporary copy of the current BREP
    BREP tempBREP = currentBREP;

    // Determine which side of the plane to keep
    int sideToKeep = (sign == "positive") ? 1 : -1;

    // Classify vertices relative to the plane
    std::vector<int> vertexSigns(tempBREP.vertex.size(), 0);
    std::map<int, int> oldToNewVertexIndex;  // Map old vertex indices to new indices

    for (size_t i = 0; i < tempBREP.vertex.size(); ++i) {
        float value = mathUtils.solvePlaneEquation(planeEqn.a, planeEqn.b, planeEqn.c, planeEqn.d, tempBREP.vertex[i]);
        if (value > 1e-6) {
            vertexSigns[i] = 1;
        } else if (value < -1e-6) {
            vertexSigns[i] = -1;
        } else {
            vertexSigns[i] = 0;  // On the plane
        }
    }

    // New BREP components
    BREP newBREP;
    std::map<std::pair<int, int>, int> edgeIntersectionVertexIndex;

    // Collect cross-sectional edges for loop construction
    std::vector<Edge> crossSectionEdges;

    // Process edges
    for (const auto& edge : tempBREP.edges) {
        int vStart = edge.startVertexIndex;
        int vEnd = edge.endVertexIndex;
        int signStart = vertexSigns[vStart];
        int signEnd = vertexSigns[vEnd];

        // Edge is completely on the desired side or on the plane
        if ((signStart == sideToKeep || signStart == 0) && (signEnd == sideToKeep || signEnd == 0)) {
            int newStartIdx, newEndIdx;

            // Map or add start vertex
            if (oldToNewVertexIndex.find(vStart) == oldToNewVertexIndex.end()) {
                newStartIdx = newBREP.vertex.size();
                newBREP.vertex.push_back(tempBREP.vertex[vStart]);
                oldToNewVertexIndex[vStart] = newStartIdx;
            } else {
                newStartIdx = oldToNewVertexIndex[vStart];
            }

            // Map or add end vertex
            if (oldToNewVertexIndex.find(vEnd) == oldToNewVertexIndex.end()) {
                newEndIdx = newBREP.vertex.size();
                newBREP.vertex.push_back(tempBREP.vertex[vEnd]);
                oldToNewVertexIndex[vEnd] = newEndIdx;
            } else {
                newEndIdx = oldToNewVertexIndex[vEnd];
            }

            // Add edge to new BREP
            newBREP.edges.push_back(Edge(newStartIdx, newEndIdx, edge.style));
        }
        // Edge crosses the plane
        else if (signStart * signEnd < 0) {
            // Compute intersection point
            Vertex intersection = mathUtils.interpolateEdgeIntersection(edge, planeEqn, tempBREP);

            int intersectionIndex;
            std::pair<int, int> edgeKey = std::minmax(vStart, vEnd);
            if (edgeIntersectionVertexIndex.find(edgeKey) != edgeIntersectionVertexIndex.end()) {
                intersectionIndex = edgeIntersectionVertexIndex[edgeKey];
            } else {
                intersectionIndex = newBREP.vertex.size();
                newBREP.vertex.push_back(intersection);
                edgeIntersectionVertexIndex[edgeKey] = intersectionIndex;
            }

            // Map or add start and end vertices
            if (vertexSigns[vStart] == sideToKeep || vertexSigns[vStart] == 0) {
                int newStartIdx;
                if (oldToNewVertexIndex.find(vStart) == oldToNewVertexIndex.end()) {
                    newStartIdx = newBREP.vertex.size();
                    newBREP.vertex.push_back(tempBREP.vertex[vStart]);
                    oldToNewVertexIndex[vStart] = newStartIdx;
                } else {
                    newStartIdx = oldToNewVertexIndex[vStart];
                }
                newBREP.edges.push_back(Edge(newStartIdx, intersectionIndex, edge.style));
            }
            if (vertexSigns[vEnd] == sideToKeep || vertexSigns[vEnd] == 0) {
                int newEndIdx;
                if (oldToNewVertexIndex.find(vEnd) == oldToNewVertexIndex.end()) {
                    newEndIdx = newBREP.vertex.size();
                    newBREP.vertex.push_back(tempBREP.vertex[vEnd]);
                    oldToNewVertexIndex[vEnd] = newEndIdx;
                } else {
                    newEndIdx = oldToNewVertexIndex[vEnd];
                }
                newBREP.edges.push_back(Edge(intersectionIndex, newEndIdx, edge.style));
            }

            // Add cross-sectional edge
            crossSectionEdges.push_back(Edge(intersectionIndex, intersectionIndex, edge.style));  // Placeholder; we'll adjust this
        }
        // Edge lies on the plane
        else if (signStart == 0 && signEnd == 0) {
            int newStartIdx, newEndIdx;

            // Map or add start vertex
            if (oldToNewVertexIndex.find(vStart) == oldToNewVertexIndex.end()) {
                newStartIdx = newBREP.vertex.size();
                newBREP.vertex.push_back(tempBREP.vertex[vStart]);
                oldToNewVertexIndex[vStart] = newStartIdx;
            } else {
                newStartIdx = oldToNewVertexIndex[vStart];
            }

            // Map or add end vertex
            if (oldToNewVertexIndex.find(vEnd) == oldToNewVertexIndex.end()) {
                newEndIdx = newBREP.vertex.size();
                newBREP.vertex.push_back(tempBREP.vertex[vEnd]);
                oldToNewVertexIndex[vEnd] = newEndIdx;
            } else {
                newEndIdx = oldToNewVertexIndex[vEnd];
            }

            // Add cross-sectional edge
            crossSectionEdges.push_back(Edge(newStartIdx, newEndIdx, edge.style));
        }
    }

    // Build adjacency list from cross-sectional edges
    std::map<int, std::vector<int>> adjacencyList;
    for (const auto& edge : crossSectionEdges) {
        int v1 = edge.startVertexIndex;
        int v2 = edge.endVertexIndex;
        if (v1 != v2) {
            adjacencyList[v1].push_back(v2);
            adjacencyList[v2].push_back(v1);
        }
    }

    // Find loops from adjacency list
    std::vector<bool> visited(newBREP.vertex.size(), false);
    std::vector<Loop> loops;

    for (auto& adjEntry : adjacencyList) {
        int startVertex = adjEntry.first;
        if (visited[startVertex]) continue;

        std::vector<int> loopVertices;
        std::stack<int> stack;
        stack.push(startVertex);

        while (!stack.empty()) {
            int currentVertex = stack.top();
            stack.pop();

            if (visited[currentVertex]) continue;

            visited[currentVertex] = true;
            loopVertices.push_back(currentVertex);

            for (int neighbor : adjacencyList[currentVertex]) {
                if (!visited[neighbor]) {
                    stack.push(neighbor);
                }
            }
        }

        // Ensure the loop is closed
        if (loopVertices.front() != loopVertices.back()) {
            loopVertices.push_back(loopVertices.front());
        }

        // Create edges for the loop
        Loop newLoop;
        for (size_t i = 0; i < loopVertices.size() - 1; ++i) {
            int idx1 = loopVertices[i];
            int idx2 = loopVertices[i + 1];
            Edge loopEdge(idx1, idx2, 0);  // Style can be set as needed
            newLoop.edges.push_back(std::make_tuple(loopEdge, ""));
        }

        // Ensure edges are ordered
        orderEdgesInLoop(newLoop);

        // Determine if the loop is outer or inner
        newLoop.isOuter = isOuterLoop(newLoop);

        // Add loop to BREP
        newBREP.loops.push_back(newLoop);
        loops.push_back(newLoop);
    }

    // Construct faces from loops
    std::vector<Loop> outerLoops;
    std::vector<Loop> innerLoops;

    for (auto& loop : loops) {
        if (loop.isOuter) {
            outerLoops.push_back(loop);
        } else {
            innerLoops.push_back(loop);
        }
    }

    // Associate inner loops with outer loops to form faces
    for (auto& outerLoop : outerLoops) {
        Faces face;
        face.outerLoop = outerLoop;

        for (auto& hole : innerLoops) {
            if (isLoopInsideAnother(hole, outerLoop)) {
                face.innerLoops.push_back(hole);
            }
        }

        newBREP.faces.push_back(face);
    }

    // Display the result to the user
    std::cout << "Cross-section operation performed on a temporary BREP." << std::endl;
    newBREP.display();

    // Prompt the user to save the temporary BREP
    std::cout << "Do you want to save the cross-sectional BREP? (yes/no): ";
    std::string response;
    std::getline(std::cin, response);
    if (response == "yes" || response == "y") {
        // Save the temporary BREP as the current BREP
        currentBREP = newBREP;
        history.push_back({currentBREP, "crossSection"});
        redoStack.clear();
        std::cout << "Cross-sectional BREP saved." << std::endl;
    } else {
        std::cout << "Cross-sectional BREP discarded." << std::endl;
    }
}

bool BREPOperations::isOuterLoop(const Loop& loop) {
    Vertex normal = computeLoopNormal(loop);
    std::string plane = determineProjectionPlane(normal);
    float area = calculateSignedArea(loop, plane);

    // For consistency, we can define positive area as outer loop
    // and negative area as inner loop (hole)
    return area > 0.0f;
}

Vertex BREPOperations::computeLoopNormal(const Loop& loop) {
    Vertex normal(0.0f, 0.0f, 0.0f);
    for (size_t i = 0; i < loop.edges.size(); ++i) {
        const Edge& edge = std::get<0>(loop.edges[i]);
        const Vertex& current = currentBREP.vertex[edge.startVertexIndex];
        const Vertex& next = currentBREP.vertex[edge.endVertexIndex];

        normal.x += (current.y - next.y) * (current.z + next.z);
        normal.y += (current.z - next.z) * (current.x + next.x);
        normal.z += (current.x - next.x) * (current.y + next.y);
    }
    // Normalize the normal vector
    float length = std::sqrt(normal.x * normal.x + normal.y * normal.y + normal.z * normal.z);
    if (length > 1e-6) {
        normal.x /= length;
        normal.y /= length;
        normal.z /= length;
    }
    return normal;
}

std::string BREPOperations::determineProjectionPlane(const Vertex& normal) {
    float absX = std::abs(normal.x);
    float absY = std::abs(normal.y);
    float absZ = std::abs(normal.z);

    if (absZ >= absX && absZ >= absY) {
        return "xy";  // Project onto XY plane
    } else if (absX >= absY && absX >= absZ) {
        return "yz";  // Project onto YZ plane
    } else {
        return "xz";  // Project onto XZ plane
    }
}

float BREPOperations::calculateSignedArea(const Loop& loop, const std::string& plane) {
    float area = 0.0f;
    size_t n = loop.edges.size();
    for (size_t i = 0; i < n; ++i) {
        const Edge& edge = std::get<0>(loop.edges[i]);
        const Vertex& vi = currentBREP.vertex[edge.startVertexIndex];
        const Vertex& vj = currentBREP.vertex[edge.endVertexIndex];

        float xi, yi, xj, yj;
        if (plane == "xy") {
            xi = vi.x;
            yi = vi.y;
            xj = vj.x;
            yj = vj.y;
        } else if (plane == "yz") {
            xi = vi.y;
            yi = vi.z;
            xj = vj.y;
            yj = vj.z;
        } else {  // "xz"
            xi = vi.x;
            yi = vi.z;
            xj = vj.x;
            yj = vj.z;
        }

        area += (xi * yj - xj * yi);
    }
    area *= 0.5f;
    return area;
}
void BREPOperations::orderEdgesInLoop(Loop& loop) {
    if (loop.edges.empty()) return;

    std::vector<std::tuple<Edge, std::string>> orderedEdges;
    std::unordered_set<int> usedEdges;
    orderedEdges.push_back(loop.edges[0]);
    usedEdges.insert(0);

    int currentVertexIndex = std::get<0>(loop.edges[0]).endVertexIndex;

    while (orderedEdges.size() < loop.edges.size()) {
        bool found = false;
        for (size_t i = 1; i < loop.edges.size(); ++i) {
            if (usedEdges.find(i) != usedEdges.end()) continue;

            Edge edge = std::get<0>(loop.edges[i]);
            if (edge.startVertexIndex == currentVertexIndex) {
                orderedEdges.push_back(loop.edges[i]);
                usedEdges.insert(i);
                currentVertexIndex = edge.endVertexIndex;
                found = true;
                break;
            } else if (edge.endVertexIndex == currentVertexIndex) {
                // Reverse edge direction
                Edge reversedEdge = Edge(edge.endVertexIndex, edge.startVertexIndex, edge.style);
                orderedEdges.push_back(std::make_tuple(reversedEdge, ""));
                usedEdges.insert(i);
                currentVertexIndex = reversedEdge.endVertexIndex;
                found = true;
                break;
            }
        }
        if (!found) {
            std::cerr << "Error: Unable to order edges in loop." << std::endl;
            break;
        }
    }
    loop.edges = orderedEdges;
}

Vertex BREPOperations::computeLoopCentroid(const Loop& loop) {
    float x = 0.0f, y = 0.0f, z = 0.0f;
    int count = 0;
    for (const auto& edgeTuple : loop.edges) {
        const Edge& edge = std::get<0>(edgeTuple);
        const Vertex& v = currentBREP.vertex[edge.startVertexIndex];
        x += v.x;
        y += v.y;
        z += v.z;
        count++;
    }
    return Vertex(x / count, y / count, z / count);
}

bool BREPOperations::pointInPolygon(const Vertex& point, const Loop& loop) {
    // Project point and loop onto XY plane
    float x = point.x;
    float y = point.y;
    bool inside = false;
    int n = loop.edges.size();
    for (int i = 0, j = n - 1; i < n; j = i++) {
        const Edge& edgeI = std::get<0>(loop.edges[i]);
        const Edge& edgeJ = std::get<0>(loop.edges[j]);
        const Vertex& vi = currentBREP.vertex[edgeI.startVertexIndex];
        const Vertex& vj = currentBREP.vertex[edgeJ.startVertexIndex];
        if (((vi.y > y) != (vj.y > y)) &&
            (x < (vj.x - vi.x) * (y - vi.y) / (vj.y - vi.y + 1e-6) + vi.x)) {
            inside = !inside;
        }
    }
    return inside;
}

bool BREPOperations::isLoopInsideAnother(const Loop& innerLoop, const Loop& outerLoop) {
    Vertex centroid = computeLoopCentroid(innerLoop);
    return pointInPolygon(centroid, outerLoop);
}

void BREPOperations::constructFaces() {
    // Clear existing faces
    currentBREP.faces.clear();

    // Collect all loops
    std::vector<Loop> outerLoops;
    std::vector<Loop> innerLoops;

    for (auto& loop : currentBREP.loops) {
        // Ensure edges in the loop are ordered correctly
        orderEdgesInLoop(loop);

        // Determine if the loop is an outer loop or a hole
        loop.isOuter = isOuterLoop(loop);

        if (loop.isOuter) {
            outerLoops.push_back(loop);
        } else {
            innerLoops.push_back(loop);
        }
    }

    // Associate inner loops (holes) with their respective outer loops
    for (auto& outerLoop : outerLoops) {
        Faces face;
        face.outerLoop = outerLoop;

        for (auto& hole : innerLoops) {
            if (isLoopInsideAnother(hole, outerLoop)) {
                face.innerLoops.push_back(hole);
            }
        }

        // Add the face to the BREP
        currentBREP.faces.push_back(face);
    }
}
