#ifndef BREP_OPERATIONS_H
#define BREP_OPERATIONS_H

#include <vector>
#include <string>
#include <map>
#include <utility>
#include "brep.h"
#include "math_utils.h"
#include "vertex.h"
#include "edge.h"
#include "loop.h"
#include "faces.h"
#include "plane.h"

class BREPOperations {
public:
    std::vector<std::pair<BREP, std::string>> history;
    std::vector<std::pair<BREP, std::string>> redoStack;
    BREP currentBREP;
    MathUtils mathUtils;

    BREPOperations();

    void loadBREP(const BREP& brep);

    void rotate(const std::string& axis, float theta);

    void translate(const std::string& axis, float units);

    void orthogonalProjection(const std::string& view);

    void crossSection(const Plane& planeEqn, const std::string& sign);

    void render(const std::string& filename);

    void save(const std::string& filename = "brep_saved.json");

    void reset();

    void undo();

    void redo();

    void displayCurrentBREP() const;
    void importBREP(const std::string& filename);

    void loadBREPFromFile(const std::string& filename);

private:
    bool isOuterLoop(const Loop& loop);

    Vertex computeLoopNormal(const Loop& loop);

    std::string determineProjectionPlane(const Vertex& normal);

    float calculateSignedArea(const Loop& loop, const std::string& plane);

    void orderEdgesInLoop(Loop& loop);

    Vertex computeLoopCentroid(const Loop& loop);

    bool pointInPolygon(const Vertex& point, const Loop& loop);

    bool isLoopInsideAnother(const Loop& innerLoop, const Loop& outerLoop);

    void constructFaces();
};

#endif // BREP_OPERATIONS_H
