#ifndef MATH_UTILS_H
#define MATH_UTILS_H

#include <string>
#include "matrix.h"
#include "plane.h"
#include "vertex.h"
#include "edge.h"
#include "brep.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

class MathUtils {
public:
    Matrix rotationMatrix(const std::string& axis, float theta);

    Matrix translationMatrix(const std::string& axis, float units);

    Matrix projectionMatrix(const std::string& view);

    float solvePlaneEquation(float a, float b, float c, float d, const Vertex& v);

    Vertex interpolateEdgeIntersection(const Edge& e, const Plane& plane, const BREP& brep);
};

#endif // MATH_UTILS_H
