#include "math_utils.h"
#include <cmath>
#include <stdexcept>

Matrix MathUtils::rotationMatrix(const std::string& axis, float theta) {
    Matrix R;
    float cos_theta = cos(theta * M_PI / 180.0f);
    float sin_theta = sin(theta * M_PI / 180.0f);
    if (axis == "x") {
        R.data = {{1, 0, 0, 0},
                  {0, cos_theta, -sin_theta, 0},
                  {0, sin_theta, cos_theta, 0},
                  {0, 0, 0, 1}};
    } else if (axis == "y") {
        R.data = {{cos_theta, 0, sin_theta, 0},
                  {0, 1, 0, 0},
                  {-sin_theta, 0, cos_theta, 0},
                  {0, 0, 0, 1}};
    } else if (axis == "z") {
        R.data = {{cos_theta, -sin_theta, 0, 0},
                  {sin_theta, cos_theta, 0, 0},
                  {0, 0, 1, 0},
                  {0, 0, 0, 1}};
    } else {
        throw std::invalid_argument("Invalid axis specified.");
    }
    return R;
}

Matrix MathUtils::translationMatrix(const std::string& axis, float units) {
    Matrix T;
    T.data = {{1, 0, 0, 0},
              {0, 1, 0, 0},
              {0, 0, 1, 0},
              {0, 0, 0, 1}};
    if (axis == "x") {
        T.data[0][3] = units;
    } else if (axis == "y") {
        T.data[1][3] = units;
    } else if (axis == "z") {
        T.data[2][3] = units;
    } else {
        throw std::invalid_argument("Invalid axis specified.");
    }
    return T;
}

Matrix MathUtils::projectionMatrix(const std::string& view) {
    Matrix P;
    if (view == "top") {
        P.data = {{1, 0, 0, 0},
                  {0, 1, 0, 0},
                  {0, 0, 0, 0},
                  {0, 0, 0, 1}};
    } else if (view == "bottom") {
        P.data = {{1, 0, 0, 0},
                  {0, -1, 0, 0},
                  {0, 0, 0, 0},
                  {0, 0, 0, 1}};
    } else if (view == "front") {
        P.data = {{1, 0, 0, 0},
                  {0, 0, 0, 0},
                  {0, 0, 1, 0},
                  {0, 0, 0, 1}};
    } else if (view == "back") {
        P.data = {{-1, 0, 0, 0},
                  {0, 0, 0, 0},
                  {0, 0, 1, 0},
                  {0, 0, 0, 1}};
    } else if (view == "left") {
        P.data = {{0, 0, 0, 0},
                  {0, 1, 0, 0},
                  {0, 0, 1, 0},
                  {0, 0, 0, 1}};
    } else if (view == "right") {
        P.data = {{0, 0, 0, 0},
                  {0, -1, 0, 0},
                  {0, 0, 1, 0},
                  {0, 0, 0, 1}};
    } else {
        throw std::invalid_argument("Invalid view specified.");
    }
    return P;
}

float MathUtils::solvePlaneEquation(float a, float b, float c, float d, const Vertex& v) {
    return a * v.x + b * v.y + c * v.z + d;
}

Vertex MathUtils::interpolateEdgeIntersection(const Edge& e, const Plane& plane, const BREP& brep) {
    const Vertex& v1 = brep.vertex[e.startVertexIndex];
    const Vertex& v2 = brep.vertex[e.endVertexIndex];
    float numerator = -(plane.a * v1.x + plane.b * v1.y + plane.c * v1.z + plane.d);
    float denominator = plane.a * (v2.x - v1.x) + plane.b * (v2.y - v1.y) + plane.c * (v2.z - v1.z);
    if (denominator == 0) {
        throw std::runtime_error("Edge is parallel to the plane.");
    }
    float t = numerator / denominator;
    Vertex result;
    result.x = v1.x + t * (v2.x - v1.x);
    result.y = v1.y + t * (v2.y - v1.y);
    result.z = v1.z + t * (v2.z - v1.z);
    return result;
}
