#include "vertex.h"
#include <cmath>

Vertex::Vertex() : x(0), y(0), z(0) {}

Vertex::Vertex(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}

bool Vertex::operator==(const Vertex& other) const {
    return std::abs(x - other.x) < 1e-6 && std::abs(y - other.y) < 1e-6 && std::abs(z - other.z) < 1e-6;
}
