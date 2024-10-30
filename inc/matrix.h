#ifndef MATRIX_H
#define MATRIX_H

#include <vector>
#include "vertex.h"

class Matrix {
public:
    std::vector<std::vector<float>> data;

    Matrix();

    Matrix multiply(const Matrix& m) const;

    Vertex transform(const Vertex& v) const;
};

#endif // MATRIX_H
