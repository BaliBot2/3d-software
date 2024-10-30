#include "matrix.h"

Matrix::Matrix() {
    data = std::vector<std::vector<float>>(4, std::vector<float>(4, 0));
}

Matrix Matrix::multiply(const Matrix& m) const {
    Matrix result;
    int n = data.size();
    int mCols = m.data[0].size();
    int p = data[0].size();
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < mCols; j++) {
            result.data[i][j] = 0;
            for (int k = 0; k < p; k++) {
                result.data[i][j] += data[i][k] * m.data[k][j];
            }
        }
    }
    return result;
}

Vertex Matrix::transform(const Vertex& v) const {
    Vertex result;
    float x = v.x, y = v.y, z = v.z;
    result.x = data[0][0] * x + data[0][1] * y + data[0][2] * z + data[0][3];
    result.y = data[1][0] * x + data[1][1] * y + data[1][2] * z + data[1][3];
    result.z = data[2][0] * x + data[2][1] * y + data[2][2] * z + data[2][3];
    return result;
}
