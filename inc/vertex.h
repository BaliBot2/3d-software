#ifndef VERTEX_H
#define VERTEX_H

class Vertex {
public:
    float x, y, z;
    Vertex();
    Vertex(float x_, float y_, float z_);
    bool operator==(const Vertex& other) const;
};

#endif
