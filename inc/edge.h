#ifndef EDGE_H
#define EDGE_H

class Edge {
public:
    int startVertexIndex;
    int endVertexIndex;
    int style;
    Edge();
    Edge(int startIdx, int endIdx, int style_);
    bool operator==(const Edge& other) const;
};

#endif
