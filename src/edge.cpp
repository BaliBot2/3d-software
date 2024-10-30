#include "edge.h"

Edge::Edge() : startVertexIndex(-1), endVertexIndex(-1), style(0) {}

Edge::Edge(int startIdx, int endIdx, int style_) : startVertexIndex(startIdx), endVertexIndex(endIdx), style(style_) {}

bool Edge::operator==(const Edge& other) const {
    return (startVertexIndex == other.startVertexIndex &&
            endVertexIndex == other.endVertexIndex &&
            style == other.style) ||
           (startVertexIndex == other.endVertexIndex &&
            endVertexIndex == other.startVertexIndex &&
            style == other.style);
}
