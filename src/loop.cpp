#include "loop.h"

Loop::Loop() : isOuter(true) {}

bool Loop::operator==(const Loop& other) const {
    return edges == other.edges && isOuter == other.isOuter;
}
