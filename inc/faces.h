#ifndef FACES_H
#define FACES_H

#include <vector>
#include "loop.h"

class Faces {
public:
    Loop outerLoop;
    std::vector<Loop> innerLoops;  // Holes within the face

    Faces();
};

#endif // FACES_H
