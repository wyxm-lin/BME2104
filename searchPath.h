#ifndef _SEARCHPATH_H_
#define _SEARCHPATH_H_

#include "common.h"

class Atlas;
class Robot;

struct Node {
    int x, y;
    int g, h;

    Node() = default;
    ~Node() = default;

    Node(int _x, int _y, int _g, int _h) : x(_x), y(_y), g(_g), h(_h) {}

    bool operator < (const Node &b) const {
        return g + h > b.g + b.h;
    }
    
};

void SearchPath(Robot &robot, Atlas& atlas);
void astar(Robot &robot, Atlas& atlas);

#endif