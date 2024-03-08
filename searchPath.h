#ifndef _SEARCHPATH_H_
#define _SEARCHPATH_H_

#include "common.h"

class Atlas;
class Robot;

struct Node {
    int x, y;
    int g, h;
    int forward;
    bool isVisited = false;

    Node() = default;
    ~Node() = default;

    Node(int _x, int _y, int _g, int _h, int _forward) : x(_x), y(_y), g(_g), h(_h), forward(_forward) {}

    bool operator == (const Node &b) const {
        return x == b.x && y == b.y;
    }

    bool operator < (const Node &b) const {
        return g + h > b.g + b.h;
    }
    
};

void SearchPath(Robot (&robot)[RobotNumber], Atlas& atlas);
void astar(Robot (&robot)[RobotNumber], Atlas& atlas);

#endif