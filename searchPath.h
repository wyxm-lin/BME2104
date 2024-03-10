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
void astarEpsilon(Robot &robot, Atlas& atlas, double epsilon);

struct NodeWithTime {
    int x, y, Time;
    int g, h;

    NodeWithTime() = default;
    ~NodeWithTime() = default;

    NodeWithTime(int _x, int _y, int _Time, int _g, int _h) : x(_x), y(_y), Time(_Time), g(_g), h(_h) {}

    bool operator < (const NodeWithTime &b) const {
        return g + h > b.g + b.h;
    }
    bool operator == (const NodeWithTime &b) const {
        return x == b.x && y == b.y && Time == b.Time;
    }
    bool operator != (const NodeWithTime &b) const {
        return x != b.x || y != b.y || Time != b.Time;
    }

};

namespace std {
    template <>
    struct hash<NodeWithTime> {
        size_t operator()(const NodeWithTime &node) const {
            return node.x + node.y * 200 + node.Time * 40000;
        }
    };
}
void AstarTest(Robot (&robots)[RobotNumber], Atlas &atlas, double epsilon, int NowFrame);


#endif