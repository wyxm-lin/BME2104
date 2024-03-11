#ifndef _SEARCHPATH_H_
#define _SEARCHPATH_H_

#include "common.h"
#include <bitset>
using std::bitset;

class Atlas;
class Robot;

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
            return node.x + node.y * 200 + node.Time * 40000; // NOTE: efficient hash
        }
    };
}

void bitsetReset(bitset<RobotNumber> (&a)[MapSize][MapSize], Robot (&robot)[RobotNumber]);
void avoidCollison(Robot (&robot)[RobotNumber], Atlas& atlas);
void SearchPath(Robot &robot, Atlas& atlas);
void AstarTimeEpsilon(Robot &robot, Atlas &atlas, double epsilon);
void AstarTimeEpsilonWithConflict(Robot &robot, Atlas &atlas, double epsilon, Robot (&otherRobot)[RobotNumber]);

#endif