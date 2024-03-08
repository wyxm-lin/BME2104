#ifndef _ROBOT_H_
#define _ROBOT_H_

#include <vector>
#include "item.h"

using std::vector;
using std::pair;

class Robot {
public:
    int id, nowx, nowy;
    int targetX, targetY;
    bool IsCarry, IsAvailable, IsWorking;
    bool IsPathGenerated;

    int ValueLimit; // when value >= ValueLimit, this robot work
    vector <pair<int, int> > path;
    int pathIndex;

    Robot() = default;
    ~Robot() = default;

    /**
    * @brief Update Robot info each frame
    */
    void update(int x, int y, bool carry, bool available);

    bool UnableTakeOrder();

    void TakeOrder(Item it);

    // Below are the functions that are not used in this project
    void RobotPrintPath();
};

#endif