#ifndef _ROBOT_H_
#define _ROBOT_H_

#include <vector>
#include "item.h"

using std::vector;

class Robot {
public:
    int id, nowx, nowy;
    int targetX, targetY;
    bool IsCarry, IsAvailable, IsWorking;
    int ValueLimit; // when value >= ValueLimit, this robot work
    vector <int> path;

    Robot() = default;
    ~Robot() = default;

    /**
    * @brief Update Robot info each frame
    */
    void update(int x, int y, bool carry, bool available);

    bool UnableTakeOrder();

    void TakeOrder(Item it);
};

#endif