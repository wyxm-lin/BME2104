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

    int ValueLimit; // when value >= ValueLimit, this robot work
    vector <pair<int, int> > path;
    int pathIndex = 0;
    bool IsPathGenerated = false; // NOTE Consideration of whether the variable is needed

    Robot(): id(-1), nowx(-1), nowy(-1), targetX(-1), targetY(-1), IsCarry(false), IsAvailable(true), IsWorking(false), ValueLimit(0) {}
    ~Robot() = default;

    /**
    * @brief Update Robot info each frame
    */
    void update(int x, int y, bool carry, bool available);

    bool UnableTakeOrder();

    void TakeOrder(Item it);

    void Print();

    /************Below variables and functions are for debug***************/
    void RobotPrintPath();
    bool FinishFirstTakenOrder = false;
    bool HasFirstTakenOrder = false;
};

#endif