#ifndef _ROBOT_H_
#define _ROBOT_H_

#include <vector>

using std::vector;
using std::pair;

class Robot {
public:
    int id, nowx, nowy;
    int targetX, targetY;
    bool IsCarry, IsAvailable, IsWorking;
    vector <pair<int, int> > path;

    Robot() = default;
    ~Robot() = default;

    /**
    * @brief Update Robot info each frame
    */
    void update(int x, int y, bool carry, bool available);

    // Below are the functions that are not used in this project
    void RobotPrintPath();
};

#endif