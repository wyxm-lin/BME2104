#ifndef _ROBOT_H_
#define _ROBOT_H_

class Robot {
public:
    int id, nowx, nowy;
    bool IsCarry, IsAvailable;

    /**
    * @brief Update Robot info each frame
    */
    void update(int x, int y, bool carry, bool available);
};

#endif