#ifndef _ROBOT_H_
#define _ROBOT_H_

#include <vector>
#include "item.h"
#include "searchPath.h"

using std::vector;
using std::pair;
using std::unordered_set;

class Robot {
public:
    int id, nowx, nowy;
    int targetX, targetY, targetport; // TODO order need to add targetport
    int oldPort = -1;
    bool IsCarry, IsAvailable, IsWorking;
    int ValueLimit; // when value >= ValueLimit, this robot work
    bool RecoverFlag; // this variable is for debug when search path
    int UnavailableMoment; // the moment when the robot is unavailable
    int NowFrame;
    unordered_set<NodeWithTime> OccupiedNodeSet;
    vector<NodeWithTime> pathWithTime;
    int pathIndex;
    Item carryItem;
    int NextX, NextY;

    Robot(): id(-1), nowx(-1), nowy(-1), targetX(-1), targetY(-1), targetport(-1), oldPort(-1), IsCarry(false), IsAvailable(true), 
            IsWorking(false), ValueLimit(-1), RecoverFlag(false), pathIndex(-1), UnavailableMoment(0), NextX(-1), NextY(-1), NowFrame(0)
            {
                OccupiedNodeSet.clear();
                pathWithTime.clear();
            }
    ~Robot() = default;

    /**
    * @brief Update Robot info each frame
    */
    void update(int x, int y, bool carry, bool available, int frameID);

    bool UnableTakeOrder();
    void TakeOrder(Item it);

    void get(int PortX, int PortY);
    void pull();
    void move();
    void FakeGet();
    void FakePull(int PortX, int PortY);
    void RealGet(int PortX, int PortY);
    void RealPull();
    


    /************Below variables and functions are for debug***************/
    void RobotPrintPath();
};

void RobotDisUpdateBatch(Robot robot[RobotNumber], int start, int end);
void RobotDisUpdate(int RobotX, int RobotY, int id);
int RobotGetDis(int x, int y, int id);

#endif