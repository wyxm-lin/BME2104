#include "util.h"
#include "robot.h"
#include "searchPath.h"

using std::cout;
using std::endl;
using std::bitset;

void Robot::update(int x, int y, bool carry, bool available, int frameID) {
    nowx = x, nowy = y;
    IsCarry = carry, IsAvailable = available;
    NowFrame = frameID;
}

bool Robot::UnableTakeOrder() {
    if(IsAvailable == false) return true;
    if(IsWorking == true) return true;
    return false;
}

void Robot::TakeOrder(Item it) {
    targetX = it.x;
    targetY = it.y;
    targetport = it.destination;
    IsWorking = true;
    IsPathGenerated = false; // FIXME this variable is for debug
}

void Robot::move() {
    if (pathIndex == -1)
        return;
    if (path[pathIndex].x == nowx + 1) {
        printf("move %d %d\n", id, DOWN);
    }
    else if (path[pathIndex].x == nowx - 1) {
        printf("move %d %d\n", id, UP);
    }
    else if (path[pathIndex].y == nowy + 1) {
        printf("move %d %d\n", id, RIGHT);
    }
    else if (path[pathIndex].y == nowy - 1) {
        printf("move %d %d\n", id, LEFT);
    }
    NodeWithTimeSet.erase(NodeWithTime(nowx, nowy, NowFrame, 0, 0)); // erase the current position
    pathIndex++;
}

void Robot::DropItem() {
    IsWorking = false;
    IsCarry = false;
    printf("pull %d\n", id);
}

void Robot::TakeItem(int NewX, int NewY) {
    IsCarry = true;
    targetX = NewX;
    targetY = NewY;
    printf("get %d\n", id);
}

/************Below variables and functions are for debug***************/
void Robot::RobotPrintPath() {
    using std::cout;
    using std::endl;
    cout << "path.size is " << path.size() << endl;
    cout << "the start point is (" << nowx << ", " << nowy << ")\n";
    for (int i = 0; i < path.size(); i++) {
        cout << "(" << path[i].x << ", " << path[i].y << ") ";
        if ((i + 1) % 10 == 0)
            cout << '\n';
    }
    cout << "\nthe target point is (" << targetX << ", " << targetY << ")\n";
}