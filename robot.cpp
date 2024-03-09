#include "util.h"
#include "robot.h"
#include "searchPath.h"

using std::cout;
using std::endl;

void Robot::update(int x, int y, bool carry, bool available) {
    nowx = x, nowy = y;
    IsCarry = carry, IsAvailable = available;
}

bool Robot::UnableTakeOrder() {
    if(IsAvailable == false) return true;
    if(IsWorking == true) return true;
    return false;
}

void Robot::TakeOrder(Item it) {
    targetX = it.x;
    targetY = it.y;
    IsWorking = true;
    IsPathGenerated = false;
}

void Robot::Print() {
    if (IsWorking && FinishFirstTakenOrder == false) { // TODO consider the situation that when the robot get the Item, how to change robot's all state variables
        if (nowx == targetX && nowy == targetY) {
            cout << "get " << id << '\n';
            IsCarry = true;
            IsWorking = false; // FIXME just for test
            FinishFirstTakenOrder = true; // FIXME this varaible is for debug
        }
        else {
            if (path[pathIndex].first == nowx + 1) {
                cout << "move " << id << " " << DOWN << "\n";
            }
            else if (path[pathIndex].first == nowx - 1) {
                cout << "move " << id << " " << UP << "\n";
            }
            else if (path[pathIndex].second == nowy + 1) {
                cout << "move " << id << " " << RIGHT << "\n";
            }
            else if (path[pathIndex].second == nowy - 1) {
                cout << "move " << id << " " << LEFT << "\n";
            }
            pathIndex++;
        }
    }
}

/************Below variables and functions are for debug***************/
void Robot::RobotPrintPath() {
    using std::cout;
    using std::endl;
    cout << "path.size is " << path.size() << endl;
    cout << "the start point is (" << nowx << ", " << nowy << ")\n";
    for (int i = 0; i < path.size(); i++) {
        cout << "(" << path[i].first << ", " << path[i].second << ") ";
        if ((i + 1) % 10 == 0)
            cout << '\n';
    }
    cout << "\nthe target point is (" << targetX << ", " << targetY << ")\n";
}