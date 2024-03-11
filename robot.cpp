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
    OccupiedNodeSet.insert(NodeWithTime(nowx, nowy, NowFrame, 0, 0)); // occupied the current position
    {
        if (id == 1 && 1400 <= NowFrame && NowFrame <= 2000) {
            {
                using std::fstream;
                fstream out;
                out.open("log.txt", std::ios::app);
                out << "frame is " << NowFrame << std::endl;
                out << "robot 1 is at " << nowx << " " << nowy << std::endl;
                for (int i = 0; i < pathWithTime.size(); i++) {
                    out << "(" << pathWithTime[i].x << ", " << pathWithTime[i].y << ") ";
                    if ((i + 1) % 10 == 0)
                        out << '\n';
                }
                out << "\n";
                out << IsAvailable << " " << IsCarry << " " << IsWorking << std::endl;
                out << targetX << " " << targetY << " " << targetport << std::endl;
                out.close();
            }
        }
    }
}

bool Robot::UnableTakeOrder() { // FIXME how to define UnableTakeOrder
    if (IsAvailable == false) return true;
    if (IsWorking == true) return true;
    // if (IsCarry == true) return true;
    return false;
}

void Robot::TakeOrder(Item it) {
    targetX = it.x;
    targetY = it.y;
    targetport = it.destination;
    IsWorking = true;
}

void Robot::pull() {
    IsWorking = false;
    IsCarry = false;
    targetX = targetY = targetport = -1; // invalid
    for (int i = pathIndex; i < pathWithTime.size(); i++) { // erase unused position
        OccupiedNodeSet.erase(pathWithTime[i]);
    }
    pathWithTime.clear(); // clear the path
    pathIndex = -1; // set invalid
    printf("pull %d\n", id);
    if (id == 1 && 1400 <= NowFrame && NowFrame <= 1600) {
        {
            using std::fstream;
            fstream out;
            out.open("log.txt", std::ios::app);
            out << "frame is " << NowFrame << std::endl;
            out << "robot 1 is at " << nowx << " " << nowy << std::endl;
            out << "robot 1 pull\n";
            out.close();
        }
    }
}

void Robot::get(int PortX, int PortY) {
    if (IsAvailable == false || IsCarry == true || IsWorking == false) {
        return;
    }
    if (nowx == targetX && nowy == targetY) {
        targetX = PortX;
        targetY = PortY;
        IsCarry = true;
        printf("get %d\n", id);
    }
}

void Robot::move() {
    if (pathIndex == -1) // no path to move (this variable is for debug when search path)
        return; 
    if (pathWithTime[pathIndex].x == nowx && pathWithTime[pathIndex].y == nowy) { // wait
        ++ pathIndex;
        return;
    }
    else if (pathWithTime[pathIndex].x == nowx + 1) { // down
        printf("move %d %d\n", id, DOWN);
    }
    else if (pathWithTime[pathIndex].x == nowx - 1) { // up
        printf("move %d %d\n", id, UP);
    }
    else if (pathWithTime[pathIndex].y == nowy + 1) { // right
        printf("move %d %d\n", id, RIGHT);
    }
    else if (pathWithTime[pathIndex].y == nowy - 1) { // left
        printf("move %d %d\n", id, LEFT);
    }
    OccupiedNodeSet.erase(NodeWithTime(nowx, nowy, NowFrame, 0, 0)); // erase the current position
    ++ pathIndex;
}



/************Below variables and functions are for debug***************/
void Robot::RobotPrintPath() {
    using std::cout;
    using std::endl;
    cout << "path.size is " << pathWithTime.size() << endl;
    cout << "the start point is (" << nowx << ", " << nowy << ")\n";
    for (int i = 0; i < pathWithTime.size(); i++) {
        cout << "(" << pathWithTime[i].x << ", " << pathWithTime[i].y << ") ";
        if ((i + 1) % 10 == 0)
            cout << '\n';
    }
    cout << "\nthe target point is (" << targetX << ", " << targetY << ")\n";
}