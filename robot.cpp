#include "util.h"
#include "robot.h"

void Robot::update(int x, int y, bool carry, bool available) {
    nowx = x, nowy = y;
    IsCarry = carry, IsAvailable = available;
}

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