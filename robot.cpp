#include "util.h"
#include "robot.h"
#include "searchPath.h"

using std::cout;
using std::endl;
using std::bitset;
using std::queue;
using std::pair;
using std::priority_queue;

extern int color[MapSize][MapSize];

int RobotDis[RobotNumber][MapSize][MapSize];

void RobotDisUpdateBatch(Robot robot[RobotNumber], int start, int end) {
    for (int i = start; i <= end; i++) {
        RobotDisUpdate(robot[i].nowx, robot[i].nowy, i);
    }
}

void RobotDisUpdate(int RobotX, int RobotY, int id) {
    memset(RobotDis[id], -1, sizeof(RobotDis[id]));
    RobotDis[id][RobotX][RobotY] = 0;
    queue<pair<int, int>> q;
    q.push({RobotX, RobotY});
    while (q.size()) {
        int x_ = q.front().first, y_ = q.front().second;
        q.pop();
        for (int k = 0; k < 4; k++) {
            int nx = x_ + dx[k], ny = y_ + dy[k];
            if (valid(nx, ny) && RobotDis[id][nx][ny] == -1 && color[nx][ny] == color[x_][y_]) {
                RobotDis[id][nx][ny] = RobotDis[id][x_][y_] + 1;
                q.push({nx, ny});
            }
        }
    }
}

int RobotGetDis(int x, int y, int id) {
    return RobotDis[id][x][y];
}

struct RobotAstarNode {
    int x, y, g;
    double h;
    RobotAstarNode(int x_, int y_, int g_, double h_): x(x_), y(y_), g(g_), h(h_) {}
    bool operator < (const RobotAstarNode &b) const {
        return g + h > b.g + b.h;
    }
};

int RobotGetDis(int x_, int y_, int id, int robotX, int robotY) {
    double alpha = 0.5;
    memset(RobotDis[id], -1, sizeof(RobotDis[id]));
    priority_queue <RobotAstarNode> openList;
    int startX = robotX, startY = robotY;
    int targetX = x_, targetY = y_;
    RobotDis[id][startX][startY] = 0;
    openList.push(RobotAstarNode(startX, startY, 0, alpha * (abs(startX - targetX) + abs(startY - targetY))));
    while (openList.size()) {
        int x = openList.top().x, y = openList.top().y, g = openList.top().g;
        openList.pop();
        if (x == targetX && y == targetY) {
            return RobotDis[id][x][y];
        }
        for (int i = 0; i < 4; i++) {
            int nx = x + dx[i], ny = y + dy[i];
            if (valid(nx, ny) && color[nx][ny] == color[x][y] && RobotDis[id][nx][ny] == -1) {
                RobotDis[id][nx][ny] = RobotDis[id][x][y] + 1;
                openList.push(RobotAstarNode(nx, ny, g + 1, alpha * abs(nx - targetX) + abs(ny - targetY)));
            }
        }
    }
    return INF;
}


void Robot::update(int x, int y, bool carry, bool available, int frameID) {
    nowx = x, nowy = y;
    IsCarry = carry;
    IsAvailable = available;
    NowFrame = frameID;
    OccupiedNodeSet.insert(NodeWithTime(nowx, nowy, NowFrame, 0, 0)); // occupied the current position
}

bool Robot::UnableTakeOrder() { // FIXME how to define UnableTakeOrder
    if (IsAvailable == false) return true;
    // if (IsWorking == true) return true;
    if (IsCarry == true) return true;

    // NOTE if we allow the robot to switch order, isworking zhu-shi-diao, iscarry bu-yao-zhu-shi
    // NOTE remember to release the original order, if switching order
    return false;
}

void Robot::TakeOrder(Item it) {
    targetX = it.x;
    targetY = it.y;
    targetport = it.destination;
    IsWorking = true;
    carryItem = it;
}

void Robot::FakeGet() {
    // the next pos is target
    if (NextX == targetX && NextY == targetY) {
        printf("get %d\n", id);
    }
}

void Robot::RealGet(int PortX, int PortY) {
    if (nowx == targetX && nowy == targetY) {
        targetX = PortX;
        targetY = PortY;
        IsCarry = true;
    }
}

void Robot::FakePull(int PortX, int PortY) {
    // next position(if next position have)
    if (InPortArea(PortX, PortY, NextX, NextY)) {
        printf("pull %d\n", id);
    }
}

void Robot::RealPull() {
    IsWorking = false;
    IsCarry = false;
    oldPort = targetport;
    targetX = targetY = targetport = -1; // invalid
    for (int i = pathIndex; i < pathWithTime.size(); i++) { // erase unused position
        OccupiedNodeSet.erase(pathWithTime[i]);
    }
    pathWithTime.clear(); // clear the path
    pathIndex = -1; // set invalid
}

void Robot::pull() {
    // robot
    IsWorking = false;
    IsCarry = false;
    oldPort = targetport;
    targetX = targetY = targetport = -1; // invalid
    for (int i = pathIndex; i < pathWithTime.size(); i++) { // erase unused position
        OccupiedNodeSet.erase(pathWithTime[i]); // TODO modify the OccupiedNodeSet
    }
    pathWithTime.clear(); // clear the path
    pathIndex = -1; // set invalid
    printf("pull %d\n", id);
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

void Robot::move() { // maintain the (NextX, NextY)
    if (pathIndex == -1) { // no path to move (this variable is for debug when search path)
        NextX = nowx, NextY = nowy;
        return; 
    }
    if (pathWithTime[pathIndex].x == nowx && pathWithTime[pathIndex].y == nowy) { // wait
        NextX = pathWithTime[pathIndex].x, NextY = pathWithTime[pathIndex].y;
        ++ pathIndex;
        return;
    }
    else if (pathWithTime[pathIndex].x == nowx + 1) { // down
        NextX = pathWithTime[pathIndex].x, NextY = pathWithTime[pathIndex].y;
        printf("move %d %d\n", id, DOWN);
    }
    else if (pathWithTime[pathIndex].x == nowx - 1) { // up
        NextX = pathWithTime[pathIndex].x, NextY = pathWithTime[pathIndex].y;
        printf("move %d %d\n", id, UP);
    }
    else if (pathWithTime[pathIndex].y == nowy + 1) { // right
        NextX = pathWithTime[pathIndex].x, NextY = pathWithTime[pathIndex].y;
        printf("move %d %d\n", id, RIGHT);
    }
    else if (pathWithTime[pathIndex].y == nowy - 1) { // left
        NextX = pathWithTime[pathIndex].x, NextY = pathWithTime[pathIndex].y;
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