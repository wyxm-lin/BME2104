#include "util.h"
#include "searchPath.h"
#include "robot.h"
#include "atlas.h"

using std::priority_queue;
using std::pair;
using std::make_pair;

void SearchPath(Robot &robot, Atlas& atlas) {
    astar(robot, atlas);
}

void astar(Robot &robot, Atlas& atlas) {
    if (!robot.IsWorking) {
        return;
    }

    pair<int, int> fa[MapSize + 5][MapSize + 5];
    for (int i = 0; i < MapSize; i ++) {
        for (int j = 0; j < MapSize; j ++) {
            fa[i][j] = {-1, -1};
        }
    }

    priority_queue <Node> openList;
    int startX = robot.nowx, startY = robot.nowy;
    int targetX = robot.targetX, targetY = robot.targetY;
    Node start = Node(startX, startY, 0, abs(startX - targetX) + abs(startY - targetY));
    openList.push(start);
    fa[startX][startY] = {startX, startY};

    while (!openList.empty()) {
        Node now = openList.top();
        openList.pop();
        int x = now.x, y = now.y;
        if (x == targetX && y == targetY) { // find the target
            break;
        }

        for (int i = 0; i < 4; i++) {
            int nextX = x + dx[i], nextY = y + dy[i];
            if (valid(nextX, nextY) == false) { // out of range
                continue;
            }
            if (reachable(atlas.atlas[nextX][nextY], atlas.atlas[x][y]) == false) {  // not reachable
                continue;
            }
            if (fa[nextX][nextY] != make_pair(-1, -1)) { // has been pushed into the openList
                continue;
            }
            fa[nextX][nextY] = make_pair(x, y);
            Node use4Search = Node(nextX, nextY, now.g + 1, abs(nextX - targetX) + abs(nextY - targetY));
            openList.push(use4Search);
        }
    }

    vector<pair<int, int> > path;
    // pair<int, int> elem = fa[targetX][targetY];
    pair<int, int> elem = {targetX, targetY}; // add the target point into path, so path = (Start, Target]
    while (elem != make_pair(startX, startY)) {
        path.push_back(elem);
        elem = fa[elem.first][elem.second];
    }
    reverse(path.begin(), path.end());
    robot.path = path;
}