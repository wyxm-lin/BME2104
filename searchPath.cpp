#include "util.h"
#include "searchPath.h"
#include "robot.h"
#include "atlas.h"

using std::priority_queue;
using std::pair;
using std::make_pair;
using std::unordered_set;
using std::fstream;
using std::unordered_map;

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
    robot.pathIndex = 0;
}

void astarEpsilon(Robot &robot, Atlas& atlas, double epsilon) {
    if (!robot.IsWorking) {
        return;
    }

    pair<int, int> fa[MapSize + 5][MapSize + 5];
    for (int i = 0; i < MapSize; i++) {
        for (int j = 0; j < MapSize; j++) {
            fa[i][j] = {-1, -1};
        }
    }

    priority_queue <Node> openList;
    int startX = robot.nowx, startY = robot.nowy;
    int targetX = robot.targetX, targetY = robot.targetY;
    Node start = Node(startX, startY, 0, epsilon * (abs(startX - targetX) + abs(startY - targetY)));
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
            Node use4Search = Node(nextX, nextY, now.g + 1, epsilon * (abs(nextX - targetX) + abs(nextY - targetY)));
            openList.push(use4Search);
        }
    }

    vector<pair<int, int>> path;
    pair<int, int> elem = {targetX, targetY}; // add the target point into path, so path = (Start, Target]
    while (elem != make_pair(startX, startY)) {
        path.push_back(elem);
        elem = fa[elem.first][elem.second];
    }
    reverse(path.begin(), path.end());
    robot.path = path;
    robot.pathIndex = 0;
}

void AstarTimeEpsilon(Robot &robot, Atlas &atlas, double epsilon) {
    if (!robot.IsWorking) {
        return;
    }
    int NowFrame = robot.NowFrame;

    unordered_map<NodeWithTime, NodeWithTime> fa;

    priority_queue <NodeWithTime> openList;
    int startX = robot.nowx, startY = robot.nowy;   
    int targetX = robot.targetX, targetY = robot.targetY;
    NodeWithTime start = NodeWithTime(startX, startY, NowFrame, 0, epsilon * (abs(startX - targetX) + abs(startY - targetY)));
    openList.push(start);
    fa[start] = start;
    NodeWithTime Last;

    bool vis[MapSize + 5][MapSize + 5];
    memset(vis, false, sizeof(vis));
    vis[startX][startY] = true;

    while (!openList.empty()) {
        NodeWithTime now = openList.top();
        openList.pop();
        int x = now.x, y = now.y, Time = now.Time;
        if (x == targetX && y == targetY) {
            Last = now;
            break;
        }

        for (int i = 0; i < 4; i++) {
            int nextX = x + dx[i], nextY = y + dy[i], NextTime = Time + 1;
            if (valid(nextX, nextY) == false) {
                continue;
            }
            if (reachable(atlas.atlas[nextX][nextY], atlas.atlas[x][y]) == false) {
                continue;
            }
            if (vis[nextX][nextY]) { // has visited
                continue;
            }
            NodeWithTime use4Search = NodeWithTime(nextX, nextY, NextTime, now.g + 1, epsilon * (abs(nextX - targetX) + abs(nextY - targetY) + abs(NextTime - NowFrame)));
            fa[use4Search] = now;
            openList.push(use4Search);
        }
    }

    vector<NodeWithTime> pathWithTime; // pathWithTime is (start, targe]
    while (Last != start) {
        pathWithTime.push_back(Last);
        Last = fa[Last];
    }
    reverse(pathWithTime.begin(), pathWithTime.end());
    robot.pathWithTime = pathWithTime;
    robot.NodeWithTimeSet.insert(pathWithTime.begin(), pathWithTime.end());
}

void AstarTest(Robot (&robots)[RobotNumber], Atlas &atlas, double epsilon, int NowFrame) {
    for (int id = 0; id < RobotNumber; id ++) {
        if (!robots[id].IsWorking) {
            continue;
        }

        unordered_map<NodeWithTime, NodeWithTime> fa;
        unordered_set<NodeWithTime> ForbiddenSet;

        bool GridVisited[200][200]; // use for two dimension search
        memset(GridVisited, false, sizeof(GridVisited));

        priority_queue <NodeWithTime> openList;
        int startX = robots[id].nowx, startY = robots[id].nowy;
        int targetX = robots[id].targetX, targetY = robots[id].targetY;

        NodeWithTime start = NodeWithTime(startX, startY, NowFrame, 0, epsilon * (abs(startX - targetX) + abs(startY - targetY)));
        openList.push(start);
        fa[start] = start;
        NodeWithTime Last;

        while (!openList.empty()) {
            NodeWithTime now = openList.top();
            openList.pop();
            Last = now;
            int x = now.x, y = now.y, Time = now.Time;
            if (x == targetX && y == targetY) {
                break;
            }

            for (int i = 0; i < 4; i++) {
                int nextX = x + dx[i], nextY = y + dy[i], NextTime = Time + 1;
                if (valid(nextX, nextY) == false) {
                    continue;
                }
                if (reachable(atlas.atlas[nextX][nextY], atlas.atlas[x][y]) == false) {
                    continue;
                }
                if (GridVisited[nextX][nextY]) { // GridVisited
                    continue;
                }
                
                NodeWithTime use4Search = NodeWithTime(nextX, nextY, NextTime, now.g + 1, epsilon * (abs(nextX - targetX) + abs(nextY - targetY) + abs(NextTime - NowFrame)));
                NodeWithTime NextPosNowTime = NodeWithTime(nextX, nextY, Time, 0, 0);
                NodeWithTime NowPosNextTime = NodeWithTime(x, y, NextTime, 0, 0);
                
                if (ForbiddenSet.find(use4Search) != ForbiddenSet.end()) { // reducing search space
                    continue;
                }

                bool flag = true;
                for (int pre = 0; pre < id; pre ++) {
                    unordered_set<NodeWithTime>& it = robots[pre].NodeWithTimeSet;
                    if (it.find(use4Search) != it.end() || // vertex conflict
                        it.find(NextPosNowTime) != it.end() && it.find(NowPosNextTime) != it.end()) { // edge conflict
                        ForbiddenSet.insert(use4Search);

                        flag = false;
                        break;
                    }
                }
                if (flag == false) {
                    // FIXME go back
                    // 将目前的这个点再push进去
                    NodeWithTime NowPosNextTime = NodeWithTime(x, y, NextTime, 0, 0);                    
                    continue;
                }
                else {
                    fa[use4Search] = now;
                    GridVisited[nextX][nextY] = true; // two dimension search
                    openList.push(use4Search);
                }
                
            }
        }

        if (Last.x == targetX && Last.y == targetY) {
            vector<pair<int, int> > path;
            while (Last != start) {
                path.push_back({Last.x, Last.y});
                robots[id].NodeWithTimeSet.insert(Last);
                Last = fa[Last];
            }
            reverse(path.begin(), path.end());
            robots[id].path = path;
            robots[id].pathIndex = 0;
        }
        else {
            // TODO 没有找到路径
            robots[id].path = vector<pair<int, int> >();
            robots[id].pathIndex = -1;
        }
    }
}