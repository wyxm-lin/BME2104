#include "util.h"
#include "searchPath.h"
#include "robot.h"
#include "atlas.h"
#include <bitset>

using std::priority_queue;
using std::pair;
using std::make_pair;
using std::unordered_set;
using std::fstream;
using std::unordered_map;
using std::bitset;

void SearchPath(Robot &robot, Atlas& atlas) {
    astar(robot, atlas);
}

void bitsetReset(bitset<RobotNumber> (&a)[MapSize][MapSize], Robot (&robot)[RobotNumber]) {
    for(int i = 0; i < MapSize; i++) {
        for(int j = 0; j < MapSize; j++) {
            a[i][j].reset();
        }
    }

    for(int i = 0; i < RobotNumber; i++) {
        if(robot[i].IsWorking) {
            for(int j = 0; j < robot[i].pathWithTime.size(); j++) {
                a[robot[i].pathWithTime[j].x][robot[i].pathWithTime[j].y].set(i);
            }
        }
    }

}

void avoidCollison(Robot (&robot)[RobotNumber], Atlas& atlas) {
    bitset <RobotNumber> passThrough[MapSize][MapSize];
    memset(passThrough, 0, sizeof(passThrough));

    // search for collision without time
    bitsetReset(passThrough, robot);

    // add time
    bool modifyFlag = false;
    for(int time = 1; time <= FrameLimit - robot[0].NowFrame; time++){
        if(modifyFlag) {
            break;
        }
        for(int i1 = 0; i1 < RobotNumber; i1++) {
            if(!robot[i1].IsWorking) {continue;}

            // passThrough[robot[i1].pathWithTime[time].x][robot[i1].pathWithTime[time].y].reset(i1);

            // collision on area
            if(passThrough[robot[i1].pathWithTime[time].x][robot[i1].pathWithTime[time].y].count() > 1) {

                // timeNin is the time when the robot enter the area
                // timeNout is the time when the robot leave the area but STILL IN the area
                int time1in = time, time1out = time, time2in = 0, time2out = 0;
                int i2 = passThrough[robot[i1].pathWithTime[time].x][robot[i1].pathWithTime[time].y]._Find_next(i1);    // i1 is strictly less than i2

                // find time1out
                for(int timeCheck = time; timeCheck <robot[i1].pathWithTime.size(); timeCheck++) {
                    if(passThrough[robot[i1].pathWithTime[timeCheck].x][robot[i1].pathWithTime[timeCheck].y].test(i2)) {
                        continue;
                    } else {
                        time1out = timeCheck - 1;   // make sure the robot is still in the area
                        break;
                    }
                }
                
                // check time dimension to see if i1 & i2 collide in time
                for(int timeCheck = time; timeCheck < robot[i2].pathWithTime.size(); timeCheck++) {
                    if(time2in == 0 && passThrough[robot[i2].pathWithTime[timeCheck].x][robot[i2].pathWithTime[timeCheck].y].test(i1)) {
                        // first get in the area
                        time2in = timeCheck;
                        continue;
                    }else if(time2in != 0 && passThrough[robot[i2].pathWithTime[timeCheck].x][robot[i2].pathWithTime[timeCheck].y].test(i1)) {
                        // still in the area
                        continue;
                    }else if(time2in != 0 && !passThrough[robot[i2].pathWithTime[timeCheck].x][robot[i2].pathWithTime[timeCheck].y].test(i1)) {
                        // get out of the area
                        time2out = timeCheck - 1;
                        break;
                    }
                }
                
                // if no collision in time
                if(time1out < time2in || time2out < time1in) {
                    // return to search for next robot
                    continue;
                }

                // if collision in time
                // i1 gets into the area first
                // set i2 to stop until i1 get out of the area
                vector<NodeWithTime> pathModify;
                int StopTime = time1out - time2in + 1;
                for(int timeCheck = 0; timeCheck < robot[i2].pathWithTime.size() + StopTime; timeCheck++) {
                    if(timeCheck < time2in) {
                        pathModify.push_back(robot[i2].pathWithTime[timeCheck]);
                    } else if(timeCheck >= time2in && timeCheck < time2in + StopTime) {
                        pathModify.push_back(robot[i2].pathWithTime[timeCheck - 1]);
                    } else {
                        pathModify.push_back(robot[i2].pathWithTime[timeCheck - StopTime]);
                    }
                }
                robot[i2].pathWithTime = pathModify;
                bitsetReset(passThrough, robot);
                modifyFlag = true;
            }
            // passThrough[robot[i1].pathWithTime[time].x][robot[i1].pathWithTime[time].y].set(i1);
        }
    }

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
    robot.pathToBeDelete = path;
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
    robot.pathToBeDelete = path;
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
            robots[id].pathToBeDelete = path;
            robots[id].pathIndex = 0;
        }
        else {
            // TODO 没有找到路径
            robots[id].pathToBeDelete = vector<pair<int, int> >();
            robots[id].pathIndex = -1;
        }
    }
}