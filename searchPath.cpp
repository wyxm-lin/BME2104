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
    AstarTimeEpsilon(robot, atlas, 1.0); // FIXME epsilon value
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
    // memset(passThrough, 0, sizeof(passThrough));

    // search for collision without time
    bitsetReset(passThrough, robot);

    // add time
    bool modifyFlag = false;
    for(int time = 0; time <= FrameLimit - robot[0].NowFrame; time++){ // time is offset
        if(modifyFlag) {
            break;
        }
        for(int i1 = 0; i1 < RobotNumber; i1++) {
            if (!robot[i1].IsWorking) { // no working, no path
                continue;
            }

            // passThrough[robot[i1].pathWithTime[time].x][robot[i1].pathWithTime[time].y].reset(i1);

            // collision on area
            int p1 = robot[i1].pathIndex + time;
            if (passThrough[robot[i1].pathWithTime[p1].x][robot[i1].pathWithTime[p1].y].count() > 1) { // space conflict, time conflict maybe not exist
                // robot path search index
                

                // timeNin is the time when the robot enter the area
                // timeNout is the time when the robot leave the area but STILL IN the area
                int time1in = p1, time1out = p1, time2in = 0, time2out = 0;
                int i2 = passThrough[robot[i1].pathWithTime[p1].x][robot[i1].pathWithTime[p1].y]._Find_next(i1);    // i1 is strictly less than i2 
                // NOTE above line: if have more than one conflict, how to deal?

                // find time1out
                for(int timeCheck = p1; timeCheck < robot[i1].pathWithTime.size(); timeCheck++) {
                    if(passThrough[robot[i1].pathWithTime[timeCheck].x][robot[i1].pathWithTime[timeCheck].y].test(i2)) { // i1 and i2 have conflict (x, y, Time)
                        continue;
                    } else {
                        time1out = timeCheck - 1;   // make sure the robot is still in the area
                        break;
                    }
                }
                
                // check time dimension to see if i1 & i2 collide in time
                int p2 = robot[i2].pathIndex + time;
                for(int timeCheck = p2; timeCheck < robot[i2].pathWithTime.size(); timeCheck++) {
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
                    if(timeCheck < time2in) { // head
                        pathModify.push_back(robot[i2].pathWithTime[timeCheck]);
                    } else if(timeCheck >= time2in && timeCheck < time2in + StopTime) { // mid 
                        pathModify.push_back(robot[i2].pathWithTime[time2in - 1]); // stop at the same position
                    } else { // tail
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
            vis[nextX][nextY] = true; // add this line 
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
    robot.pathIndex = 0; // add this line NOTE: set pathIndex to 0
    robot.NodeWithTimeSet.insert(pathWithTime.begin(), pathWithTime.end());
}

void AstarTimeEpsilonWithConflict(Robot &robot, Atlas &atlas, double epsilon, Robot (&otherRobot)[RobotNumber]) {
    int NowRobotId = robot.id;
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
        Last = now;
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
            if (vis[nextX][nextY]) { // has visited
                continue;
            }
            NodeWithTime use4Search = NodeWithTime(nextX, nextY, NextTime, now.g + 1, epsilon * (abs(nextX - targetX) + abs(nextY - targetY) + abs(NextTime - NowFrame)));
            NodeWithTime NowTimeNextPos = NodeWithTime(nextX, nextY, Time, 0, 0);
            NodeWithTime NextTimeNowPos = NodeWithTime(x, y, NextTime, 0, 0);

            bool NoConflict = true;
            for (int other = 0; other < RobotNumber; other ++) {
                if (other == NowRobotId) {
                    continue;
                }
                if (otherRobot[other].NodeWithTimeSet.empty()) {
                    continue;
                }
                unordered_set<NodeWithTime>& otherRobotNodeWithTimeSet = otherRobot[other].NodeWithTimeSet;
                if ( otherRobotNodeWithTimeSet.find(use4Search) != otherRobotNodeWithTimeSet.end() || // vertex conflict
                    otherRobotNodeWithTimeSet.find(NowTimeNextPos) != otherRobotNodeWithTimeSet.end() && otherRobotNodeWithTimeSet.find(NextTimeNowPos) != otherRobotNodeWithTimeSet.end()) // edge conflict
                {
                    NoConflict = false;
                    break;
                }
            }
            if (NoConflict == false) {
                continue;
            }
            else {  
                vis[nextX][nextY] = true; // add this line 
                fa[use4Search] = now;
                openList.push(use4Search);
            }
        }
    }
    if (Last.x == targetX && Last.y == targetY) {
        vector<NodeWithTime> pathWithTime; // pathWithTime is (start, targe]
        while (Last != start) {
            pathWithTime.push_back(Last);
            Last = fa[Last];
        }
        reverse(pathWithTime.begin(), pathWithTime.end());
        robot.pathWithTime = pathWithTime;
        robot.pathIndex = 0; // add this line NOTE: set pathIndex to 0
        robot.NodeWithTimeSet.insert(pathWithTime.begin(), pathWithTime.end());
    }
    else {
        AstarTimeEpsilon(robot, atlas, epsilon); // if no path and no conflict, use AstarTimeEpsilon
    }
}
