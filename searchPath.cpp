#include "util.h"
#include "searchPath.h"
#include "robot.h"
#include "atlas.h"
#include <fstream>

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

    // {
    //     if(robot[0].NowFrame != 1) {
    //         fstream out;
    //         out.open("bitsetReset1.txt", std::ios::app);
            
    //         for(int i = 0; i < MapSize; i++) {
    //             for(int j = 0; j < MapSize; j++) {
    //                 out << a[i][j] << " ";
    //             }
    //             out << std::endl;
    //         }
    //         out << "-------------- "<< robot[0].NowFrame << "--------------" << std::endl;
    //         out.close();
    //     }
    
    // }
}

void avoidCollison(Robot (&robot)[RobotNumber], Atlas& atlas) {
    bitset <RobotNumber> passThrough[MapSize][MapSize];
    // memset(passThrough, 0, sizeof(passThrough));

    // search for collision without time
    bitsetReset(passThrough, robot);

    // {
    //     if(robot[0].NowFrame == 2){
    //         fstream out;
    //         out.open("avoidCollison.txt", std::ios::app);
    //         for(int i = 0; i < MapSize; i++) {
    //             for(int j = 0; j < MapSize; j++) {
    //                 out << passThrough[i][j] << " ";
    //             }
    //             out << std::endl;
    //         }
    //         out << "-------------- "<< robot[0].NowFrame << "--------------" << std::endl;
    //         out.close();
    //     }
    // }

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
                //int i2 = passThrough[robot[i1].pathWithTime[p1].x][robot[i1].pathWithTime[p1].y]._Find_next(i1);
                
                int i2 = passThrough[robot[i1].pathWithTime[p1].x][robot[i1].pathWithTime[p1].y]._Find_first();
                if(i2 == i1) {
                    i2 = passThrough[robot[i1].pathWithTime[p1].x][robot[i1].pathWithTime[p1].y]._Find_next(i2);
                }

                // {
                //     fstream out;
                //     out.open("avoidCollison1.txt", std::ios::app);
                //     out << i1 << " " << time1in << " " << time1out << std::endl;
                //     out << i2 << " " << time2in << " " << time2out << std::endl;
                //     out.close();
                // }

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

                // get in the area in the same direction and NOT in the same time
                // TODO: redundant search
                if(robot[i1].pathWithTime[time1out].x == robot[i2].pathWithTime[time2out].x && robot[i1].pathWithTime[time1out].y == robot[i2].pathWithTime[time2out].y\
                    && time1out != time2out) {
                    continue;
                }

                // {
                //     fstream out;
                //     out.open("avoidCollison1.txt", std::ios::app);
                //     out << robot[i1].pathIndex << " " << time << std::endl;
                //     out << robot[i2].pathIndex << " " << time << std::endl;
                    
                //     out.close();
                // }

                // {
                //     fstream out;
                //     out.open("avoidCollison1.txt", std::ios::app);
                //     out << i1 << " " << time1in << " " << time1out << std::endl;
                //     out << i2 << " " << time2in << " " << time2out << std::endl;

                //     out << "------------i1----------------" << std::endl;
                //     out << robot[i1].pathWithTime[time1in].x << " " << robot[i1].pathWithTime[time1in].y << " " << robot[i1].pathWithTime[time1in].Time << std::endl;
                //     out << robot[i1].pathWithTime[time1out].x << " " << robot[i1].pathWithTime[time1out].y << " " << robot[i1].pathWithTime[time1out].Time << std::endl;
                //     out << "------------i2----------------" << std::endl;
                //     out << robot[i2].pathWithTime[time2in].x << " " << robot[i2].pathWithTime[time2in].y << " " << robot[i2].pathWithTime[time2in].Time << std::endl;
                //     out << robot[i2].pathWithTime[time2out].x << " " << robot[i2].pathWithTime[time2out].y << " " << robot[i2].pathWithTime[time2out].Time << std::endl;
                //     out.close();
                // }

                // if collision in time
                // i1 gets into the area first
                // set i2 to stop until i1 get out of the area
                vector<NodeWithTime> pathModify;
                int StopTime = time1out - time2in + 1;
                int cntTime = robot[i2].pathWithTime[0].Time;
                for(int timeCheck = 0; timeCheck < robot[i2].pathWithTime.size() + StopTime; timeCheck++) {
                    if(timeCheck < time2in) { // head
                        pathModify.push_back(robot[i2].pathWithTime[timeCheck]);
                    } else if(timeCheck >= time2in && timeCheck < time2in + StopTime) { // mid 
                        NodeWithTime tmp = robot[i2].pathWithTime[time2in - 1];
                        tmp.Time = cntTime;
                        pathModify.push_back(tmp); // stop at the same position
                    } else { // tail
                        NodeWithTime tmp = robot[i2].pathWithTime[timeCheck - StopTime];
                        tmp.Time = cntTime;
                        pathModify.push_back(tmp);
                    }
                    cntTime++;
                }

                // {
                //     fstream out;
                //     out.open("avoidCollison1.txt", std::ios::app);
                //     out << i1 << " " << time1in << " " << time1out << std::endl;
                //     out << i2 << " " << time2in << " " << time2out << std::endl;

                //     out << "------------i1----------------" << std::endl;
                //     for(int i = 0; i < robot[i1].pathWithTime.size(); i++) {
                //         out << robot[i1].pathWithTime[i].x << " " << robot[i1].pathWithTime[i].y << " " << robot[i1].pathWithTime[i].Time << std::endl;
                //     }
                //     out << "------------i2----------------" << std::endl;
                //     for(int i = 0; i < robot[i2].pathWithTime.size(); i++) {
                //         out << robot[i2].pathWithTime[i].x << " " << robot[i2].pathWithTime[i].y << " " << robot[i2].pathWithTime[i].Time << std::endl;
                //     }
                //     out << "------------i2----------------" << std::endl;
                //     for(int i = 0; i < pathModify.size(); i++) {
                //         out << pathModify[i].x << " " << pathModify[i].y << " " << pathModify[i].Time << std::endl;
                //     }
                //     out.close();
                // }

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