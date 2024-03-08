#include "util.h"
#include "searchPath.h"
#include "robot.h"
#include "atlas.h"

using std::priority_queue;

void SearchPath(Robot (&robot)[RobotNumber], Atlas& atlas) {
    astar(robot, atlas);
}

void astar(Robot (&robot)[RobotNumber], Atlas& atlas) {
    for (int robotId = 0; robotId < RobotNumber; robotId++) {
        if (!robot[robotId].IsWorking) {
            continue;
        }
        priority_queue <Node> openList;
        vector <Node> closeList;
        int startX = robot[robotId].nowx, startY = robot[robotId].nowy;
        int targetX = robot[robotId].targetX, targetY = robot[robotId].targetY;
        Node start = Node(startX, startY, 0, abs(startX - targetX) + abs(startY - targetY), -1);

        openList.push(start);
        while (!openList.empty()) {
            Node now = openList.top();
            openList.pop();
            int x = now.x, y = now.y;
            closeList.push_back(now);
            now.isVisited = true;
            if (x == targetX && y == targetY) {
                break;
            }
            for (int i = 0; i < 4; i++) {
                int nextX = x + dx[i], nextY = y + dy[i];
                if (in(nextX, nextY) == false) { // out of range
                        continue;
                }
                if (atlas.atlas[nextX][nextY] == WALL) {  // wall
                    continue;
                }
                Node use4Search = Node(nextX, nextY, now.g + 1, abs(nextX - targetX) + abs(nextY - targetY), i);   // only x and y are used here
                if (find(closeList.begin(), closeList.end(), use4Search) != closeList.end()) {  // in closeList
                    continue;
                }
                if (now.isVisited) {
                    continue;
                }
                now.isVisited = true;
                Node tmp = Node(nextX, nextY, now.g + 1, abs(nextX - targetX) + abs(nextY - targetY), i);   // i here refers how the previous node goes to this node
                openList.push(tmp);
            }
        }

        vector <int> path;
        Node now = closeList[closeList.size() - 1];
        while(now.x != startX || now.y != startY){
            path.push_back(now.forward);
            int preX = now.x - dx[now.forward];
            int preY = now.y - dy[now.forward];
            for(int i = 0; i < closeList.size(); i++){
                if(closeList[i].x == preX && closeList[i].y == preY){
                    now = closeList[i];
                    break;
                }
            }
        }
        reverse(path.begin(), path.end());
        robot[robotId].path = path;
    }
}