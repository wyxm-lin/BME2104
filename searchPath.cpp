#include "searchPath.h"


void SearchPath(Map $BME2104) {
    astar($BME2104);
}

void astar(Map &BME2104) {
    for(int robotId = 0; robotId < RobotNumber; robotId++){
        if(!BME2104.robot[robotId].IsWorking){
            continue;
        }
        std::priority_queue <Node> openList;
        vector <Node> closeList;
        int startX = BME2104.robot[robotId].nowx, startY = BME2104.robot[robotId].nowy;
        int targetX = BME2104.robot[robotId].targetX, targetY = BME2104.robot[robotId].targetY;
        Node start = Node(startX, startY, 0, abs(startX - targetX) + abs(startY - targetY), -1);
        
        openList.push(start);
        while(!openList.empty()){
            Node now = openList.top();
            openList.pop();
            int x = now.x, y = now.y;
            closeList.push_back(now);
            now.isVisited = true;
            if(x == targetX && y == targetY){
                break;
            }
            for(int i = 0; i < 4; i++){
                int nextX = x + dx[i], nextY = y + dy[i];
                if(nextX < 0 || nextX >= 200 || nextY < 0 || nextY >= 200){ // out of range
                    continue;
                }
                if(BME2104.map[nextX][nextY] == WALL){  // wall
                    continue;
                }
                if(find(closeList.begin(), closeList.end(), std::make_pair(nextX, nextY)) != closeList.end()){  // in closeList
                    continue;
                }
                if(now.isVisited){
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
        BME2104.robot[robotId].path = path;
    }
}