#include "order.h"
#include "robot.h"
#include "atlas.h"
#include "item.h"
#include "port.h"

using std::queue;
using std::vector;
using std::sort;
using std::thread;

extern double AllItemAveValue;
extern int AllItemValue;
extern int AllItemNum;

extern vector <pair <pair<int, int>, int> > robotPathSize[RobotNumber];
extern vector <pair<int, int> > robotItemValue[RobotNumber];

extern int atlas[MapSize][MapSize];
extern int color[MapSize][MapSize];
extern int RobotDis[RobotNumber][MapSize][MapSize];

/**
 * @brief go over all the items, calc the value of all orders, val = itemvalue / dis
*/
// choose a port for the item
int ItemChoosePort(Item& it, Port (&port)[PortNumber]) {
    int aimport = -1, minDis = INF;
    for (int i = 0; i < PortNumber; i++) {
        if (port[i].isopen() == false) {
            continue;
        }
        int dis = PortGetDis(it.x, it.y, i);
        if (dis == -1) {
            continue;
        }
        if (port[i].isbooked == true) {
            dis /= 1.5;
        }
        // dis /= (port[i].velocity);
        if (dis != -1 && dis < minDis) {
            minDis = dis;
            aimport = i;;
        }
    }
    if (aimport != -1) {
        it.destination = aimport;
    }
    return aimport;
}

void GenerateOrdersNew(Robot (&robot)[RobotNumber], queue <pair<int, int>> Q, Port (&port)[PortNumber], Item (&ItemMap)[MapSize][MapSize], int NowFrame) {
    if (AllItemNum == 0) {
        return;
    }
    vector <Order> ords[RobotNumber];
    AllItemAveValue = ((double)AllItemValue) / AllItemNum; // calculate the average value of all generated items


    while (Q.size()) {
        int x = Q.front().first, y = Q.front().second;
        Q.pop();
        if (ItemMap[x][y] == EmptyItem) { // this item has been taken, just kick out
            continue;
        }
        if (NowFrame < 13500 && ItemMap[x][y].value < AllItemAveValue) { // NOTE
            continue;
        }
        if (ItemMap[x][y].isbooked()) 
        {
            continue;
        }
        for (int aimport = 0; aimport < PortNumber; aimport++) { 
            if (PortGetDis(x, y, aimport) == -1) {
                continue;
            }
            if(port[aimport].isopen() == false) {
                continue;
            }
            for (int i = 0; i < RobotNumber; i++) {
                if (color[robot[i].nowx][robot[i].nowy] != color[port[aimport].x][port[aimport].y]) { // robot and port are not in the same area
                    continue;
                }
                if (robot[i].UnableTakeOrder()) { // TODO think how to define this function to take order when in half way
                    continue;
                }
                if (ItemMap[x][y].value < robot[i].ValueLimit) {
                    continue;
                }
                if (robot[i].IsWorking == true && ItemMap[x][y].BirthFrame != NowFrame) {
                    continue;
                }
                Order ord;
                ord.DisItemToPort = PortGetDis(x, y, aimport); // exact distance from item to port
                if (robot[i].IsWorking == false || robot[i].oldPort == -1) {
                    ord.DisRobotToItem = RobotGetDis(x, y, i, robot[i].nowx, robot[i].nowy); // use A* searth
                }
                else {
                    ord.DisRobotToItem = PortGetDis(x, y, robot[i].oldPort);
                }

                if (ord.DisRobotToItem + NowFrame >= ItemMap[x][y].BirthFrame + ExistFrame) { // disappear
                    continue;
                }
                if (ord.DisRobotToItem + ord.DisItemToPort + NowFrame + port[aimport].T >= TotalFrame) { // no enough time to finish the order
                    continue;
                }
                ord.PortId = aimport;
                ord.RobotId = i;
                ord.val = (double)ItemMap[x][y].value / (ord.DisItemToPort + ord.DisRobotToItem); // TODO how to define the value of the order
                if (port[aimport].isbooked) {
                    ord.val *= 1.2;
                }
                ord.it = ItemMap[x][y];
                ord.it.destination = aimport;
                ords[i].push_back(ord);
            }
        }
    }

    // order-switch
    for (int i = 0; i < RobotNumber; i++) {
        if (robot[i].UnableTakeOrder()) {
            continue;
        }
        sort(ords[i].begin(), ords[i].end());
        for(auto ord: ords[i]) {
            int px = ord.it.x;
            int py = ord.it.y;
            if (robot[i].IsWorking == true && ItemMap[px][py] == robot[i].carryItem) {
                break; // if the robot has order taken and decide not to switch its order, give up
            }

            if (robot[i].IsWorking == true) {
#ifdef DEBUG
                using std::fstream;
                using std::endl;
                fstream out;
                out.open("switch-order.txt", std::ios::app);
                out << "Frame:" << NowFrame << endl;
                out << "Robot Id:" << i << endl << endl;
                // out << 
                out.close();
#endif
            }

            if (ItemMap[px][py].isbooked()) {
                continue;
            }


            if(robot[i].IsWorking) { // order-switch
                if(ord.it.value <= robot[i].carryItem.value || RobotGetDis(ord.it.x, ord.it.y, i, robot[i].nowx, robot[i].nowy) >= RobotGetDis(robot[i].carryItem.x, robot[i].carryItem.y, i, robot[i].nowx, robot[i].nowy)) {
                    continue;
                }
                
            }

            if (robot[i].IsWorking) {
                int PreviousTargetX = robot[i].targetX, PreviousTargetY = robot[i].targetY;
                ItemMap[PreviousTargetX][PreviousTargetY].release();
                // The robot decides to switch order, release the previous order
            }

            ItemMap[px][py].book();
            robot[i].TakeOrder(ord.it);
            AstarTimeEpsilonWithConflict(robot[i], EPSILON, robot);
#ifdef LOG
            robotPathSize[i].push_back({{robot[i].pathWithTime.size(), i}, NowFrame});
            robotItemValue[i].push_back({ord.it.value, ord.it.destination});
#endif
            break;
        }
    }
}

void GenerateOrdersVersion4(Robot (&robot)[RobotNumber], queue <pair<int, int>> Q, Port (&port)[PortNumber], Item (&ItemMap)[MapSize][MapSize], int NowFrame) {
    if (AllItemNum == 0) {
        return;
    }
    vector <Order> ords[RobotNumber];
    AllItemAveValue = ((double)AllItemValue) / AllItemNum; // calculate the average value of all items
    queue <pair<int, int>> Q2 = Q;
    while (Q.size()) {
        int x = Q.front().first, y = Q.front().second;
        Q.pop();
        if (ItemMap[x][y] == EmptyItem) { // this item has been taken, just kick out
            continue;
        }
        if (NowFrame < 13500 && ItemMap[x][y].value < AllItemAveValue) { // NOTE
            continue;
        }
        if (ItemMap[x][y].isbooked()) 
        {
            continue;
        }
        for(int aimport = 0; aimport < PortNumber; aimport++){
            if(PortGetDis(x, y, aimport) == -1){
                continue;
            }
            if(port[aimport].isopen() == false){
                continue;
            }
            for (int i = 0; i < RobotNumber; i++) {
                if (color[robot[i].nowx][robot[i].nowy] != color[port[aimport].x][port[aimport].y]) { // robot and port are not in the same area
                    continue;
                }
                if (robot[i].UnableTakeOrder()) { // TODO think how to define this function to take order when in half way
                    continue;
                }
                if (ItemMap[x][y].value < robot[i].ValueLimit) {
                    continue;
                }
                Order ord;
                ord.DisItemToPort = PortGetDis(x, y, aimport); // exact distance from item to port
                // ord.DisRobotToItem = RobotGetDis(x, y, i); // exact distance from robot to item
                if (robot[i].oldPort != -1) {
                    ord.DisRobotToItem = PortGetDis(x, y, robot[i].oldPort);
                } 
                else {
                    ord.DisRobotToItem = RobotGetDis(x, y, i);
                }
                if (ord.DisRobotToItem + NowFrame >= ItemMap[x][y].BirthFrame + ExistFrame) { // disappear
                    continue;
                }
                if (ord.DisRobotToItem + ord.DisItemToPort + NowFrame + port[aimport].T >= TotalFrame) { // no enough time to finish the order
                    continue;
                }
                ord.PortId = aimport;
                ord.RobotId = i;
                ord.val = (double)ItemMap[x][y].value / (ord.DisItemToPort + ord.DisRobotToItem); // TODO how to define the value of the order
                if(port[aimport].isbooked){
                    ord.val *= 1.2;
                }
                // ord.val += CalculateAdditionalValue(aimport, port, Q2, ItemMap, NowFrame, x, y) / 50000;
                ord.it = ItemMap[x][y];
                ord.it.destination = aimport;
                ords[i].push_back(ord);
            }
        }
        
    }

#ifdef DEBUG
    {
        std::fstream out;
        out.open("order.txt", std::ios::app);
        out << "----------frame: " << NowFrame << "-----------" << std::endl;
        for(int i = 0; i < RobotNumber; i++)
            out << "robot Id: " << i << "\torder size: " << ords[i].size() << std::endl;
        out.close();
    }
#endif

    for (int i = 0; i < RobotNumber; i++) {
        if (robot[i].UnableTakeOrder()) {
            continue;
        }
        sort(ords[i].begin(), ords[i].end());
        for(auto ord: ords[i]) {
            int px = ord.it.x;
            int py = ord.it.y;
            if (ItemMap[px][py].isbooked()) {
                continue;
            }
            ItemMap[px][py].book();
            robot[i].TakeOrder(ord.it);
            AstarTimeEpsilonWithConflict(robot[i], EPSILON, robot);
#ifdef LOG
            robotPathSize[i].push_back({{robot[i].pathWithTime.size(), i}, NowFrame});
            robotItemValue[i].push_back({ord.it.value, ord.it.destination});
#endif
            break;
        }
    }
}

double CalculateAdditionalValue(int portId, Port (&port)[PortNumber], queue <pair<int, int>> Q, Item (ItemMap)[MapSize][MapSize], int NowFrame, int outx, int outy){
    std::priority_queue <double> pq;
    while (Q.size()) {
        int x = Q.front().first, y = Q.front().second;
        if(x == outx && y == outy) {
            Q.pop();
            continue;
        }
        Q.pop();
        if (ItemMap[x][y] == EmptyItem) { // this item has been taken, just kick out
            continue;
        }
        if (NowFrame < 13500 && ItemMap[x][y].value < AllItemAveValue) { // NOTE
            continue;
        }
        if (ItemMap[x][y].isbooked()) {
            continue;
        }

        int dis = PortGetDis(x, y, portId);
        if(dis == -1) {
            continue;
        }

        int aimport = ItemChoosePort(ItemMap[x][y], port);
        if(aimport == -1) {
            continue;
        }
        dis += PortGetDis(x, y, aimport);
        double val = (double)ItemMap[x][y].value / dis;
        pq.push(val);
    }
    if(pq.empty()) {
        return 0;
    }
    double additionalValue = pq.top();
    return additionalValue;
}

void tryRetakeOrder(Robot &robot) {
    // if the robot is carry
    if (robot.IsCarry == true) { // TODO how to define the robot is carry
        return;
    }
    int remainDis = robot.pathWithTime.size() - robot.pathIndex;
    Item it = robot.carryItem;
    // TODO 20?
    if (remainDis + robot.NowFrame + 20 >= it.BirthFrame + ExistFrame) {  // set a threshold to make sure the robot won't take the disappeared item
        robot.carryItem = EmptyItem;
        robot.IsCarry = false;
        robot.IsWorking = false;
        robot.oldPort = -1;  // work with order value calculate 
        robot.targetX = robot.targetY = robot.targetport = -1;
        for (int i = robot.pathIndex; i < robot.pathWithTime.size(); i++) {
            robot.OccupiedNodeSet.erase(robot.pathWithTime[i]);
        }
        robot.pathWithTime.clear();
        robot.pathIndex = -1;
        return;
    }
}
