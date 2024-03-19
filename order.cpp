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
void GenerateOrders(Robot (&robot)[RobotNumber], queue <Item> Q, Port (&port)[PortNumber], Item (&ItemMap)[MapSize][MapSize], int NowFrame) {
    if (AllItemNum == 0) {
        return;
    }
    vector <Order> ords[RobotNumber];
    AllItemAveValue = ((double)AllItemValue) / AllItemNum;
    for (int id = 0; id < RobotNumber; id++) {
        RobotDisUpdate(robot[id].nowx, robot[id].nowy, id);
    }
    while (Q.size()) {
        Item it = Q.front(); Q.pop();
        if (it != ItemMap[it.x][it.y]) { // this item has been taken, just kick out
            continue; // NOTE: what does it mean?
        }
        if (it.value < AllItemAveValue) { // NOTE
            continue;
        }
        if (ItemMap[it.x][it.y].isbooked()) 
        {
            continue;
        }
        int aimport = it.destination;
        for (int i = 0; i < RobotNumber; i++) {
            if (color[robot[i].nowx][robot[i].nowy] != color[port[aimport].x][port[aimport].y]) { // robot and port are not in the same area
                continue;
            }

            if (robot[i].UnableTakeOrder()) { // TODO think how to define this function to take order when in half way
                continue;
            }
            if (it.value < robot[i].ValueLimit) {
                continue;
            }
            Order ord;
            ord.DisItemToPort = PortGetDis(it.x, it.y, aimport); // exact distance from item to port
            ord.DisRobotToItem = RobotGetDis(it.x, it.y, i); // exact distance from robot to item
            if (ord.DisRobotToItem + NowFrame >= it.BirthFrame + ExistFrame) { // disappear
                continue;
            }
            if (ord.DisRobotToItem + ord.DisItemToPort + NowFrame + port[aimport].T >= TotalFrame) { // no enough time to finish the order
                continue;
            }
            ord.PortId = aimport;
            ord.RobotId = i;
            ord.val = (double)it.value / (ord.DisItemToPort + ord.DisRobotToItem);
            ord.it = it;
            ords[i].push_back(ord);
        }
    }

    for (int i = 0; i < RobotNumber; i++) {
        if (robot[i].UnableTakeOrder()) {
            continue;
        }
        sort(ords[i].begin(), ords[i].end());
        for(auto ord: ords[i]) {
            int px = ord.it.x;
            int py = ord.it.y;
            if(ItemMap[px][py].isbooked()) {
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

    // thread t[2];
    // t[0] = thread(RobotDisUpdateBatch, robot, 0, 4);
    // t[1] = thread(RobotDisUpdateBatch, robot, 5, 9);
    // t[0].join();
    // t[1].join();
    // thread t[RobotNumber];
    // for (int id = 0; id < RobotNumber; id++) {
    //     t[id] = thread(RobotDisUpdate, robot[id].nowx, robot[id].nowy, id);
    // }
    // for (int id = 0; id < RobotNumber; id++) {
    //     t[id].join();
    // }
    // for (int id = 0; id < RobotNumber; id++) {
    //     RobotDisUpdate(robot[id].nowx, robot[id].nowy, id);
    // }

    //FIXME how to use this BFS based on thread?

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
        // int aimport = ItemMap[x][y].destination;
        int aimport = ItemChoosePort(ItemMap[x][y], port); // choose the port: now just choose the nearest port
        if (aimport == -1) { // no port can be chosen
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
            // ord.DisRobotToItem = RobotGetDis(x, y, i, robot[i].nowx, robot[i].nowy);
            // {
            //     using std::fstream;
            //     using std::endl;
            //     using std::ios;
            //     fstream file;
            //     file.open("debug.txt", ios::out | ios::app);
            //     file << ord.DisRobotToItem << endl;
            //     file.close();
            // }
            // ord.DisRobotToItem = RobotGetDis(x, y, i); // exact distance from robot to item
            // if (robot[i].oldPort != -1) { // The robot has had taken order
            //     ord.DisRobotToItem = PortGetDis(x, y, robot[i].oldPort);
            //     // ord.DisRobotToItem = RobotGetDis(x, y, i);
            //     // NOTE FIXME modify here for switching order, turning to real-time robot-dis-to-item
            // } 
            // else { // The robot take order for the first time
            //     ord.DisRobotToItem = RobotGetDis(x, y, i);
            // }
            // if (robot[i].IsWorking == true) {
            //     ord.DisRobotToItem = RobotGetDis(x, y, i, robot[i].nowx, robot[i].nowy);
            // }
            // else {
            //     ord.DisRobotToItem = PortGetDis(x, y, robot[i].oldPort);
            // }
            if (robot[i].IsWorking == false || robot[i].oldPort == -1) {
                ord.DisRobotToItem = RobotGetDis(x, y, i, robot[i].nowx, robot[i].nowy);
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
            ord.it = ItemMap[x][y];
            ords[i].push_back(ord);
        }
    }

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


            if(robot[i].IsWorking) {
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

// don't matter
void GenerateOrdersVersion3(Robot (&robot)[RobotNumber], queue <pair<int, int>> Q, Port (&port)[PortNumber], Item (&ItemMap)[MapSize][MapSize], int NowFrame) {
    if (AllItemNum == 0) {
        return;
    }
    vector <Order> ords[RobotNumber];
    AllItemAveValue = ((double)AllItemValue) / AllItemNum; // calculate the average value of all items
    vector <Order> AllOrders;
    while (Q.size()) {
        int x = Q.front().first, y = Q.front().second;
        Q.pop();
        if (ItemMap[x][y] == EmptyItem) { // this item has been taken, just kick out
            continue;
        }
        if (ItemMap[x][y].value < AllItemAveValue) { // NOTE
            continue;
        }
        if (ItemMap[x][y].isbooked()) 
        {
            continue;
        }
        // int aimport = ItemMap[x][y].destination;
        int aimport = ItemChoosePort(ItemMap[x][y], port); // choose the port: now just choose the nearest port
        if (aimport == -1) { // no port can be chosen
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
            ord.it = ItemMap[x][y];
            // ords[i].push_back(ord);
            AllOrders.push_back(ord);
        }
    }

    sort(AllOrders.begin(), AllOrders.end());
    int WorkingRobotCnt = 0;
    for (int i = 0; i < RobotNumber; i++) {
        if (robot[i].IsWorking) {
            WorkingRobotCnt++;
        }
    }
    for (int i = 0; i < AllOrders.size(); i ++) {
        int RobotId = AllOrders[i].RobotId;
        if (robot[RobotId].UnableTakeOrder()) {
            continue;
        }
        int px = AllOrders[i].it.x;
        int py = AllOrders[i].it.y;
        if (ItemMap[px][py].isbooked()) {
            continue;
        }
        ItemMap[px][py].book();
        robot[RobotId].TakeOrder(AllOrders[i].it);
        AstarTimeEpsilonWithConflict(robot[RobotId], EPSILON, robot);
        WorkingRobotCnt ++;
        if (WorkingRobotCnt == RobotNumber) {
            break;
        }
    }
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
