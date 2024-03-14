#include "order.h"
#include "robot.h"
#include "atlas.h"
#include "item.h"
#include "port.h"

using std::queue;
using std::vector;
using std::sort;

extern double AllItemAveValue;
extern int AllItemValue;
extern int AllItemNum;

/**
 * @brief go over all the items, calc the value of all orders, val = itemvalue / dis
*/
void GenerateOrders(Robot (&robot)[RobotNumber], queue <Item> Q, Port (&port)[PortNumber], Item (&ItemMap)[MapSize][MapSize], Atlas &atlas, int NowFrame) {
    vector <Order> ords[RobotNumber]; 
    if (AllItemNum == 0) {
        return;
    }
    AllItemAveValue = ((double)AllItemValue) / AllItemNum;
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
            if (atlas.color[robot[i].nowx][robot[i].nowy] != atlas.color[port[aimport].x][port[aimport].y]) { // robot and port are not in the same area
                continue;
            }

            if (robot[i].UnableTakeOrder()) { // TODO think how to define this function to take order when in half way
                continue;
            }
            if (it.value < robot[i].ValueLimit) {
                continue;
            }
            Order ord;
            ord.DisItemToPort = port[aimport].GetDis(it.x, it.y); // exact distance from item to port
            if(robot[i].oldPort == -1){     // the first order or order when the last item disapear
                ord.DisRobotToItem = 400; // get a high value to make sure the robot will take the order
            }else{  // other orders
                ord.DisRobotToItem = port[robot[i].oldPort].GetDis(it.x, it.y);
            }
            if(ord.DisRobotToItem + ord.DisItemToPort + NowFrame >= it.BirthFrame + ExistFrame) {
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
            AstarTimeEpsilonWithConflict(robot[i], atlas, EPSILON, robot);
            break;
        }
    }
}

void tryRetakeOrder(Robot &robot){
    int remainDis = robot.pathWithTime.size() - robot.pathIndex;
    Item it = robot.carryItem;
    if(remainDis + robot.NowFrame + 20 >= it.BirthFrame + ExistFrame) {  // set a threshold to make sure the robot won't take the disappeared item
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
