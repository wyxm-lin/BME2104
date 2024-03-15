#ifndef _ORDER_H_
#define _ORDER_H

#include "common.h"
#include "item.h"
#include "searchPath.h"


class Robot;
class Port;
class Item;

using std::queue;
using std::pair;

struct Order {
    Item it;
    int RobotId, PortId;
    int DisItemToPort, DisRobotToItem;
    double val;
    bool operator<(const Order &a)const {
        return val > a.val;
    }

    Order(): it(-1, -1, -1, -1, -1), RobotId(-1), PortId(-1), DisItemToPort(-1), DisRobotToItem(-1), val(0.0) {}
};

void GenerateOrdersNew(Robot (&robot)[RobotNumber], queue <pair<int, int>> Q, Port (&port)[PortNumber], Item (&ItemMap)[MapSize][MapSize], int frameID);
void GenerateOrders(Robot (&robot)[RobotNumber], queue <Item> Q, Port (&port)[PortNumber], Item (&ItemMap)[MapSize][MapSize], int frameID);
void tryRetakeOrder(Robot &robot);

#endif