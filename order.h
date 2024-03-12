#ifndef _ORDER_H_
#define _ORDER_H

#include "common.h"
#include "item.h"
#include "searchPath.h"


class Robot;
class Port;
class Item;
class Altas;

using std::queue;

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

void GenerateOrders(Robot (&robot)[RobotNumber], queue <Item> Q, Port (&port)[PortNumber], Item (&ItemMap)[MapSize][MapSize], Atlas &altas, int frameID);
void tryRetakeOrder(Robot &robot);

#endif