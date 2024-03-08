#ifndef _ORDER_H_
#define _ORDER_H

#include "common.h"


class Robot;
class Port;
class Item;
class Altas;

struct Order {
    Item it;
    int RobotId, PortId;
    int DisItemToPort, DisRobotToItem;
    double val;
    bool operator<(const Order &a)const {
        return val < a.val;
    }
};

void GenerateOrders(Robot (&robot)[RobotNumber], queue <Item> Q, Port (&port)[PortNumber], Item (&ItemMap)[MapSize][MapSize]);

#endif