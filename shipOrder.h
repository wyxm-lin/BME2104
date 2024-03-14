#ifndef SHIPORDER_H
#define SHIPORDER_H

#include "common.h"
#include "port.h"
#include "ship.h"
#include <queue>
#include <fstream>

class Ship;
class Port;

struct ShipOrder {
    Port port;
    int shipId, portId;
    double val;
    ShipOrder(): port(), shipId(-1), portId(-1), val(0.0) {}
}; 

void GenerateShipOrders(Port (&port)[PortNumber], Ship (&ship)[ShipNumber], int NowFrame);


#endif // SHIPORDER_H