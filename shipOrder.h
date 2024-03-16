#ifndef SHIPORDER_H
#define SHIPORDER_H

#include "common.h"
#include "port.h"
#include "ship.h"
#include <queue>
#include <fstream>
#include <vector>

class Ship;
class Port;

struct ShipOrder {
    Port port;
    int shipId, portId;
    double val;
    ShipOrder(): port(), shipId(-1), portId(-1), val(0.0) {}
    ShipOrder(Port p, int shipId_, int portId_, double value): port(p), shipId(shipId_), portId(portId_), val(value) {}

    bool operator < (const ShipOrder &a) const {
        return val < a.val;
    }
}; 

void GenerateShipOrders(Port (&port)[PortNumber], Ship (&ship)[ShipNumber], int NowFrame);

void HandleLastFrames(Port (&port)[PortNumber], Ship (&ship)[ShipNumber], int NowFrame);

void GenerateShipOrdersNew(Port (&port)[PortNumber], Ship (&ship)[ShipNumber], int NowFrame);

#endif // SHIPORDER_H