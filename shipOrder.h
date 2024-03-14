#ifndef SHIPORDER_H
#define SHIPORDER_H

#include "common.h"
#include "port.h"

struct ShipOrder {
    Port port;
    int shipId, portId;
    double val;
    int dis2Target;
}; 


#endif // SHIPORDER_H