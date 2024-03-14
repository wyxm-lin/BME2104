#ifndef _SHIP_H_
#define _SHIP_H_

#include "common.h"
#include "port.h"

class ShipOrder;

class Ship {
public:
    int id, capacity, target;
    int aimPort;
    ShipStatus status;
    Port port;

    int NotMoveMoment;
    int HaveLoad;
    bool finishLoad = false;  // finish load at a port, NOT means the ship is full 
    bool shipFull = false;  // means the ship is full
    bool afterSell = false; // if this is true and target is -1, mean the ship is at the virtual point
    bool afterMove = false; // if this is true, means the ship is at the target port

    Ship():id(-1), capacity(-1), target(-1), aimPort(-1), status(ShipStatus::SHIPPING), NotMoveMoment(-1), HaveLoad(0) {}
    ~Ship() = default;

    /**
     * @brief Update ship info
    */
    void update(ShipStatus sta, int targ);

    void MoveToPort(int tar);
    void Sell();
};

#endif