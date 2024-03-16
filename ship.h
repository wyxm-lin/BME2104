#ifndef _SHIP_H_
#define _SHIP_H_

#include "common.h"
#include "port.h"

class ShipOrder;

class Ship {
public:
    int id, capacity, target;
    int aimPort;
    int portLastGo = -1; // special for last frames, go to ports that has been shut down
    ShipStatus status;
    Port port;
    

    
    bool finishLoad = false;  // finish load at a port, NOT means the ship is full 
    bool shipFull = false;  // means the ship is full
    bool afterSell = false; // if this is true and target is -1, mean the ship is at the virtual point
    bool afterMove = false; // if this is true, means the ship is at the target port

    int NowFrame;
    int HaveLoad; // the amount of goods the ship has loaded
    bool AtVirtualPoint = false; // if this is true, means the ship is at the virtual point
    bool AtPortLoading = false; // if this is true, means the ship is at the port and loading
    bool AtPortWaiting = false; // if this is true, means the ship is at the port and waiting
    bool FromVirtualToPort = false; // if this is true, means the ship is from virtual point to port
    bool FromPortToVirtual = false; // if this is true, means the ship is from port to virtual point
    bool FromPortToPort = false; // if this is true, means the ship is from port to port
    bool PrintShip = false; // if this is true, means the ship should print 'ship'
    bool PrintGo = false; // if this is true, means the ship should go to virtual point to sell
    bool ShouldLeaveNowPort = false; // if this is true, means the ship should leave the port (because ship is full or now port is empty)
    int LeaveVirtualPointFrame; // the frame that the ship should leave the virtual point
    int oldTarget = -1; // the old pos of the ship (used for log)

    Ship():id(-1), capacity(-1), target(-1), aimPort(-1), status(ShipStatus::SHIPPING), HaveLoad(0) {}
    ~Ship() = default;

    /**
     * @brief Update ship info
    */
    void update(ShipStatus sta, int targ, int frameId);

    void MoveToPort(int tar);
    void Sell();
};

#endif