#ifndef _SHIP_H_
#define _SHIP_H_

#include "common.h"

class Ship {
public:
    int id, capacity, target;
    ShipStatus status;

    int NotMoveMoment;

    Ship():id(-1), capacity(-1), target(-1), status(ShipStatus::SHIPPING), NotMoveMoment(-1) {}
    ~Ship() = default;

    /**
     * @brief Update ship info
    */
    void update(ShipStatus sta, int targ);

    void MoveToPort(int tar);
    void Sell();
};

#endif