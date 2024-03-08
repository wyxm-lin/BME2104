#ifndef _SHIP_H_
#define _SHIP_H_

#include "common.h"

class Ship {
public:
    int id, capacity, target;
    ShipStatus status;

    Ship() = default;
    ~Ship() = default;

    /**
     * @brief Update ship info
    */
    void update(ShipStatus sta, int targ);
};

#endif