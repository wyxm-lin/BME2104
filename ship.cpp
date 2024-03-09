#include "util.h"
#include "ship.h"

void Ship::update(ShipStatus sta, int targ) {
    status = sta, target = targ;
}

void Ship::MoveToPort(int tar) {
    printf("ship %d %d\n", id, tar);
    target = tar;
}

void Ship::Sell() {
    printf("go %d\n", id);
    target = -1;
}