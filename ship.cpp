#include "util.h"
#include "ship.h"

void Ship::update(ShipStatus sta, int targ) {
    status = sta, target = targ;
}

void Ship::MoveToPort(int tar) {
    printf("ship %d %d\n", id, tar);
    afterMove = true;
    target = tar;
}

void Ship::Sell() {
    printf("go %d\n", id);
    afterSell = true;
    shipFull = false;
    finishLoad = false;
    HaveLoad = 0;
    target = -1;
}