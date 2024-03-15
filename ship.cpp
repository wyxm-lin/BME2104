#include "util.h"
#include "ship.h"
#include <vector>

extern std::vector <std::pair <std::pair<int, int>, int > > shipPathSize[ShipNumber];

void Ship::update(ShipStatus sta, int targ) {
    status = sta, target = targ;
}

void Ship::MoveToPort(int tar) {
    printf("ship %d %d\n", id, tar);
    shipPathSize[id].push_back(std::make_pair(std::make_pair(tar, HaveLoad), NowFrame));
    afterMove = true;
    target = tar;
}

void Ship::Sell() {
    printf("go %d\n", id);
    shipPathSize[id].push_back(std::make_pair(std::make_pair(-1, HaveLoad), NowFrame));
    afterSell = true;
    shipFull = false;
    finishLoad = false;
    // HaveLoad = 0;
    target = -1;
}