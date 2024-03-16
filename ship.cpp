#include "util.h"
#include "ship.h"
#include <vector>

extern std::vector <std::pair <std::pair<int, int>, int > > shipPathSize[ShipNumber];

void Ship::update(ShipStatus sta, int targ, int frameId) {
    status = sta, target = targ, NowFrame = frameId;
    // maintain the status of the ship
    AtVirtualPoint = AtPortLoading = AtPortWaiting = FromVirtualToPort = FromPortToVirtual = FromPortToPort = false;
    if (status == ShipStatus::SHIPPING) {
        if (target == -1) {
            AtVirtualPoint = true;
            HaveLoad = 0; // maintain this variable
        }
        else {
            AtPortLoading = true;
        }
    }
    else if (status == ShipStatus::MOVING) {
        if (target == -1) {
            FromPortToVirtual = true;
        }
        else {
            if (oldTarget == -1) {
                FromVirtualToPort = true;
            }
            else {
                FromPortToPort = true;
            }
        }
    }
    else if (status == ShipStatus::WAITING) {
        AtPortWaiting = true;
    }
}

void Ship::MoveToPort(int tar) {
    oldTarget = target;
    printf("ship %d %d\n", id, tar);
#ifdef LOG
    shipPathSize[id].push_back(std::make_pair(std::make_pair(tar, HaveLoad), NowFrame));
#endif
    afterMove = true;
    target = tar;
    LeaveVirtualPointFrame = NowFrame;
}

void Ship::Sell() {
    oldTarget = target;
    printf("go %d\n", id);
#ifdef LOG
    shipPathSize[id].push_back(std::make_pair(std::make_pair(-1, HaveLoad), NowFrame));
#endif
    afterSell = true;
    shipFull = false;
    finishLoad = false;
    // HaveLoad = 0;
    target = -1;
}