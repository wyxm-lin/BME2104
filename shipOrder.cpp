#include "shipOrder.h"

using std::priority_queue;
using std::fstream;
using std::ios;
using std::endl;
using std::min;
using std::vector;

extern vector <int> portShutDown;

void ShipMoveDecision(Port (&port)[PortNumber], Ship (&ship)[ShipNumber]) {
    vector <Port> ports;
    for(int i = 0; i < PortNumber; i++) {
        ports.push_back(port[i]);
    }
    sort(ports.begin(), ports.end(), [=](const Port &a, const Port &b) -> bool {return a.T < b.T;});
    for(int i = 0; i < ShipNumber; i++) {
        ship[i].FixedTurnAim[2] = ports[i].id;
        ship[i].FixedTurnAim[1] = ports[PortNumber - i - 1].id;
        int RestFrame = TurnFrame - ports[i].T - ports[PortNumber - i - 1].T - 500 - 2;
        ship[i].WaitTime[1] = RestFrame / 2;
        ship[i].WaitTime[2] = RestFrame - ship[i].WaitTime[1];
    }
}

void GenerateShipOrders(Port (&port)[PortNumber], Ship (&ship)[ShipNumber], int NowFrame){
    priority_queue <ShipOrder> shipOrder;
    for(int i = 0; i < PortNumber; i++){
        if(port[i].isopen() == false) continue;
        if(port[i].isbooked) continue;
        double val = (double)port[i].nowItemCnt;    // TODO maybe change to another way
        shipOrder.push(ShipOrder(port[i], -1, i, val));
    }

    for (int i = 0; i < ShipNumber; i++) {
        if (ship[i].status == SHIPPING) {
            if (ship[i].portLastGo != -1) {   // only occurs in the last frames
                ship[i].HaveLoad = 0; 
                port[ship[i].portLastGo].isbooked = true;
                ship[i].aimPort = ship[i].portLastGo;
                ship[i].port = port[ship[i].portLastGo];
                ship[i].portLastGo = -1;
            }
            else if (ship[i].afterMove) { // ship just arrive at a new port
                ship[i].afterMove = false;
            }
            else if (ship[i].afterSell) { // ship just arrive at the virtual point
                ship[i].HaveLoad = 0; // NOTE
                ship[i].aimPort = -1; // detect where no port to go
                while (!shipOrder.empty()) {
                    int Id = shipOrder.top().portId;
                    if(port[Id].isbooked){  // maybe changed by other ships in the same frame
                        shipOrder.pop(); 
                        continue;
                    }
                    if (port[Id].T * 2 + NowFrame >= TotalFrame) { // add this line
                        shipOrder.pop();
                        continue;
                    }
                    shipOrder.pop(); 
                    // ship[i].afterSell = false;
                    port[Id].isbooked = true;
                    ship[i].aimPort = Id;
                    ship[i].port = port[Id];
                    break;
                }
            }
            else if (ship[i].shipFull) { // ship is full, go to sell 
                port[ship[i].target].isbooked = false;
            }
            else if (ship[i].finishLoad) { // ship is not full, but the port now is empty
                port[ship[i].target].isbooked = false;
                ship[i].aimPort = -1;
                while (!shipOrder.empty()) {
                    int Id = shipOrder.top().portId;
                    if(port[Id].isbooked) {  // maybe changed by other ships in the same frame
                        shipOrder.pop(); 
                        continue;
                    }
                    // if(Id == ship[i].target){    // won't happen and will cause bug
                    //     shipOrder.pop(); 
                    //     continue;
                    // }
                    shipOrder.pop();
                    port[Id].isbooked = true;
                    ship[i].aimPort = Id;
                    ship[i].finishLoad = false;
                    ship[i].port = port[Id];
                    break;
                }
                
            }
            else { // ship is simply loading
                if (port[i].T + NowFrame >= TotalFrame) { // must go to virtual point // NOTE this has a bug
                    port[i].isbooked = false;
                    ship[i].aimPort = -1;
                    ship[i].Sell();
                }
            }
        }
        else if (ship[i].status == MOVING) {
            // TODO maybe do nothing
        }
        else if (ship[i].status == WAITING) {
            // TODO
        }
    }
}

void HandleLastFrames(Port (&port)[PortNumber], Ship (&ship)[ShipNumber], int NowFrame) {
    // use to search for ports to be shut down
    auto cmp = [](Port a, Port b){
        return a.totalItemCnt > b.totalItemCnt; // TODO maybe change to another way
    };
    priority_queue <Port, vector<Port>, decltype(cmp)> ports(cmp);

    // first push the targets port, avoid redundant close
    for (int i = 0; i < ShipNumber; i++) {
        if (ship[i].status == SHIPPING || ship[i].status == MOVING) {
            if (ship[i].target != -1) { // moving to a certain port || at a certain port
                port[ship[i].target].close();
#ifdef LOG
                portShutDown.push_back(ship[i].target);
#endif
            }
        }
    }

    for(int i = 0; i < PortNumber; i++){
        if (port[i].isopen() == false) {
            continue;
        }
        ports.push(port[i]);
    }

    for (int i = 0; i < ShipNumber; i++) {
        if(ship[i].status == WAITING){
            // TODO
        }
        else if (ship[i].status == MOVING) {
            if (ship[i].target != -1) {   // moving to a certain port, close this after loading
                // port[ship[i].target].close();
                // portShutDown.push_back(ship[i].target);
            }
            else {      // moving to the virtual point, choose the last port to close
                ship[i].portLastGo = ports.top().id;
                ports.pop();
                port[ship[i].portLastGo].close();
#ifdef LOG
                portShutDown.push_back(ship[i].portLastGo);
#endif
            }
        }
        else if (ship[i].status == SHIPPING) {
            if (ship[i].target != -1) {  // loading at a certain port, close this after loading
                // port[ship[i].target].close();
                // portShutDown.push_back(ship[i].target);
            }
            else {      // be right at the virtual point, choose the last port to close
                ship[i].portLastGo = ports.top().id;
                ports.pop();
                port[ship[i].portLastGo].close();
#ifdef LOG
                portShutDown.push_back(ship[i].portLastGo);
#endif
            }
        }
    }
}

void GenerateShipOrdersNew(Port (&port)[PortNumber], Ship (&ship)[ShipNumber], int NowFrame){
    if (NowFrame == 1) {
        for (int i = 0; i < ShipNumber; i++) {
            ship[i].aimPort = i;
            ship[i].AtVirtualPoint = true;
            ship[i].PrintShip = true;
            port[i].isbooked = true;
        }
        return;
    }

    vector<ShipOrder> shipOrder;
    for (int i = 0; i < PortNumber; i++) {
        if (port[i].isopen() == false) {
            continue;
        }
        if (port[i].isbooked) {
            continue;
        }
        double val = (double)port[i].nowItemCnt;    // TODO maybe change to another way
        shipOrder.push_back(ShipOrder(port[i], -1, i, val));
    }
    sort(shipOrder.begin(), shipOrder.end());
    reverse(shipOrder.begin(), shipOrder.end());

    for (int i = 0; i < ShipNumber; i++) {
        ship[i].PrintGo = ship[i].PrintShip = false; // reset

        if (ship[i].FromPortToVirtual == true || ship[i].FromVirtualToPort == true || ship[i].FromPortToPort == true) {
            // TODO maybe do nothing
        }
        else if (ship[i].AtPortWaiting == true) {
            // TODO maybe do nothing (ps. Now this situation won't happen)
        }
        else if (ship[i].AtVirtualPoint) {
            for (int j = 0; j < shipOrder.size(); j++) {
                int Id = shipOrder[j].portId;
                if (port[Id].isbooked) {  // maybe changed by other ships in the same frame
                    continue;
                }
                if (NowFrame + port[Id].T * 2 + 1 <= TotalFrame) {
                    port[Id].isbooked = true;
                    ship[i].aimPort = Id;
                    ship[i].PrintShip = true;
                    break;
                }
            }
        }
        else if (ship[i].AtPortLoading) {
            if (ship[i].ShouldLeaveNowPort) {
                if (ship[i].HaveLoad == ship[i].capacity) { // ship is full
                    ship[i].aimPort = -1;
                    ship[i].PrintGo = true;
                }
                else { // ship is not full
                    bool TakeOrder = false;
                    for (int j = 0; j < shipOrder.size(); j++) {
                        int Id = shipOrder[j].portId;
                        if (port[Id].isbooked) {  // maybe changed by other ships in the same frame
                            continue;
                        }
                        if (Id == ship[i].target) { // NOTE not choose origin port
                            continue;
                        }
                        if (NowFrame + 500 + 1 + port[Id].T <= TotalFrame) {
                            int increase = min(ship[i].capacity - ship[i].HaveLoad, port[Id].nowItemCnt);
                            // int increase = min(ship[i].capacity - ship[i].HaveLoad, port[Id].nowItemCnt + (int)((double)Port[Id].totalItemCnt / NowFrame * 500));
                            int FirstPathTime = NowFrame - ship[i].LeaveVirtualPointFrame;
                            int NowPortId = ship[i].target;
                            if (ship[i].HaveLoad * (FirstPathTime + 500 + increase / port[Id].velocity + port[Id].T) >= (ship[i].HaveLoad + increase) * (FirstPathTime + port[NowPortId].T)) {
                                continue;
                            }
                            port[Id].isbooked = true;
                            ship[i].aimPort = Id;
                            ship[i].PrintShip = true;
                            TakeOrder = true;
                            break;
                        }
                    }
                    if (TakeOrder == false) {
                        ship[i].aimPort = -1;
                        ship[i].PrintGo = true;
                    }
                }
            }
            else { // ship is simply loading
                if (port[ship[i].target].T + NowFrame == TotalFrame) {
                    port[ship[i].target].isbooked = false;
                    ship[i].aimPort = -1;
                    ship[i].PrintGo = true;
                }
            }
        }
    }
}

