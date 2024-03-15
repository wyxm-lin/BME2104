#include "shipOrder.h"

using std::priority_queue;
using std::fstream;
using std::ios;
using std::endl;
using std::vector;

extern vector <int> portShutDown;

void GenerateShipOrders(Port (&port)[PortNumber], Ship (&ship)[ShipNumber], int NowFrame){
    priority_queue <ShipOrder> shipOrder;
    for(int i = 0; i < PortNumber; i++){
        if(port[i].isopen() == false) continue;
        if(port[i].isbooked) continue;
        double val = (double)port[i].nowItemCnt;    // TODO
        shipOrder.push(ShipOrder(port[i], -1, i, val));
    }

    for (int i = 0; i < ShipNumber; i++) {
        if (ship[i].status == SHIPPING) {
            if(ship[i].portLastGo != -1){   // only occurs in the last frames
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
                    if(port[Id].isbooked){  // maybe changed by other ships in the same frame
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

void HandleLastFrames(Port (&port)[PortNumber], Ship (&ship)[ShipNumber], int NowFrame){
    // use to search for ports to be shut down
    auto cmp = [](Port a, Port b){
        return a.totalItemCnt > b.totalItemCnt;
    };
    priority_queue <Port, vector<Port>, decltype(cmp)> ports(cmp);

    // first push the targets port, avoid redundant close
    for(int i = 0; i < ShipNumber; i++){
        if(ship[i].status == SHIPPING || ship[i].status == MOVING){
            if(ship[i].target != -1){
                port[ship[i].target].close();
                portShutDown.push_back(ship[i].target);
            }
        }
    }

    for(int i = 0; i < PortNumber; i++){
        if(port[i].isopen() == false) continue;
        ports.push(port[i]);
    }

    for(int i = 0; i < ShipNumber; i++){
        if(ship[i].status == WAITING){
            // TODO
        }else if(ship[i].status == MOVING){
            if(ship[i].target != -1){   // moving to a certain port, close this after loading
                // port[ship[i].target].close();
                // portShutDown.push_back(ship[i].target);
            }else{      // moving to the virtual point, choose the last port to close
                ship[i].portLastGo = ports.top().id;
                ports.pop();
                port[ship[i].portLastGo].close();
                portShutDown.push_back(ship[i].portLastGo);
            }
        }else if(ship[i].status == SHIPPING){
            if(ship[i].target != -1){  // loading at a certain port, close this after loading
                // port[ship[i].target].close();
                // portShutDown.push_back(ship[i].target);
            }else{      // be right at the virtual point, choose the last port to close
                ship[i].portLastGo = ports.top().id;
                ports.pop();
                port[ship[i].portLastGo].close();
                portShutDown.push_back(ship[i].portLastGo);
            }
        }
    }
}

