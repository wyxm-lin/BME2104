#include "shipOrder.h"

using std::priority_queue;
using std::fstream;
using std::ios;
using std::endl;

void GenerateShipOrdersNew(Port (&port)[PortNumber], Ship (&ship)[ShipNumber], int NowFrame){
    priority_queue <ShipOrder> shipOrder;
    for(int i = 0; i < PortNumber; i++){
        if(port[i].isopen() == false) continue;
        if(port[i].isbooked) continue;
        double val = (double)port[i].nowItemCnt;    // TODO
        shipOrder.push(ShipOrder(port[i], -1, i, val));
    }

    for (int i = 0; i < ShipNumber; i++) {
        if (ship[i].status == SHIPPING) {
            if (ship[i].afterMove) { // ship just arrive at a new port
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

void GenerateShipOrders(Port (&port)[PortNumber], Ship (&ship)[ShipNumber], int NowFrame){
    // TODO without considering the distance between the ship and the port
    // TODO last some frames, shut down 5 ports
    {
        if (NowFrame == 13500) {
            for (int i = 5; i < PortNumber; i ++) {
                port[i].close();
            }
        }
    }
    priority_queue <Port> heap;
    for (int i = 0; i < PortNumber; i++) {
        if (port[i].isbooked) { // NOTE thick twice, observe the Time ship from Virtual point to port
            continue;
        }
        if (port[i].isopen() == false)
            continue;
        heap.push(port[i]);
    }

    for (int i = 0; i < ShipNumber; i++) {
        if (ship[i].status == SHIPPING) {
            if (ship[i].afterMove) { // ship just arrive at a new port
                {
                    fstream out;
                    out.open("log.txt", std::ios::app);
                    out << "ship " << i << " arrive at " << ship[i].target << endl;
                    out << port[ship[i].target].T << endl;
                    out.close();
                }
                ship[i].afterMove = false;
            }
            else if (ship[i].afterSell) { // ship just arrive at the virtual point
                {
                    fstream out;
                    out.open("log.txt", std::ios::app);
                    out << "ship " << i << " arrive at virtual point" << endl;
                    out.close();
                }
                ship[i].HaveLoad = 0; // NOTE
                ship[i].aimPort = -1; // detect where no port to go
                while (!heap.empty()) {
                    int Id = heap.top().id;
                    if(Id == ship[i].target){
                        heap.pop(); 
                        continue;
                    }
                    if (port[Id].T * 2 + NowFrame >= TotalFrame) { // add this line
                        heap.pop();
                        continue;
                    }
                    heap.pop(); 
                    // ship[i].afterSell = false;
                    port[Id].isbooked = true;
                    ship[i].aimPort = Id;
                    break;
                }
            }
            else if (ship[i].shipFull) { // ship is full, go to sell
                {
                    fstream out;
                    out.open("log.txt", std::ios::app);
                    out << "ship " << i << " shipFull" << endl;
                    out.close();
                }
                port[ship[i].target].isbooked = false;
                // ship[i].Sell();
            }
            else if (ship[i].finishLoad) { // ship is not full, but the port now is empty
                {
                    fstream out;
                    out.open("log.txt", std::ios::app);
                    out << "ship " << i << " finishLoad" << endl;
                    out.close();
                }
                port[ship[i].target].isbooked = false;
                // {
                //     if (port[i].T + NowFrame == TotalFrame) {
                //         fstream out;
                //         out.open("log.txt", std::ios::app);
                //         out << "ship " << i << " sell at " << ship[i].target << endl;
                //         out.close();
                //     }
                // }
                ship[i].aimPort = -1;
                while (!heap.empty()) {
                    int Id = heap.top().id;
                    if(Id == ship[i].target){
                        heap.pop(); 
                        continue;
                    }
                    heap.pop();
                    port[Id].isbooked = true;
                    ship[i].aimPort = Id;
                    ship[i].finishLoad = false;
                    break;
                }
                
            }
            else { // ship is simply loading
                // if (ship[i].target != -1) {
                //     if (port[ship[i].target].T + NowFrame == TotalFrame - 1) { // NOTE
                //         ship[i].Sell();
                //         {
                //             fstream out;
                //             out.open("log.txt", std::ios::app);
                //             out << "ship " << i << " sell at " << ship[i].target << endl;
                //             out.close();
                        
                //         }
                //     }
                // }
                
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
