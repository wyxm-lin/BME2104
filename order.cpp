#include "order.h"
#include "robot.h"
#include "atlas.h"
#include "item.h"
#include "port.h"

using std::queue;
using std::vector;
using std::sort;

/**
 * @brief go over all the items, calc the value of all orders, val = itemvalue / dis
*/
void GenerateOrders(Robot (&robot)[RobotNumber], queue <Item> Q, Port (&port)[PortNumber], Item (&ItemMap)[MapSize][MapSize], Atlas &atlas, int NowFrame) {
    vector <Order> ords[RobotNumber]; 
    while (Q.size()) {
        Item it = Q.front(); Q.pop();
        if(port[it.destination].isopen() == false) continue; // NOTE: this is for debug
        if (it != ItemMap[it.x][it.y]) { // this item has been taken, just kick out
            continue; // NOTE: what does it mean?
        }
        // if(it.isbooked()) continue;
        if (ItemMap[it.x][it.y].isbooked()) 
        {
            continue;
        }
        int aimport = it.destination;
        for (int i = 0; i < RobotNumber; i++) {
            if (atlas.color[robot[i].nowx][robot[i].nowy] != atlas.color[port[aimport].x][port[aimport].y]) { // robot and port are not in the same area
                continue;
            }

            if (robot[i].UnableTakeOrder()) {
                continue;
            }
            if (it.value < robot[i].ValueLimit) {
                continue;
            }
            Order ord;
            ord.DisItemToPort = port[aimport].GetDis(it.x, it.y);
            // ord.DisRobotToItem =  // TODO
            if(ord.DisRobotToItem + NowFrame + CONSTDELTA >= it.BirthFrame + ExistFrame) {  // FIXME CONSTDELTA
                continue;
            }
            ord.PortId = aimport;
            ord.RobotId = i;
            ord.val = (double)it.value / (ord.DisItemToPort + ord.DisRobotToItem);
            ord.it = it;
            ords[i].push_back(ord);
        }
    }

    for (int i = 0; i < RobotNumber; i++) {
        if (robot[i].UnableTakeOrder()) {
            continue;
        }
        // {
        //     using std::fstream;
        //     fstream out;
        //     out.open("order.txt", std::ios::app);
        //     out << "id is " << i << "will take order\n";
        //     out << "frame is " << NowFrame << std::endl;
        //     out << "the ord size is " << ords[i].size() << std::endl;
        //     out.close();
        // }
        sort(ords[i].begin(), ords[i].end());
        for(auto ord: ords[i]) {
            int px = ord.it.x;
            int py = ord.it.y;
            if(ItemMap[px][py].isbooked()) {
                continue;
            }
            ItemMap[px][py].book();
            robot[i].TakeOrder(ord.it);
            // AstarTest(robot, atlas, 1.0, NowFrame);
            // SearchPath(robot[i], atlas);
            AstarTimeEpsilon(robot[i], atlas, 1.0);
            // {
            //     using std::fstream;
            //     fstream out;
            //     out.open("order.txt", std::ios::app);                
            //     out << ord.it.x << " " << ord.it.y << " " << ord.it.value << " " << ord.it.destination << " " << ord.val << std::endl;
            //     out.close();
            // }
            break;
        }
    }
}
