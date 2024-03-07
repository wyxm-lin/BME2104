#ifndef _MAP_H_
#define _MAP_H_

#include "common.h"
#include "util.h"
#include "robot.h"
#include "item.h"
#include "port.h"
#include "ship.h"

using std::queue;

class Map {
public:
    MapStatus map[MapSize][MapSize];
    int ItemValue[MapSize][MapSize];
    queue <Item> ItemList;
    Robot robot[RobotNumber];
    Port port[PortNumber];
    Ship ship[ShipNumber];

    /**
    * @brief Initize the map, stage preprocess
    */
    void Init();

    /**
     * @brief pop time-out-items from ItemList queue
    */
    void ItemTimeOutDisappear(int frameID);

    /**
    * @brief Run the program frame by frame
    */
    void RunByFrame();

};

#endif