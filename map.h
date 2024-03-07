#ifndef _MAP_H_
#define _MAP_H_

#include "common.h"
#include "robot.h"
#include "port.h"
#include "ship.h"

class Map {
public:
    MapStatus map[MapSize][MapSize];
    Robot robot[RobotNumber];
    Port port[PortNumber];
    Ship ship[ShipNumber];

    /*
    * @brief Initize the map, stage preprocess
    */
    void Init();

};

#endif