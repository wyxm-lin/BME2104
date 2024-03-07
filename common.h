#ifndef _COMMON_H_
#define _COMMON_H_

const int RobotNumber = 10; // the number of robot
const int ShipNumber = 5; // the number of ship
const int PortNumber = 10;
const int MapSize = 200; // the size of map
const int FrameLimit = 15000; // the limit of the frames

enum MapStatus {
    EMPTY = 0, // empty areas
    WALL = 1, // walls, not to go
    WATER = 2, // waters, do not fall in
    PORT = 3 // ship park here, robots come here
};

enum ShipStatus {
    MOVING = 0,
    FINISHED = 1,
    WAITING = 2
};

#endif