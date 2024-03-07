#ifndef _COMMON_H_
#define _COMMON_H_

/**
 * @brief common definitions
*/

#include <iostream>
#include <cstdio>
#include <queue>
#include <vector>

const int RobotNumber = 10; // the number of robot
const int ShipNumber = 5; // the number of ship
const int PortNumber = 10;
const int MapSize = 200; // the size of map
const int FrameLimit = 15000; // the limit of the frames
const int ExistFrame = 1000;
const int dx[] = {0, 0, -1, 1};
const int dy[] = {1, -1, 0, 0};

enum MapStatus {
    EMPTY = 0, // empty areas
    WALL = 1, // walls, not to go
    WATER = 2, // waters, do not fall in
    PORT = 3 // ship park here, robots come here
};

enum ShipStatus {
    MOVING = 0, // the ship is moving
    SHIPPING = 1, // the ship finished last task, you could give it further instruction
    WAITING = 2 // the ship is waiting out of the port
};

enum MoveDirection {
    RIGHT = 0,
    LEFT = 1,
    UP = 2,
    DOWN = 3
};

#endif