#ifndef _COMMON_H_
#define _COMMON_H_

/**
 * @brief common definitions
*/

#include <iostream>
#include <cstdio>
#include <cstring>
#include <string>
#include <algorithm>
#include <queue>
#include <vector>
#include <fstream>
#include <unordered_set>
#include <bitset>
#include <unordered_map>
#include <cmath>
#include <set>
#include <thread>

// #define DEBUG

const int RobotNumber = 10; // the number of robot
const int ShipNumber = 5; // the number of ship
const int PortNumber = 10;
const int MapSize = 200; // the size of atlas
const int FrameLimit = 15000; // the limit of the frames
const int ExistFrame = 1000;
const int dx[] = {0, 0, -1, 1};
const int dy[] = {1, -1, 0, 0};
const int INF = 0x3f3f3f3f;
const int CONSTDELTA = 500; // FIXME
const int TotalFrame = 15000;
const double EPSILON = 1.0;
const int FrameLastTimeHandle = 12000;

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