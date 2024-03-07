#ifndef _PORT_H_
#define _PORT_H_

#include "common.h"
#include "util.h"

class Map;

class Port {
public:
    int id, x, y, T, velocity;
    int dis[MapSize][MapSize];

    /**
     * @brief Initize the dis
     */
    void PortDisInit(Map* ctlr);

    /**
     * @brief log for debug
    */
    void PrintDis();

};

#endif