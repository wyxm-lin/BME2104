#ifndef _PORT_H_
#define _PORT_H_

#include "common.h"

using std::string;

class Atlas;

class Port {
public:
    int id, x, y, T, velocity;
    int dis[MapSize][MapSize];

    Port() = default;
    ~Port() = default;

    /**
     * @brief Initize the dis
     */
    void PortDisInit(Atlas* atlas);

    /**
     * @brief below are the functions to test the correctness of the atlas
    */
    void PrintDis(int x_, int y_);

};

#endif