#ifndef _UTIL_H_
#define _UTIL_H_

/**
 * @brief util functions
*/

#include "common.h"

inline bool valid (int x, int y) {
    return x >= 0 && x < MapSize && y >= 0 && y < MapSize;
}

inline bool reachable (MapStatus x, MapStatus y) {
    if (x == WALL || y == WALL)
        return false;
    if (x == y)
        return true;
    else if (x == EMPTY && y == PORT)
        return true;
    else if (x == PORT && y == EMPTY)
        return true;
    else
        return false;
}

inline bool InPortArea(int PortX, int PortY, int x, int y) {
    if (PortX <= x && x <= PortX + 3 && PortY <= y && y <= PortY + 3) {
        return true;
    }
    return false;
}


#endif