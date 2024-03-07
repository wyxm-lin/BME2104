#ifndef _UTIL_H_
#define _UTIL_H_

/**
 * @brief util functions
*/

#include "common.h"

inline bool in (int x, int y) {
    return x >= 0 && x < MapSize && y >= 0 && y < MapSize;
}

inline bool reachable (MapStatus x, MapStatus y) {
    if (x == WALL || y == WALL)
        return false;
    if (x == y)
        return true;
    else if (x == EMPTY && y == WATER)
        return true;
    else if (x == WATER && y == EMPTY)
        return true;
    else
        return false;
}

#endif