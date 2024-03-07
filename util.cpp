#include "util.h"

bool in (int x, int y) {
    return x >= 0 && x < MapSize && y >= 0 && y < MapSize;
}