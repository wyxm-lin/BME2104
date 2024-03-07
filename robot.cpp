#include "robot.h"

void Robot::update(int x, int y, bool carry, bool available) {
    nowx = x, nowy = y;
    iscarry = carry, isavailable = available;
}