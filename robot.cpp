#include "util.h"
#include "robot.h"

void Robot::update(int x, int y, bool carry, bool available) {
    nowx = x, nowy = y;
    IsCarry = carry, IsAvailable = available;
}