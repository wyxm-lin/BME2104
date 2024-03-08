#include "util.h"
#include "robot.h"

void Robot::update(int x, int y, bool carry, bool available) {
    nowx = x, nowy = y;
    IsCarry = carry, IsAvailable = available;
}

bool Robot::UnableTakeOrder() {
    if(IsAvailable == false || IsCarry == true) return true;
    return false;
}

void Robot::TakeOrder(Item it) {
    targetX = it.x;
    targetY = it.y;
    IsWorking = true;
}