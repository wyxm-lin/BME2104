#include "util.h"
#include "item.h"

Item::Item(int frame, int x, int y, int val, int des): BirthFrame(frame), x(x), y(y), value(val), destination(des) {}

bool Item::isbooked() {
    return booked;
}

bool Item::operator!=(const Item &a)const {
    return (a.BirthFrame != BirthFrame) || (x != a.x) || (y != a.y) || (value != a.value);
}

void Item::book() {
    booked = true;
}