#include "test.h"

void Test::setA(int a) {
    a_ = a;
}

void Test::setB(int b) {
    b_ = b;
}

int Test::getA() {
    return a_;
}

int Test::getB() {
    return b_;
}

int Test::sum() {
    return a_ + b_;
}
