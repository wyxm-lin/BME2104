/*
    How to run this file:
    have not completed yet
*/

#include "controller.h"

using namespace std;

int main() {
    Controller BME2104;
    BME2104.Init();
    for(int i = 1; i < RobotNumber; i ++) {
        BME2104.robot[i].ValueLimit = 100000;
    }
    BME2104.PreProcess();
    BME2104.RunByFrame();
    return 0;
}