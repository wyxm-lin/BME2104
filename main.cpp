#include "controller.h"

using namespace std;

int main() {
    Controller BME2104;
    BME2104.Init();
    for(int i = 1; i < RobotNumber; i ++) {
        BME2104.robot[i].ValueLimit = 100000;
    }
    for(int i = 0; i < 5; i++) {
        BME2104.port[i].open();
    }
    for(int i = 5; i < PortNumber; i++) {
        BME2104.port[i].close();
    }
    
    BME2104.PreProcess();
    BME2104.RunByFrame();
    return 0;
}