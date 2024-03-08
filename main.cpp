#include "controller.h"

#define DEBUG

using namespace std;

int main() {
    Controller BME2104;
    BME2104.Init();


#ifdef DEBUG
    for(int i=1; i<RobotNumber; i++){
        BME2104.robot[i].ValueLimit = 0xdeadbeef;
    }
#endif
    BME2104.PreProcess();
    BME2104.RunByFrame();
    return 0;
}