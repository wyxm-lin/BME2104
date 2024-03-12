/*
    How to run this file:
    Step1: build
    Step2: in the shell (directory: /BME2104/), run the following command:
        ./bin/test-one_robot
    Note: because this exe need to interact with the Judger, so please check some function and variables in debug state

    outline: this exe display ont robot take order once.

*/

#include "controller.h"

using namespace std;

int main() {
    // clear the log.txt
    // fstream out;
    // out.open("log.txt", ios::out);
    // out.close();
    Controller BME2104;
    BME2104.Init();
    for(int i = 2; i < RobotNumber; i ++) {
        BME2104.robot[i].ValueLimit = 100000;
    }
    for(int i = 0; i < 5; i++) {
        BME2104.port[i].open();
    }
    for(int i = 5; i < PortNumber; i++) {
        BME2104.port[i].close();
    }
    
    BME2104.RunByFrame();
    return 0;
}