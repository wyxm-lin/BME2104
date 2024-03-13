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

    // fstream out;
    // out.open("log.txt", ios::out);
    // out.close();
    
    Controller BME2104;
    BME2104.Init();
    BME2104.RunByFrame();
    return 0;
}