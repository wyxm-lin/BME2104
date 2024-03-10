#include <iostream>
#include "controller.h"
#include "searchPath.h"

using namespace std;

void MyFrameRun(Controller* BME2104) {
    int nowamoney = 0;
    while(BME2104->NowFrame < FrameLimit) {
        cin >> BME2104->NowFrame >> nowamoney;
        BME2104->ItemUpdateByFrame(BME2104->NowFrame);

        //FIXME

        for(int i = 0; i < RobotNumber; i++) {
            int carry, x, y, status;
            cin >> carry >> x >> y >> status;
            BME2104->robot[i].update(x, y, carry, status, BME2104->NowFrame);
        }
        // Robots Data Update

        for(int i = 0; i < ShipNumber; i++) {
            int status, tarid;
            cin >> status >> tarid;
            BME2104->ship[i].update((ShipStatus)status, tarid);
            // the status is accordingly assigned to integers, in 'common.h'
        }
        // Ship Data Update
        string OKstring;
        cin >> OKstring;
        // Read 'OK'

        printf("OK\n");
        fflush(stdout);
    }
}


int main() {
    Controller BME2104;
    BME2104.Init();
    MyFrameRun(&BME2104);

    return 0;
}