/*

    How to run this file:
    Step1: build
    Step2: in the shell (directory: /BME2104/), run the following command:
        ../ContestProblem/LinuxRelease/PreliminaryJudge ./bin/just_move -m ../ContestProblem/LinuxRelease/maps/map1.txt
        (Note: need to modify the replayer's relative path) 

*/

#include "controller.h"
#include "common.h"
#include "searchPath.h"

using namespace std;

void justMove(Controller* BME);

int main() {
    Controller BME2104;
    BME2104.Init();
    for(int i = 1; i < RobotNumber; i ++) {
        BME2104.robot[i].ValueLimit = 100000;
    }
    BME2104.PreProcess();
    justMove(&BME2104);
    return 0;
}

void justMove(Controller* BME) {
    // when run, clear the file
    fstream out;
    out.open("output.txt", ios::out);
    out.close();

    int frameID = 0, nowamoney = 0;
    while(frameID < FrameLimit) {
        cin >> frameID >> nowamoney;
        BME->ItemUpdateByFrame(frameID);

        for(int i = 0; i < RobotNumber; i++) {
            int carry, x, y, status;
            cin >> carry >> x >> y >> status;
            BME->robot[i].update(x, y, carry, status);
        }
        // Robots Data Update

        for(int i = 0; i < ShipNumber; i++) {
            int status, tarid;
            cin >> status >> tarid;
            BME->ship[i].update((ShipStatus)status, tarid);
            // the status is accordingly assigned to integers, in 'common.h'
        }
        // Ship Data Update
        string OKstring;
        cin >> OKstring;
        // Read 'OK'

        /********************************Out Test**********************************/
        for (int i = 0; i < RobotNumber; i++) {
            if (BME->robot[i].IsWorking == false) { // use this variable to avoid setting target repeatedly every frame
                BME->robot[i].IsWorking = true;
                if (BME->robot[i].nowx == 64 && BME->robot[i].nowy == 75) { // assign the just a robot to move to (90, 27)
                    BME->robot[i].targetX = 90;
                    BME->robot[i].targetY = 27;
                    SearchPath(BME->robot[i], BME->atlas); 
                    BME->robot[i].IsPathGenerated = true;
                    BME->robot[i].pathIndex = 0;
                }
            }
        }

        for (int i = 0; i < RobotNumber; i ++) {
            if (BME->robot[i].IsPathGenerated) { // if the path is generated, move the robot
                BME->robot[i].Print();
            }
        }
        cout << "OK\n";
    }
}



// #include "controller.h"
// #include "searchPath.h"
// #include <iostream>
// #include <fstream>

// using namespace std;

// void justRun(Controller* BME) {
//     fstream out;
//     out.open("output.txt", ios::out);
//     out.close();

//     int frameID = 0, nowamoney = 0;
//     while(frameID < FrameLimit) {
//         cin >> frameID >> nowamoney;
//         BME->ItemUpdateByFrame(frameID);

//         for(int i = 0; i < RobotNumber; i++) {
//             int carry, x, y, status;
//             cin >> carry >> x >> y >> status;
//             BME->robot[i].update(x, y, carry, status);
//         }
//         // Robots Data Update

//         for(int i = 0; i < ShipNumber; i++) {
//             int status, tarid;
//             cin >> status >> tarid;
//             BME->ship[i].update((ShipStatus)status, tarid);
//             // the status is accordingly assigned to integers, in 'common.h'
//         }
//         // Ship Data Update
//         string OKstring;
//         cin >> OKstring;
//         // Read 'OK'

//         for (int i = 0; i < RobotNumber; i++) {
//             if (BME->robot[i].IsWorking == false) {
//                 BME->robot[i].IsWorking = true;
//                 if (BME->robot[i].nowx == 64 && BME->robot[i].nowy == 75) {
//                     BME->robot[i].targetX = 90;
//                     BME->robot[i].targetY = 27;
//                     // fstream out;
//                     // out.open("output.txt", ios::out | ios::app);
//                     // out << i << " " << "is in setting target\n";
//                     // out << BME->robot[i].nowx << " " << BME->robot[i].nowy << " " << BME->robot[i].targetX << " " << BME->robot[i].targetY << endl;
//                     // out.close();
//                     SearchPath(BME->robot[i], BME->atlas); 
//                     BME->robot[i].IsPathGenerated = true;
//                     BME->robot[i].pathIndex = 0;
//                 }
//             }
//         }

//         for (int i = 0; i < RobotNumber; i ++) {
//             if (BME->robot[i].IsPathGenerated) {
//                 if (BME->robot[i].pathIndex >= BME->robot[i].path.size()) {
//                     if (BME->robot[i].pathIndex == BME->robot[i].path.size()) {
//                         fstream out;
//                         out.open("output.txt", ios::out | ios::app);
//                         out << "get " << i << endl;
//                         out.close();
//                         cout << "get " << i << endl;
//                         BME->robot[i].pathIndex ++;
//                     }
//                     continue;
//                 }
//                 pair<int, int> next = BME->robot[i].path[BME->robot[i].pathIndex];
//                 cout << "move " << i << " ";
//                 if (next.first == BME->robot[i].nowx + 1) {
//                     cout << "3" << endl;
//                 }
//                 else if (next.first == BME->robot[i].nowx - 1) {
//                     cout << "2" << endl;
//                 }
//                 else if (next.second == BME->robot[i].nowy + 1) {
//                     cout << "0" << endl;
//                 }
//                 else if (next.second == BME->robot[i].nowy - 1) {
//                     cout << "1" << endl;
//                 }
//                 BME->robot[i].pathIndex ++;

//                 fstream out;
//                 out.open("output.txt", ios::out | ios::app);
//                 out << "move " << i << " ";
//                 if (next.first == BME->robot[i].nowx + 1) {
//                     out << "3" << endl;
//                 }
//                 else if (next.first == BME->robot[i].nowx - 1) {
//                     out << "2" << endl;
//                 }
//                 else if (next.second == BME->robot[i].nowy + 1) {
//                     out << "0" << endl;
//                 }
//                 else if (next.second == BME->robot[i].nowy - 1) {
//                     out << "1" << endl;
//                 }
//                 out.close();
//             }
//         }
//         cout << "OK\n";
//     }
// }

// int main() {
//     Controller BME2104;
//     BME2104.Init();
//     for(int i = 1; i < RobotNumber; i ++) {
//         BME2104.robot[i].ValueLimit = 100000;
//     }
//     BME2104.PreProcess();
//     justRun(&BME2104);
//     return 0;
// }