/*

    How to run this file:
    Step1: build
    Step2: in the shell (directory: /BME2104/), run the following command:
        ./bin/test-astar

*/

#include "robot.h"
#include "atlas.h"
#include "common.h"
#include "util.h"
#include "searchPath.h"
#include <fstream>

using namespace std;

string MapPath = "../maps/map1.txt";

int main() {
    Atlas atlas;
    atlas.AtlasInitByMapTxt(MapPath);
    atlas.ColorAtlas();

    atlas.AtlasPrintMap();

    Robot robot[RobotNumber];
    robot[0].IsWorking = true;
    robot[0].nowx = 50;
    robot[0].nowy = 20;
    robot[0].targetX = 50;
    robot[0].targetY = 80;
    robot[0].IsAvailable = true;
    robot[0].IsCarry = false;
    robot[0].ValueLimit = 0;
    robot[0].IsWorking = true;
    
    AstarTest(robot, atlas, 1.0, 1);

    // AstarConsiderTime(robot, atlas, 1.0, 1);

    robot[0].RobotPrintPath();

    return 0;
}