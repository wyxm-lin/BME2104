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

    Robot robot;
    robot.IsWorking = true;
    robot.nowx = 50;
    robot.nowy = 20;
    robot.targetX = 50;
    robot.targetY = 80;
    
    astar(robot, atlas);

    robot.RobotPrintPath();

    return 0;
}