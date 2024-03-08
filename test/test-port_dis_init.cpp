/*

    How to run this file:
    Step1: build
    Step2: in the shell (directory: /BME2104/), run the following command:
        ./bin/test-port_dis_init

*/

#include <iostream>
#include <fstream>
#include <string>
#include <cstring>
#include "common.h"
#include "util.h"
#include "port.h"
#include "atlas.h"

using namespace std;

string MapPath = "../maps/map1.txt";

int main() {
    /*************atlas init*****************/
    Atlas atlas;
    atlas.AtlasInitByMapTxt(MapPath);
    // atlas.AtlasPrintMap();
    atlas.ColorAtlas();
    atlas.AtlasPrintColor(0, 0, MapSize - 1, MapSize - 1);

    /*************port init*****************/    
    Port port[PortNumber];
    int id = -1;

    bool vis[MapSize][MapSize];
    memset(vis, false, sizeof vis);

    for (int i = 0; i < MapSize; i ++) {
        for (int j = 0; j < MapSize; j++) {
            if (atlas.atlas[i][j] == PORT && vis[i][j] == false) {
                for (int k = 0; k < 4; k++) {
                    for (int l = 0; l < 4; l++) {
                        vis[i + k][j + l] = true;
                    }
                }
                id++;
                port[id].id = id;
                port[id].x = i;
                port[id].y = j;
            }
        }
    }

    for (int i = 0; i < PortNumber; i++) {
        port[i].PortDisInit(&atlas);
    }

    /*************start to test*****************/    
    for (int i = 0; i < PortNumber; i++) {
        for (int j = 0; j < PortNumber; j++) {
            port[i].PrintDis(port[j].x, port[j].y);
        }
    }
    
    return 0;
}