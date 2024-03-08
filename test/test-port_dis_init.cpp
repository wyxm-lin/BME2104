#include <iostream>
#include <fstream>
#include <string>
#include "common.h"
#include "util.h"
#include "port.h"
#include "atlas.h"

using namespace std;

string MapPath = "../maps/map1.txt";

int main() {
    cout << "hello world\n";
    // atlas init
    Atlas atlas;
    atlas.AtlasInitByMapTxt(MapPath);
    atlas.ColorAtlas();
    cout << "voer\n";
    // all port init
    Port port[PortNumber];
    int id = -1;

    for (int i = 0; i < MapSize; i ++) {
        for (int j = 0; j < MapSize; j++) {
            cout << atlas.atlas[i][j] << " ";
            if (atlas.atlas[i][j] == PORT) {
                id++;
                port[id].id = id;
                port[id].x = i;
                port[id].y = j;
            }
        }
        cout << endl;
    }
    cout << "come here\n";
    for (int i = 0; i < PortNumber; i++) {
        port[i].PortDisInit(&atlas);
    }

    // start to test
    for (int i = 0; i < PortNumber; i++) {
        for (int j = 0; j < PortNumber; j++) {
            port[i].PrintDis(port[j].x, port[j].y);
        }
    }
    

    
    return 0;
}