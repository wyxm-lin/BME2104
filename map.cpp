#include "map.h"
#include <string>
#include <iostream>

using std::string;
using std::cin;
using std::cout;
using std::endl;

void Map::Init() {
    for(int i = 0, robotid = -1; i < MapSize; i++) {
        string line;
        cin >> line;
        for(int j = 0; j < MapSize; j++) {
            switch (line[j]) {
            case '.':
                map[i][j] = EMPTY;
                break;
            case '*':
                map[i][j] = WATER;
                break;
            case '#':
                map[i][j] = WALL;
                break;
            case 'A':
                map[i][j] = EMPTY;
                ++robotid;
                robot[robotid].id = robotid;
                robot[robotid].update(i, j, false, false);
                break;
            case 'B':
                map[i][j] = PORT;
                break;
            default:
                break;
            }
        }
    }

    // map information finished

    for(int i = 0; i < PortNumber; i++) {
        int id, x, y, T, v;
        cin >> id >> x >> y >> T >> v;
        port[id].id = id;
        port[id].x = x;
        port[id].y = y;
        port[id].T = T;
        port[id].velocity = v;
    }

    // port information finished
    int capa; cin >> capa;
    for(int i = 0; i < ShipNumber; i++) {
        ship[i].id = i;
        ship[i].capacity = capa;
    }

    string OKstring;
    cin >> OKstring;
    cout << "OK" << endl;
    // End stage preprocess here
    // endl would automatically fflush the std output
}