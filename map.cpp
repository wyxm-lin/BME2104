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
                robot[robotid].update(i, j, false, true);
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

void Map::RunByFrame() {
    int frameID = 0, nowamoney = 0;
    while(frameID < FrameLimit) {
        cin >> frameID >> nowamoney;
        int NewItemCount;
        cin >> NewItemCount;
        for(int i = 1; i <= NewItemCount; i++) {
            int x, y, val;
            cin >> x >> y >> val;
            ItemList.emplace(Item(frameID, x, y, val));
            ItemValue[x][y] = val;
        }
        // Finish new item input
        ItemTimeOutDisappear(frameID);
        // Kick out disappeared items

        for(int i = 0; i < RobotNumber; i++) {
            int carry, x, y, status;
            cin >> carry >> x >> y >> status;
            robot[i].update(x, y, carry, status);
        }
        // Robots Data Update

        for(int i = 0; i < ShipNumber; i++) {
            int status, tarid;
            cin >> status >> tarid;
            ship[i].update((ShipStatus)status, tarid);
            // the status is accordingly assigned to integers, in 'common.h'
        }
        // Ship Data Update
        string OKstring;
        cin >> OKstring;
        // Read 'OK'
    }
}

void Map::ItemTimeOutDisappear(int frameID) {
    while(ItemList.size()) {
        Item it = ItemList.front();
        if(it.BirthFrame + ExistFrame <= frameID) {
            ItemValue[it.x][it.y] = 0;
            ItemList.pop();
        }
        else {
            break;
        }
    }
}