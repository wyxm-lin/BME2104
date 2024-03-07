#include "map.h"
#include "util.h"
#include <string>
#include <iostream>

using std::string;
using std::cin;
using std::cout;
using std::endl;
using std::pair;

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

void Map::ColorMap() {
    // WALL is invalid, other status is valid
    for (int i = 0; i < MapSize; i++) {
        for (int j = 0; j < MapSize; j++) {
            if (map[i][j] == WALL)
                Color[i][j] = -1;
            else 
                Color[i][j] = 0;
        }
    }
    // Color the map
    ColorCount = 0;
    for (int i = 0; i < MapSize; i++) {
        for (int j = 0; j < MapSize; j++) {
            if (Color[i][j] == 0) {
                ColorCount++;
                queue<pair<int, int>> q;
                q.push({i, j});
                Color[i][j] = ColorCount;
                while (q.size()) {
                    pair<int, int> now = q.front();
                    q.pop();
                    for (int k = 0; k < 4; k++) {
                        int x = now.first + dx[k], y = now.second + dy[k];
                        if (in(x, y) && Color[x][y] == 0 && reachable(map[now.first][now.second], map[x][y])) {
                            Color[x][y] = ColorCount;
                            q.push({x, y});
                        }
                    }
                }
            }
        }
    }
}