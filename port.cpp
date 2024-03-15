#include "util.h"
#include "port.h"
#include "atlas.h"

using std::queue;
using std::pair;

extern int atlas[MapSize][MapSize];
extern int color[MapSize][MapSize];

int PortDis[PortNumber][MapSize][MapSize];

void PortDisInit(int PortX, int PortY, int id) {
    // initialize the distance is invalid
    memset(PortDis[id], -1, sizeof(PortDis[id]));
    // BFS to calculate the distance
    PortDis[id][PortX][PortY] = 0;
    queue<pair<int, int>> q;
    q.push({PortX, PortY});
    while (q.size()) {
        int x_ = q.front().first, y_ = q.front().second;
        q.pop();
        for (int k = 0; k < 4; k++) {
            int nx = x_ + dx[k], ny = y_ + dy[k];
            if (valid(nx, ny) && PortDis[id][nx][ny] == -1 && color[nx][ny] == color[x_][y_]) {
                PortDis[id][nx][ny] = PortDis[id][x_][y_] + 1;
                q.push({nx, ny});
            }
        }
    }
}

int PortGetDis(int aimX, int aimY, int id) {
    return PortDis[id][aimX][aimY];
}

bool Port::arrive(int askx, int asky) {
    if(x <= askx && askx <= x + 3 && y <= asky && asky <= y + 3) {
        return true;
    }
    return false;
}

bool Port::isopen() {
    return openstatus;
}

void Port::open() {
    openstatus = true;
}

void Port::close() {
    openstatus = false;
}

void Port::pull(int value) {    // robot pull item
    nowItemCnt++;
    totalItemCnt++;
}

void Port::load(int actualLoadCnt) {   // ship load item
    nowItemCnt -= actualLoadCnt;
}
