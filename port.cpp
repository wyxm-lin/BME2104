#include "port.h"
#include "map.h"

using std::pair;

void Port::PortDisInit(Map* ctlr) {
    // initialize the distance is invalid
    for (int i = 0; i < MapSize; i++) {
        for (int j = 0; j < MapSize; j++) {
            dis[i][j] = -1;
        }
    }
    // BFS to calculate the distance
    dis[x][y] = 0;
    queue<pair<int, int>> q;
    q.push({x, y});
    while (q.size()) {
        int x_ = q.front().first, y_ = q.front().second;
        q.pop();
        for (int k = 0; k < 4; k++) {
            int nx = x_ + dx[k], ny = y_ + dy[k];
            if (in(nx, ny) && dis[nx][ny] == -1 && ctlr->Color[nx][ny] == ctlr->Color[x][y]) {
                dis[nx][ny] = dis[x_][y_] + 1;
                q.push({nx, ny});
            }
        }
    }
}

void Port::PrintDis() {
    // TODO PrintDis function hasn't been implemented
}