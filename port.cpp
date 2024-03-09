#include "util.h"
#include "port.h"
#include "atlas.h"

using std::queue;
using std::pair;

void Port::PortDisInit(Atlas* atlas) {
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
            if (valid(nx, ny) && dis[nx][ny] == -1 && atlas->color[nx][ny] == atlas->color[x_][y_]) {
                dis[nx][ny] = dis[x_][y_] + 1;
                q.push({nx, ny});
            }
        }
    }
}

int Port::GetDis(int aimX, int aimY) {
    return dis[aimX][aimY];
}

bool Port::arrive(int askx, int asky) {
    if(x <= askx && askx <= x + 3 && y <= asky && asky <= y + 3) {
        return true;
    }
    return false;
}

/************Below variables and functions are for debug***************/
void Port::PrintDis(int x_, int y_) {
    printf("The distance from Port(%d, %d) to (%d, %d) is %d\n", x, y, x_, y_, dis[x_][y_]);
}