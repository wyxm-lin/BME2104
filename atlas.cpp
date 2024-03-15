#include "util.h"
#include "atlas.h"

using std::queue;
using std::pair;

MapStatus atlas[MapSize][MapSize];
int color[MapSize][MapSize];
int ColorCount;
int degree[MapSize][MapSize];

void ColorAtlas() {
    // WALL is invalid, other is valid
    for (int i = 0; i < MapSize; i++) {
        for (int j = 0; j < MapSize; j++) {
            if (atlas[i][j] == WALL)
                color[i][j] = -1;
            else 
                color[i][j] = 0;
        }
    }
    // Color the altas
    ColorCount = 0;
    for (int i = 0; i < MapSize; i++) {
        for (int j = 0; j < MapSize; j++) {
            if (color[i][j] == 0) {
                ColorCount++;
                queue<pair<int, int> > q;
                q.push({i, j});
                color[i][j] = ColorCount;
                while (q.size()) {
                    pair<int, int> now = q.front();
                    q.pop();
                    for (int k = 0; k < 4; k++) {
                        int x = now.first + dx[k], y = now.second + dy[k];
                        if (valid(x, y) && color[x][y] == 0 && reachable(atlas[now.first][now.second], atlas[x][y])) {
                            color[x][y] = ColorCount;
                            q.push({x, y});
                        }
                    }
                }
            }
        }
    }
}

void DegreeInit() {
    for (int i = 0; i < MapSize; i++) {
        for (int j = 0; j < MapSize; j++) {
            degree[i][j] = 0;
            if (atlas[i][j] == WALL || atlas[i][j] == WATER) {
                continue;
            }
            for (int k = 0; k < 4; k++) {
                int x = i + dx[k], y = j + dy[k];
                if (valid(x, y) && atlas[x][y] != WALL && atlas[x][y] != WATER) {
                    degree[i][j]++;
                }
            }
        }
    }
}
