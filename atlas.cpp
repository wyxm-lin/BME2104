#include "util.h"
#include "atlas.h"

using std::queue;
using std::pair;

void Atlas::ColorAtlas() {
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
                        if (in(x, y) && color[x][y] == 0 && reachable(atlas[now.first][now.second], atlas[x][y])) {
                            color[x][y] = ColorCount;
                            q.push({x, y});
                        }
                    }
                }
            }
        }
    }
}

void Atlas::AtlasInitByMapTxt(string path) {
    using std::ifstream;
    ifstream fin(path);
    for (int i = 0; i < MapSize; i++) {
        string line;
        fin >> line;
        for (int j = 0; j < MapSize; j++) {
            switch (line[j]) {
            case '.':
                atlas[i][j] = EMPTY;
                break;
            case '*':
                atlas[i][j] = WATER;
                break;
            case '#':
                atlas[i][j] = WALL;
                break;
            case 'A':
                atlas[i][j] = EMPTY;
                break;
            case 'B':
                atlas[i][j] = PORT;
                break;
            default:
                break;
            }
        }
    }
    fin.close();
}

void Atlas::AtlasPrintColor(int top, int left, int bottom, int right) {
    for (int i = top; i <= bottom; i++) {
        for (int j = left; j <= right; j++) {
            if (color[i][j] == -1) {
                printf("W   ");
            } else {
                printf("%-4d", color[i][j]);
            }
        }
        printf("\n");
    }
}

void Atlas::AtlasPrintMap() {
    for (int i = 0; i < MapSize; i++) {
        for (int j = 0; j < MapSize; j++) {
            switch (atlas[i][j]) {
            case EMPTY:
                printf(". ");
                break;
            case WATER:
                printf("* ");
                break;
            case WALL:
                printf("# ");
                break;
            case PORT:
                printf("A ");
                break;
            }
        }
        printf("\n");
    }
}