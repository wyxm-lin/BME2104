#include <iostream>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <cstring>
#include <queue>
#include <algorithm>

using namespace std;

struct TimeNode {
    int x, y, Time;
    int g, h;

    TimeNode() = default;
    ~TimeNode() = default;
    TimeNode(int x_, int y_, int Time_, int g_, int h_): x(x_), y(y_), Time(Time_), g(g_), h(h_) {}

    bool equalExceptTime(TimeNode &b) {
        return x == b.x && y == b.y;
    }
    bool operator == (const TimeNode &b) const {
        return x == b.x && y == b.y && Time == b.Time;
    }
    bool operator != (const TimeNode &b) const {
        return x != b.x || y != b.y || Time != b.Time;
    }
    bool operator < (const TimeNode &b) const {
        return Time < b.Time;
    }
};

namespace std {
    template <>
    struct hash<TimeNode> {
        size_t operator()(const TimeNode &node) const {
            return node.x + node.y * 200 + node.Time * 40000;
        }
    };
}

const int TMapSize = 20 + 5;
char TMap[TMapSize][TMapSize];
const int Tdx[] = {-1, 1, 0, 0};
const int Tdy[] = {0, 0, -1, 1};

bool in (int x, int y) {
    return 1 <= x && x <= TMapSize && 1 <= y && y <= TMapSize;
}

bool isValid(int x, int y) {
    return TMap[x][y] != '#';
}

void InitTMap() {
    for (int i = 1; i <= 20; i++) {
        for (int j = 1; j <= 20; j++) {
            TMap[i][j] = '#';
        }
    }
    TMap[1][1] = TMap[1][2] = TMap[1][3] = TMap[1][4] = ' ';
    TMap[2][2] = TMap[2][3] = ' ';
    TMap[3][2] = TMap[3][3] = ' ';
    TMap[4][2] = TMap[4][3] = ' ';
    TMap[5][1] = TMap[5][2] = TMap[5][3] = TMap[5][4] = ' ';
}

void Astar(vector<pair<int,int>>& st, vector<pair<int, int>>& ed, vector<vector<TimeNode>>& solvePath) {
    int NowFrame = 1;
    for (int id = 0; id < st.size(); id++) {
        int startX = st[id].first, startY = st[id].second;
        int targetX = ed[id].first, targetY = ed[id].second;

        unordered_map<TimeNode, TimeNode> fa;
        priority_queue<TimeNode> openList;
        TimeNode start = TimeNode(startX, startY, 1, 0, abs(startX - targetX) + abs(startY - targetY)); // set first frame is 1
        openList.push(start);
        fa[start] = start;
        TimeNode Last;

        bool vis[TMapSize + 5][TMapSize + 5];
        memset(vis, false, sizeof(vis));
        vis[startX][startY] = true;

        while (!openList.empty()) {
            TimeNode now = openList.top();
            openList.pop();
            int x = now.x, y = now.y, Time = now.Time;
            if (x == targetX && y == targetY) {
                Last = now;
                break;
            }

            for (int i = 0; i < 4; i++) {
                int nextX = x + Tdx[i], nextY = y + Tdy[i], NextTime = Time + 1;
                if (in(nextX, nextY) == false) {
                    continue;
                }
                if (isValid(x, y) == false) {
                    continue;
                }
                if (vis[nextX][nextY]) { // has visited in space dimension
                    continue;
                }  
                TimeNode use4Search = {nextX, nextY, NextTime, now.g + 1, abs(nextX - targetX) + abs(nextY - targetY) + abs(NextTime - NowFrame)};
                vis[nextX][nextY] = true; // add this line 
                fa[use4Search] = now;
                openList.push(use4Search);
            }
        }

        vector<TimeNode> pathWithTime; // pathWithTime is (start, targe]
        while (Last != start) {
            pathWithTime.push_back(Last);
            Last = fa[Last];
        }
        reverse(pathWithTime.begin(), pathWithTime.end());
        solvePath[id] = pathWithTime;        
    }
}

// can't solve the vertex conflict
void AstarWithConflict(vector<pair<int,int>>& st, vector<pair<int, int>>& ed, vector<vector<TimeNode>>& solvePath, vector<unordered_set<TimeNode>> & use) {
    int NowFrame = 1;
    for (int id = 0; id < st.size(); id++) {
        int startX = st[id].first, startY = st[id].second;
        int targetX = ed[id].first, targetY = ed[id].second;

        unordered_map<TimeNode, TimeNode> fa;
        priority_queue<TimeNode> openList;
        TimeNode start = TimeNode(startX, startY, 1, 0, abs(startX - targetX) + abs(startY - targetY)); // set first frame is 1
        openList.push(start);
        fa[start] = start;
        TimeNode Last;

        bool vis[TMapSize + 5][TMapSize + 5];
        memset(vis, false, sizeof(vis));
        vis[startX][startY] = true;

        while (!openList.empty()) {
            TimeNode now = openList.top();
            openList.pop();
            Last = now;
            int x = now.x, y = now.y, Time = now.Time;
            if (x == targetX && y == targetY) {
                // Last = now;
                break;
            }

            for (int i = 0; i < 4; i++) {
                int nextX = x + Tdx[i], nextY = y + Tdy[i], NextTime = Time + 1;
                if (in(nextX, nextY) == false) {
                    continue;
                }
                if (isValid(x, y) == false) {
                    continue;
                }
                if (vis[nextX][nextY]) { // has visited in space dimension
                    continue;
                }  
                TimeNode use4Search = {nextX, nextY, NextTime, now.g + 1, abs(nextX - targetX) + abs(nextY - targetY) + abs(NextTime - NowFrame)};
                TimeNode NowPosNextTime = {x, y, NextTime, 0, 0};
                TimeNode NextPosNowTime = {nextX, nextY, Time, 0, 0};

                bool flag = true;
                for (int pre = 0; pre < id; pre ++ ) { // exist vertex conflict or edge conflict
                    unordered_set<TimeNode> used = use[pre];
                    if (used.find(use4Search) != used.end() ||
                        used.find(NowPosNextTime) != used.end() && used.find(NextPosNowTime) != used.end()) { 
                        flag = false;
                        break;
                    }
                }
                if (flag == false) {
                    continue;
                }
                vis[nextX][nextY] = true; // add this line 
                fa[use4Search] = now;
                openList.push(use4Search);
            }
        }

        if (Last.x == targetX && Last.y == targetY) {
            vector<TimeNode> pathWithTime; // pathWithTime is (start, targe]
            while (Last != start) {
                pathWithTime.push_back(Last);
                Last = fa[Last];
            }
            reverse(pathWithTime.begin(), pathWithTime.end());
            solvePath[id] = pathWithTime;        
            use[id].insert(pathWithTime.begin(), pathWithTime.end());  
        }
        else {
            solvePath[id] = vector<TimeNode>();
        }
    }
}

void printPath(vector<vector<TimeNode>>& solvePath) {
    for (int i = 0; i < solvePath.size(); i++) {
        for (int j = 0; j < solvePath[i].size(); j++) {
            cout << solvePath[i][j].x << " " << solvePath[i][j].y << " " << solvePath[i][j].Time << endl;
        }
        cout << endl;
    }
    cout << endl << endl;
}

int main() {
    InitTMap();
    vector<pair<int, int>> st, ed;
    st.push_back({1, 1});
    ed.push_back({5, 4});
    st.push_back({5, 1});
    ed.push_back({1, 1});

    vector<vector<TimeNode>> solvePath(st.size());
    Astar(st, ed, solvePath);
    vector<vector<TimeNode>> solvePath2(st.size());
    vector<unordered_set<TimeNode>> use(st.size());
    AstarWithConflict(st, ed, solvePath2, use);

    printPath(solvePath);
    printPath(solvePath2);

    return 0;
}

// #include <iostream>
// #include <vector>
// #include <unordered_set>

// // 少转弯

// using namespace std;

// char map[55][55];
// int T_MapSize = 20;

// struct TimeNode {
//     int x, y, Time;

//     bool equalExceptTime(TimeNode &b) {
//         return x == b.x && y == b.y;
//     }
//     bool operator == (const TimeNode &b) const {
//         return x == b.x && y == b.y && Time == b.Time;
//     }
// };

// namespace std {
//     template <>
//     struct hash<TimeNode> {
//         size_t operator()(const TimeNode &node) const {
//             return node.x + node.y * 200 + node.Time * 40000;
//         }
//     };
// }

// vector<TimeNode> path1, path2;
// unordered_set<TimeNode> env;

// enum Direction {
//     UP = 0,
//     DOWN = 1,
//     LEFT = 2,
//     RIGHT = 3
// };

// const int dx[] = {-1, 1, 0, 0};
// const int dy[] = {0, 0, -1, 1};

// Direction relative(int x, int y, int nextX, int nextY) {
//     if (nextX == x + 1) {
//         return DOWN;
//     }
//     else if (nextX == x - 1) {
//         return UP;
//     }
//     else if (nextY == y + 1) {
//         return RIGHT;
//     }
//     else if (nextY == y - 1) {
//         return LEFT;
//     }
//     else {
//         return UP;
//     }
// }

// bool inEnv(int x, int y, int Time) {
//     return 1 <= x && x <= T_MapSize && 1 <= y && y <= T_MapSize;
// }

// void initMap() {
//     for (int i = 1; i <= T_MapSize; i++) {
//         for (int j = 1; j <= T_MapSize; j++) {
//             map[i][j] = '#';
//         }
//     }
// }

// void printMap() {
//     for (int i = 1; i <= T_MapSize; i++) {
//         for (int j = 1; j <= T_MapSize; j++) {
//             cout << map[i][j];
//         }
//         cout << endl;
//     }
//     cout << endl << endl;
// }

// void addEmpty() {
//     int Step = 4;
//     for (int i = 1; i <= T_MapSize; i += Step) {
//         for (int j = 1; j <= T_MapSize; j++) {
//             map[i][j] = ' ';
//         }
//     }
//     for (int j = 1; j <= T_MapSize; j += Step) {
//         for (int i = 1; i <= T_MapSize; i ++) {
//             map[i][j] = ' ';
//         }
//     }
// }

// // 看似是点冲突 其实就是边冲突 (4,4)中存在点冲突
// void initPath12() {
//     initMap();
    
//     path1.push_back({2, 1, 1});
//     path1.push_back({2, 2, 2});
//     path1.push_back({2, 3, 3});
//     path1.push_back({2, 4, 4});
//     path1.push_back({3, 4, 5});
//     path1.push_back({4, 4, 6});
//     path1.push_back({5, 4, 7});
//     path1.push_back({6, 4, 8});
//     path1.push_back({7, 4, 9});
//     path1.push_back({8, 3, 10});
//     path1.push_back({8, 2, 11});
//     path1.push_back({8, 1, 12});


//     path2.push_back({7, 6, 1});
//     path2.push_back({7, 5, 2});
//     path2.push_back({7, 4, 3});
//     path2.push_back({6, 4, 4});
//     path2.push_back({5, 4, 5});
//     path2.push_back({4, 4, 6});
//     path2.push_back({3, 4, 7});
//     path2.push_back({2, 4, 8});
//     path2.push_back({2, 5, 9});
//     path2.push_back({2, 6, 10});

//     // 设置该位置的该时刻被占用
//     for (int i = 0; i < path1.size(); i++) {
//         env.insert(path1[i]);
//     }
//     for (int i = 0; i < path2.size(); i++) {
//         env.insert(path2[i]);
//     }

//     map[2][1] = '1';
//     map[2][2] = '2';
//     map[2][3] = '3';
//     map[2][4] = '4';
//     map[3][4] = '5';
//     map[4][4] = '6';
//     map[5][4] = '7';
//     map[6][4] = '8';
//     map[7][4] = '9';
//     map[8][3] = 'a';
//     map[8][2] = 'b';
//     map[8][1] = 'c';

//     map[7][6] = '1';
//     map[7][5] = '2';
//     map[7][4] = '3';
//     map[6][4] = '4';
//     map[5][4] = '5';
//     map[4][4] = '6';
//     map[3][4] = '7';    
//     map[2][4] = '8';
//     map[2][5] = '9';
//     map[2][6] = 'a';

//     map[4][3] = map[4][5] = ' ';
//     map[3][3] = ' ';
// }

// bool isWall(int x, int y) {
//     return map[x][y] == '#';
// }

// void avoid(vector<TimeNode> &path1, vector<TimeNode> &path2) {
    


//     for (int i = 0; i < min(path1.size(), path2.size()); i ++) {
//         auto node1 = path1[i];
//         auto node2 = path2[i];
//         // 发生点冲突的位置
//         if (node1.Time == node2.Time) {
//             int TimeSum = node1.Time + node2.Time;
//             int Path1Befor = i, Path2After = i; // 1给2让路
//             int Path2Before = i, Path1After = i; // 2给1让路
//             // 之后看下下面这俩个部分能不能合并
//             while (true) {
//                 Path1Befor --;
//                 Path2After ++;
//                 if (Path1Befor < 0 || Path2After >= min(path1.size(), path2.size())) {
//                     break;
//                 }
//                 if (path1[Path1Befor].equalExceptTime(path2[Path2After])) {
//                 }
//                 else {
//                     break;
//                 }
//             }
//             while (true) {
//                 Path2Before --;
//                 Path1After ++;
//                 if (Path2Before < 0 || Path1After >= min(path1.size(), path2.size())) {
//                     break;
//                 }
//                 if (path1[Path1After].equalExceptTime(path2[Path2Before])) {
//                 }
//                 else {
//                     break;
//                 }
//             }

//             // 如果是让机器人1去让路
//             // 那么就从i向Path1Before走，直到找到可以结束的位置
//             int idx = i - 1;
//             while (idx != Path1Befor) {
//                 // 判断这个位置附近有没有空闲的位置
//                 TimeNode& node_before = path1[idx];
//                 // 找一下这个位置的除了下一个位置和上一个位置之外还有什么位置，并且再去判断一下这些位置是否可以走，如果可以走， 就结束。
//                 int dir1 = relative(node_before.x, node_before.y, path1[idx + 1].x, path1[idx + 1].y);
//                 int dir2 = -1;
//                 if (idx - 1 >= 0) {
//                     dir2 = relative(node_before.x, node_before.y, path1[idx - 1].x, path1[idx - 1].y);
//                 }
//                 bool flag = false;
//                 for (int i = 0; i < 4; i++) {
//                     if (i != dir1 && i != dir2) {
//                         int nextX = node_before.x + dx[i], nextY = node_before.y + dy[i], nextTime = node_before.Time + 1;
//                         if (inEnv(nextX, nextY, nextTime) && 
//                             isWall(nextX, nextY == false) && 
//                             env.find({nextX, nextY, nextTime}) == env.end()) { // 这个位置在这个时间点没有被占用
//                             // 找到了一个可以走的位置
//                             // TODO 将该新的位置插入到path1中
                            
//                             flag = true;
//                             break;
//                         }
//                     }
//                 }
//                 if (flag )
//                     break;
//                 idx --;
//             }
//         }
//         else; // TODO 还存在else if

//     }
    
// }

// int main() {
//     initMap();
//     printMap();
//     addEmpty();
//     printMap();

//     return 0;
// }