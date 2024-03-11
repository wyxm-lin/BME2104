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
                if (isValid(nextX, nextY) == false) {
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

void AstarNoConflict(vector<pair<int,int>>& st, vector<pair<int, int>>& ed, vector<vector<TimeNode>>& solvePath, vector<unordered_set<TimeNode>> & use) {
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

            // bool flag = true;

            for (int i = 0; i < 4; i++) {
                int nextX = x + Tdx[i], nextY = y + Tdy[i], NextTime = Time + 1;
                if (in(nextX, nextY) == false) {
                    continue;
                }
                if (isValid(nextX, nextY) == false) {
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
                else {
                    vis[nextX][nextY] = true; // add this line 
                    fa[use4Search] = now;
                    openList.push(use4Search);
                }
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