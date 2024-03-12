#include <iostream>
#include <algorithm>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <bitset>
#include <cmath>
#include <fstream>
#include <queue>
#include <ctime>
#include <chrono>
#include <cstring>

using namespace std;

const char MapPath[] = "map1.txt";

const int TotalFrame = 15000;
const int MapSize = 200;
bitset<TotalFrame> occupied[MapSize][MapSize];
char grid[MapSize][MapSize];

struct TimeNode {
    int x, y, Time;
    double g, h;
    double cost;

    TimeNode() = default;
    ~TimeNode() = default;
    TimeNode(int x_, int y_, int Time_): x(x_), y(y_), Time(Time_) {}
    TimeNode(int x_, int y_, int Time_, double g_, double h_): x(x_), y(y_), Time(Time_), g(g_), h(h_) {}

    bool operator == (const TimeNode& other) const {
        return x == other.x && y == other.y && Time == other.Time;
    }

    bool operator != (const TimeNode& other) const {
        return x != other.x || y != other.y || Time != other.Time;
    }

    bool operator < (const TimeNode& other) const {
        return g + h > other.g + other.h;
    }

    double heuristic(int targetX, int targetY, int originX, int originY, int originTime) {
        // return abs(x - targetX) + abs(y - targetY) - (Time - originTime); // (195 189 14995) Execution time: 9589.71 milliseconds    
        return abs(x - targetX) + abs(y - targetY); // (195 189 376) Execution time: 235.505 milliseconds
    }
};

namespace std {
    template <>
    struct hash<TimeNode> {
        size_t operator()(const TimeNode &node) const {
            return node.x + node.y * 200 + node.Time * 40000; // NOTE: efficient hash
            // return hash<int>()(node.x) ^ hash<int>()(node.y) ^ hash<int>()(node.Time);
        }
    };
}

void InitByTxt() {
    ifstream fin;
    fin.open(MapPath);
    for (int i = 0; i < MapSize; i++) {
        for (int j = 0; j < MapSize; j++) {
            fin >> grid[i][j];
        }
    }
    fin.close();
}

bool valid(int x, int y) {
    return 0 < x && x < MapSize && 0 < y && y < MapSize;
}

bool reachable(int x, int y, int xx, int yy) {
    if (grid[x][y] == '#' || grid[xx][yy] == '#')
        return false;
    else if (grid[x][y] == '*' && grid[xx][yy] == '*') // NOTE
        return true;
    else if (grid[x][y] == grid[xx][yy])
        return true;
    else if (grid[x][y] == 'B' && grid[xx][yy] == '.')
        return true;
    else if (grid[x][y] == '.' && grid[xx][yy] == 'B')
        return true;
    else
        return false;
}

const int dx[] = {0, 0, 0, -1, 1};
const int dy[] = {0, -1, 1, 0, 0};

vector<TimeNode> search(int startX, int startY, int targetX, int targetY) {
    priority_queue<TimeNode> openSet;
    unordered_map<TimeNode, TimeNode> fa;

    int NowFrame = 0;
    int FirstArriveTime[MapSize][MapSize];
    memset(FirstArriveTime, 0, sizeof(FirstArriveTime));

    TimeNode startNode = TimeNode(startX, startY, NowFrame);
    startNode.g = 0;
    startNode.h = startNode.heuristic(targetX, targetY, startX, startY, NowFrame);
    openSet.push(startNode);
    TimeNode Last;

    while (!openSet.empty()) {
        TimeNode current = openSet.top();
        openSet.pop();
        for (int i = 0; i < 5; i++) {
            int NextX = current.x + dx[i];
            int NextY = current.y + dy[i];
            int NextTime = current.Time + 1;
            if (current.x == 194 && current.y == 189) {
                cout << NextX << " " << NextY << " " << NextTime << endl;
            }
            if (valid(NextX, NextY) && reachable(current.x, current.y, NextX, NextY) && NextTime <= TotalFrame) {
                if (occupied[NextX][NextY][NextTime])
                    continue;
                TimeNode NextNode(NextX, NextY, NextTime);
                fa[NextNode] = current;
                if (NextX == targetX && NextY == targetY) {
                    vector<TimeNode> path;
                    while (NextNode != startNode) {
                        path.push_back(NextNode);
                        NextNode = fa[NextNode];
                    }
                    reverse(path.begin(), path.end());
                    return path;
                    break;
                }
                occupied[NextX][NextY][NextTime] = true;
                if (FirstArriveTime[NextX][NextY] == 0) {
                    FirstArriveTime[NextX][NextY] = NextTime;
                    NextNode.g = current.g + 1;
                    NextNode.h = NextNode.heuristic(targetX, targetY, startX, startY, NowFrame);

                }
                else {
                    NextNode.g = NextTime - FirstArriveTime[NextX][NextY]; // Execution time: 23.611 milliseconds
                    NextNode.h = NextNode.heuristic(targetX, targetY, startX, startY, NowFrame);
                }
                openSet.push(NextNode);
                
            }
        }
    }
    return vector<TimeNode>();
}

vector<TimeNode> Astarsearch(int startX, int startY, int targetX, int targetY) {
    priority_queue<TimeNode> openSet;
    unordered_map<TimeNode, TimeNode> fa;

    int NowFrame = 0;
    int FirstArriveTime[MapSize][MapSize];
    memset(FirstArriveTime, 0, sizeof(FirstArriveTime));

    TimeNode startNode = TimeNode(startX, startY, NowFrame);
    startNode.g = 0;
    startNode.h = startNode.heuristic(targetX, targetY, startX, startY, NowFrame);
    openSet.push(startNode);
    TimeNode Last;

    while (!openSet.empty()) {
        TimeNode current = openSet.top();
        openSet.pop();
        for (int i = 0; i < 5; i++) {
            int NextX = current.x + dx[i];
            int NextY = current.y + dy[i];
            int NextTime = current.Time + 1;
            if (valid(NextX, NextY) && reachable(current.x, current.y, NextX, NextY) && NextTime <= TotalFrame) {
                if (FirstArriveTime[NextX][NextY] != 0)
                    continue;
                FirstArriveTime[NextX][NextY] = NextTime;
                TimeNode NextNode(NextX, NextY, NextTime);
                fa[NextNode] = current;
                if (NextX == targetX && NextY == targetY) {
                    vector<TimeNode> path;
                    while (NextNode != startNode) {
                        path.push_back(NextNode);
                        NextNode = fa[NextNode];
                    }
                    reverse(path.begin(), path.end());
                    return path;
                    break;
                }
                occupied[NextX][NextY][NextTime] = true;
                NextNode.g = current.g + 1;
                NextNode.h = NextNode.heuristic(targetX, targetY, startX, startY, NowFrame);
                openSet.push(NextNode);
                
            }
        }
    }
    return vector<TimeNode>();
}


int main() {
    InitByTxt();
    pair<int, int> st = {4, 4};
    pair<int, int> ed = {196, 189};
    auto start = std::chrono::high_resolution_clock::now();
    vector<TimeNode> path = search(st.first, st.second, ed.first, ed.second); // 完全三维的A*搜索
    // vector<TimeNode> path = Astarsearch(st.first, st.second, ed.first, ed.second); // 完全二维的A*搜索
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start);
    double milliseconds = duration.count() * 1.0e-6;
    for (auto &node: path) {
        cout << node.x << " " << node.y << " " << node.Time << endl;
    }
    std::cout << "Execution time: " << milliseconds << " milliseconds" << std::endl;
    return 0;
}