#include <iostream>
#include <vector>
#include <queue>
#include <unordered_set>
#include <unordered_map>
#include <utility>
#include <cmath>
#include <functional>
#include <algorithm>

using namespace std;

const int dx[] = {0, 0, 1, -1};
const int dy[] = {1, -1, 0, 0};

// 定义状态结构体
struct StateTest {
    int x, y, time;
    StateTest(int x_, int y_, int time_) : x(x_), y(y_), time(time_) {}
    bool operator==(const StateTest& s) const {
        return x == s.x && y == s.y;
    }
    bool operator!=(const StateTest& s) const {
        return x != s.x || y != s.y;
    }
    bool operator<(const StateTest& s) const {
        if (time != s.time)
            return time < s.time;
        else if (x != s.x)
            return x < s.x;
        else
            return y < s.y;
    }
    StateTest() = default;
};

namespace std {
    template <>
    struct hash<StateTest> {
        std::size_t operator()(const StateTest& s) const {
            std::size_t xHash = std::hash<int>{}(s.x);
            std::size_t yHash = std::hash<int>{}(s.y);
            std::size_t timeHash = std::hash<int>{}(s.time);
            // 使用异或操作合并哈希值
            return xHash ^ (yHash << 1) ^ (timeHash << 2);
        }
    };
}

// 定义点约束结构体
struct PointConstraint {
    int x, y, time;
    PointConstraint(int x, int y, int time) : x(x), y(y), time(time) {}
};

// 定义边约束结构体
struct EdgeConstraint {
    int startX, startY, endX, endY, startTime, endTime;
    EdgeConstraint(int startX, int startY, int endX, int endY, int startTime, int endTime)
        : startX(startX), startY(startY), endX(endX), endY(endY), startTime(startTime), endTime(endTime) {}
};

// 计算两个状态之间的欧几里得距离
double euclideanDistance(const StateTest& s1, const StateTest& s2) {
    return std::sqrt((s1.x - s2.x) * (s1.x - s2.x) + (s1.y - s2.y) * (s1.y - s2.y));
}

// 检查状态是否合法
bool isValidStateTest(const StateTest& StateTest, const std::vector<PointConstraint>& pointConstraints,
                  const std::vector<EdgeConstraint>& edgeConstraints) {
    // 检查点约束
    for (const auto& constraint : pointConstraints) {
        if (StateTest.x == constraint.x && StateTest.y == constraint.y && StateTest.time == constraint.time)
            return false;
    }

    // 检查边约束
    for (const auto& constraint : edgeConstraints) {
        if (StateTest.x >= constraint.startX && StateTest.x <= constraint.endX &&
            StateTest.y >= constraint.startY && StateTest.y <= constraint.endY &&
            StateTest.time >= constraint.startTime && StateTest.time <= constraint.endTime)
            return false;
    }

    return true;
}

// A*搜索算法
std::vector<StateTest> AStar(const StateTest& start, const StateTest& goal, const std::vector<PointConstraint>& pointConstraints,
                         const std::vector<EdgeConstraint>& edgeConstraints) {
    // 定义优先队列和已访问集合
    std::priority_queue<std::pair<double, StateTest>, std::vector<std::pair<double, StateTest>>, std::greater<>> openSet;
    std::unordered_set<StateTest> visited;

    // 将起始状态加入优先队列和已访问集合
    openSet.push({0, start});
    visited.insert(start);

    // 定义状态到父状态的映射，用于重构路径
    std::unordered_map<StateTest, StateTest> cameFrom;

    // 开始搜索
    while (!openSet.empty()) {
        // 从优先队列中取出当前状态
        StateTest current = openSet.top().second;
        openSet.pop();
        std::cout << "ing\n";
        // 如果当前状态是目标状态，重构并返回路径
        if (current == goal) {
            std::vector<StateTest> path;
            while (current != start) {
                path.push_back(current);
                current = cameFrom[current];
            }
            path.push_back(start);
            std::reverse(path.begin(), path.end());
            return path;
        }

        // 对当前状态的邻居进行搜索
        for (int i = 0; i < 4; i ++) {
            int newX = current.x + dx[i];
            int newY = current.y + dy[i];
            int newTime = current.time + 1;

            // 构造邻居状态
            StateTest neighbor(newX, newY, newTime);

            // 检查邻居状态是否合法
            if (isValidStateTest(neighbor, pointConstraints, edgeConstraints) &&
                visited.find(neighbor) == visited.end()) {
                // 计算邻居状态的代价并加入优先队列
                double gScore = euclideanDistance(current, neighbor);
                double fScore = gScore + euclideanDistance(neighbor, goal);
                openSet.push({fScore, neighbor});
                visited.insert(neighbor);
                cameFrom[neighbor] = current;
            }
        }
    }

    // 如果搜索失败，返回空路径
    vector<StateTest> ret;
    return ret;
}

int main() {
    // 定义起点和终点
    StateTest start(0, 0, 0);
    StateTest goal(10, 10, 10);

    // 定义点约束和边约束
    std::vector<PointConstraint> pointConstraints = {PointConstraint(5, 5, 3)};
    std::vector<EdgeConstraint> edgeConstraints = {EdgeConstraint(2, 2, 3, 3, 0, 5)};

    std::cout << "start" << std::endl;

    // 使用A*算法找到合法路径
    std::vector<StateTest> path = AStar(start, goal, pointConstraints, edgeConstraints);

    // 输出路径
    std::cout << "Path:";
    for (const auto& StateTest : path) {
        std::cout << " (" << StateTest.x << ", " << StateTest.y << ", " << StateTest.time << ")";
    }
    std::cout << std::endl;

    return 0;
}
