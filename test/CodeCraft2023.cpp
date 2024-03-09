#include <iostream>
#include <cstdio>
#include <algorithm>
#include <string>
#include <cstring>
#include <vector>
#include <cmath>
#include <unordered_map>
#include <time.h>
#include <queue>
#include <thread>
#define pii pair<int, int>
#define pace 4 // 步长为1/pace
using namespace std;
const double eps = 1e-6;
const double pi = acos(-1);
const int N = 50 * pace + 1; // 坐标化后每一维的长度
int frameID;
struct rob
{
    int target = -1; // 机器人正在前往的工作台编号，其中-1表示没有正在前往的工作台，[0,k - 1]表示正在前往的工作台编号
    int nexttarget = -1;
    int tableid, kind = 0; // 所处的工作台类型，手头的物品类型
    double time, hit;      // 时间价值系数，碰撞价值系数
    double radv;           // 角速度
    double vx, vy, v;      // 线速度
    double onto;           // 朝向
    double x, y;           // 坐标
    int pause;             // 暂停
    int shuntarget = -1;
    int shunnexttarget = -1;
    int avoid = 0;
    double tardeg = 0;
} robot[4];
struct tab
{
    int mark = -1;         // 用于决策函数，标记该工作台是否快要生产出对应7号工作台的产品
    int kind, time;        // 类型，剩余时间
    int material, product; // 材料状态，产品状态
    double x, y;           // 坐标
} table[50];
int k, mapid;
int num[10];                                     // num[i]表示可取的物品i的数量,如果有机器正在前往某个物品，这个物品是被视为不可取的。
vector<int> product_tab[10];                     // product_tab[i]存放生产产品为i的工作台的编号。在readmap中初始化。
vector<int> materialTabList[8];                  // 表示materialTabList[i]以i为原料，需要在readmap时初始化,只记录4～7
vector<vector<int>> block(N, vector<int>(N, 0)); // 障碍物
int row[8] = {0, 0, 1, -1, 1, 1, -1, -1}, col[8] = {1, -1, 0, 0, 1, -1, 1, -1};
// int row[4]={0,0,1,-1},col[4]={1,-1,0,0};
FILE *fp;
////mp[i]表示kind为i的工作台需要的原料,用于searchTableToSellNum
vector<vector<int>> mp = {{}, {}, {}, {}, {1, 2}, {1, 3}, {2, 3}, {4, 5, 6}};
vector<vector<int>> father = {{}, {4, 5, 9}, {4, 6, 9}, {5, 6, 9}, {7, 9}, {7, 9}, {7, 9}, {8, 9}};
vector<int> profit = {0, 3000, 3200, 3400, 7100, 7800, 8300, 29000, 0, 0};
vector<int> buyprice = {0, 3000, 4400, 5800, 15400, 17200, 19200, 76000};
vector<int> sellprice = {0, 6000, 7600, 9200, 22500, 25000, 27500, 105000};
bool st[50][10]; // st[i][j]为true表示第i号工作台的第j个物品有机器人去买或去卖
int check_block(int, int, int, int);
int close_block(int, int);
int Close_block(int, int);
double dis[2][50][N][N]; // 按小/大机器人算时table到点i,j的距离
int vis[2][50][N][N];    // 临时用
int sellable[50];        // 某个工作台是否能卖
bool easy7 = true;
void log(string s, double x = 0)
{
    fprintf(fp, "%s%lf\n", s.c_str(), x);
    return;
}
// 计算工作台p 和 工作台 q 之间的距离,op==0表示按小机器人算距离，op==1表示按大机器人
// 从p出发，目的地为q
double tab_dis(int p, int q, int op = 1)
{
    int x = (int)(table[q].y * pace), y = (int)(table[q].x * pace);
    int temp = 0;
    if (block[x][y + 1])
        temp++;
    if (block[x][y - 1])
        temp++;
    if (block[x + 1][y])
        temp++;
    if (block[x - 1][y])
        temp++;
    if (temp >= 2 && op == 1)
        return 1e9;
    double ans = 1e9;
    for (int i = -1; i <= 1; i++)
        for (int j = -1; j <= 1; j++)
            ans = min(ans, dis[op][p][x + i][y + j]);
    return ans;
}
// 机器p到工作台q
double rob_tab(int p, int q, int op = 0)
{
    int x = (int)(table[q].y * pace), y = (int)(table[q].x * pace);
    int temp = 0;
    if (block[x][y + 1])
        temp++;
    if (block[x][y - 1])
        temp++;
    if (block[x + 1][y])
        temp++;
    if (block[x - 1][y])
        temp++;
    if (temp >= 2 && op == 1)
        return 1e9;
    x = (int)(robot[p].y * pace);
    y = (int)(robot[p].x * pace);
    double ans = 1e9;
    for (int i = -1; i <= 1; i++)
        for (int j = -1; j <= 1; j++)
            ans = min(ans, dis[op][q][x + i][y + j]);
    return ans;
}
// 为机器人 p 设置 当前目标和下一目标
void assign_robot(int p, int target, int nexttarget)
{
    robot[p].target = target;
    robot[p].nexttarget = nexttarget;
    st[target][table[target].kind] = st[nexttarget][table[target].kind] = true;
}
double cal_dis(double x1, double y1, double x2, double y2) // 计算两坐标之间的距离
{
    return pow(x2 - x1, 2) + pow(y2 - y1, 2);
}
double cal_dis_rob_tab(int rob_id, int tab_id)
{
    return pow(cal_dis(robot[rob_id].x, robot[rob_id].y, table[tab_id].x, table[tab_id].y), 0.5);
}
double cal_dis_rob_point(int p, pair<double, double> v)
{
    return pow(cal_dis(robot[p].x, robot[p].y, v.first, v.second), 0.5);
}
// 确定机器人的坐标
void confirm(int op, int &X, int &Y)
{
    for (int i = 0; i < 8; i++)
    {
        if (check_block(X, Y, op, -1))
            break;
        if (check_block(X + col[i], Y + row[i], op, -1))
        {
            X = X + col[i];
            Y = Y + row[i];
            break;
        }
    }
}
// 机器人p到终点的距离
double C_D(int p)
{
    if (-1 == robot[p].target)
        return 0;
    return rob_tab(p, robot[p].target, robot[p].kind > 0);
}
int check_direct(int x, int y, int X, int Y, int op, int p = -1)
{
    int a = (x + X) / 2, b = (y + Y) / 2;
    if ((a != x || b != y) && !check_block(a, b, op, p))
        return 0;
    if (!check_block(X, Y, op, p))
        return 0;
    return 1;
}
// 调整速度防撞墙
void adjust(int p, int x, int y)
{
    double degree;
    int left = 0, right = 0, front = 0; // 每个方向只修正一次
    for (int i = -pace; i <= pace; i++)
        for (int j = -pace; j <= pace; j++)
        {
            // 每个方向都修正过了，提前退出
            if (left && right && front)
                return;
            if (x + i < 0 || x + i >= N || y + j < 0 || y + j >= N)
                continue;
            if (block[x + i][y + j])
            {
                degree = atan2(i, j);
                if (abs(robot[p].onto - degree) > pi)
                    degree += degree > 0 ? -2 * pi : 2 * pi;
                // 在侧后方，忽略
                if (abs(robot[p].onto - degree) >= pi / 3 - eps)
                    continue;
                // 障碍物在右前方
                if (!right && robot[p].onto - degree >= 0 && robot[p].onto - degree <= pi / 3)
                {
                    if (robot[p].radv < 0)
                        robot[p].radv += pi / 4;
                    right = 1;
                }
                // 障碍物在左前方
                if (!left && robot[p].onto - degree <= 0 && robot[p].onto - degree >= -pi / 3)
                {
                    if (robot[p].radv > 0)
                        robot[p].radv -= pi / 4;
                    left = 1;
                }
                // 障碍物在正前方
                if (!front && abs(robot[p].onto - degree) <= pi / 8)
                {
                    if (robot[p].kind)
                        robot[p].v /= 2;
                    robot[p].radv *= 100;
                    front = 1;
                }
                if (front && abs(i) + abs(j) <= 2 && abs(robot[p].onto - degree) <= pi / 8)
                {
                    robot[p].v = 1;
                }
            }
        }
}
// 根据目的地的位置来设定机器人的参数，朝向错误时原地调整朝向，朝向正确时最大速度直行
bool setrob(int p)
{
    if (robot[p].target == -1)
    {
        robot[p].radv = robot[p].v = 0;
        return 0;
    }
    int op = robot[p].kind > 0, X = (int)round(robot[p].y * pace), Y = (int)round(robot[p].x * pace);
    confirm(op, X, Y);
    pair<double, double> v = {robot[p].x, robot[p].y}, vv;
    double Dis = 1e9, dd = 1e9;
    int tx, ty;
    for (int i = -pace; i <= pace; i++)
        for (int j = -pace; j <= pace; j++)
        {
            if (X + i < 0 || X + i >= N || Y + j < 0 || Y + j >= N)
                continue;
            if (!check_direct(X, Y, X + i, Y + j, op, p))
                continue;
            if (i == 0 && j == 0)
                continue;
            if (dis[op][robot[p].target][X + i][Y + j] + sqrt(pow(1.0 * i / pace, 2) + pow(1.0 * j / pace, 2)) < Dis)
            {
                Dis = dis[op][robot[p].target][X + i][Y + j] + sqrt(pow(1.0 * i / pace, 2) + pow(1.0 * j / pace, 2));
                v = {(Y + j) * 1.0 / pace, (X + i) * 1.0 / pace};
                tx = X + i;
                ty = Y + j;
            }
            if (!Close_block(X + i, Y + j) && dd > dis[op][robot[p].target][X + i][Y + j])
            {
                dd = dis[op][robot[p].target][X + i][Y + j];
                vv = {(Y + j) * 1.0 / pace, (X + i) * 1.0 / pace};
            }
        }
    if (dd < dis[op][robot[p].target][X][Y] - 1 && Close_block(tx, ty))
        v = vv;
    double x = v.first - robot[p].x, y = v.second - robot[p].y, degree = atan2(y, x);
    robot[p].tardeg = degree;
    if (abs(robot[p].onto - degree) > pi)
        degree += degree > 0 ? -2 * pi : 2 * pi;
    // 调整角度速度
    if (abs(robot[p].onto - degree) > eps)
    {
        robot[p].radv = min(pi, abs(robot[p].onto - degree) / 0.02);
        if (abs(robot[p].onto - degree) <= pi / 4)
            robot[p].radv /= 2;
        if (robot[p].onto > degree)
            robot[p].radv *= -1;
        robot[p].v = 0;
        if (abs(robot[p].onto - degree) < pi / 2)
            robot[p].v = 1;
        if (abs(robot[p].onto - degree) < pi / 3)
            robot[p].v = 4;
        // if(abs(robot[p].onto - degree) < pi / 4)robot[p].v=4;
        if (abs(robot[p].onto - degree) < pi / 6)
            robot[p].v = 6;
    }
    else
    {
        robot[p].radv = 0;
        robot[p].v = 6;
    }
    adjust(p, X, Y);
    if (cal_dis_rob_tab(p, robot[p].target) < 1)
        robot[p].v = min(robot[p].v, 2.0);
    return 1;
}

// 工作台上是否已经有机器人等着
bool unsafe(int pos)
{
    int p = robot[pos].target;
    if (p == -1)
        return 0;
    for (int i = 0; i < 4; i++)
        if (i != pos)
        {
            double dis = rob_tab(i, p, robot[i].kind > 0);
            if (dis < 1 && dis < C_D(pos) && C_D(pos) < 1.7 && robot[i].v < 6)
                return 1;
            if (dis < C_D(pos) && C_D(pos) < 1.7 && robot[pos].target == robot[i].target)
                return 1;
            if (dis < C_D(pos) && C_D(pos) < 2.5 && robot[pos].target == robot[i].target && close_block((int)(table[robot[i].target].y * pace), (int)(table[robot[i].target].x * pace)))
                return 1;
        }
    return 0;
}
// 墙角系数，防止机器人往墙角躲
double corner(int x, int y)
{
    int i = x, j = y, ud, lr;
    while (!block[i][j])
        i++;
    ud = i - x;
    i = x;
    while (!block[i][j])
        i--;
    ud = min(ud, x - i);
    i = x;
    while (!block[i][j])
        j++;
    lr = j - y;
    j = y;
    while (!block[i][j])
        j--;
    lr = min(lr, y - j);
    return (1.0 / ud + 1.0 / lr);
}
// 规避函数，函数值越大越能躲开机器人I
double fun(int a, int x, int y, int op)
{
    int tab = robot[a].target;
    double X = 1.0 * y / pace, Y = 1.0 * x / pace;
    // if (dis[op][tab][x][y] >= dis[op][tab][(int)(robot[a].y * pace)][(int)(robot[a].x * pace)])return 0;
    return 2.5 * cal_dis_rob_point(a, {X, Y}) + 1 * dis[op][tab][x][y] - 4 * corner(x, y);
}
int rob_close(int p)
{
    int x = (int)(robot[p].y * pace), y = (int)(robot[p].x * pace);
    for (int i = -pace; i <= pace; i++)
        for (int j = -pace; j <= pace; j++)
            if (x + i >= 0 && x + i < N && y + j >= 0 && y + j < N && block[x + i][y + j])
                return 1;
    return 0;
}
// p让i
int A_C(int i, int p)
{
    int x = (int)round(robot[p].y * pace + eps), y = (int)round(robot[p].x * pace + eps);
    int X = (int)round(robot[i].y * pace + eps), Y = (int)round(robot[i].x * pace + eps);
    confirm(robot[p].kind > 0, x, y);
    confirm(robot[i].kind > 0, X, Y);
    int op = robot[i].kind > 0, tab = robot[i].target;
    // 不靠墙
    if (!rob_close(i) && !rob_close(p))
    {
        // 直接把初赛的复制过来,删点没用的
        int a = i, b = p;
        // 两个机器速度为0，不需要防碰撞
        if (abs(robot[a].v - robot[b].v) < eps && abs(robot[b].v) < eps)
            return 0;
        double degree = abs(robot[a].onto - robot[b].onto);
        if (degree > pi)
            degree = 2 * pi - degree; // 机器人ab前进的方向的夹角
        double x = robot[b].x - robot[a].x, y = robot[b].y - robot[a].y;
        double degreeb = atan2(y, x); // 向量a->b的方位角
        if (robot[a].onto - degreeb > pi)
            degreeb += 2 * pi;
        if (robot[a].onto - degreeb < -pi)
            degreeb -= 2 * pi;
        // 如果距离较远或者比较平行就不调整
        double dis = sqrt(x * x + y * y) * sin(abs(robot[a].onto - degreeb));
        double mindis = 25;
        if (dis > 2 || x * x + y * y > mindis)
            return 0;
        // 机器人b在a前面且速度很小或接近终点时，a绕开
        if (abs(degreeb - robot[a].onto) < pi / 2 && (abs(robot[b].v) < 1 + eps || C_D(b) < 2))
        {
            if (robot[a].onto - degreeb >= 0 && robot[a].onto - degreeb <= pi / 2)
                robot[a].radv = pi;
            if (robot[a].onto - degreeb <= 0 && robot[a].onto - degreeb >= -pi / 2)
                robot[a].radv = -pi;
            return 0;
        }
        // b机器在a机器后面，且反向行驶，不会撞，不用处理
        // b机器跟在a机器后面,且同向行驶，就给b稍微减速一点，防止追尾
        if (abs(robot[a].onto - degreeb) > pi / 2 && degree < pi / 2)
            robot[b].v -= 1;
        // 计算两机器人的朝向
        double ato = robot[a].onto, bto = robot[b].onto;
        if (ato - bto > pi)
            bto += 2 * pi;
        if (bto - ato > pi)
            ato += 2 * pi;
        // b机器在a机器右前方
        if (robot[a].onto - degreeb <= pi / 2 && robot[a].onto - degreeb >= 0)
        {
            // b机器朝向a的左前方，会撞
            if (bto - ato >= 0 && bto - ato <= pi / 2)
            {
                if (x * x + y * y < 4)
                {
                    robot[a].v -= 3;
                    robot[b].v = 6;
                    robot[a].radv = pi / 2;
                    robot[b].radv = -pi / 2;
                }
                else
                {
                    robot[a].radv = -pi;
                    // robot[b].radv=pi/3;
                }
                // a往减速左躲，b往右躲
            }
            // b机器朝向a的左后方，会撞，距离较近时需要减速
            if (bto - ato >= pi / 2 && bto - ato <= pi)
            {
                robot[a].radv = pi;
                robot[b].radv = pi;
                // ab都往各自的左边躲
                if (dis < 1.5 && x * x + y * y < 10)
                {
                    robot[a].v -= 3;
                    robot[b].v -= 3;
                }
                if (dis < 1 && x * x + y * y < 3)
                {
                    robot[a].v = robot[b].v = 0;
                }
            }
            // b机器朝a的右后方，看起来不会撞，但如果距离很近还是要减速绕一下
            if (bto - ato <= -pi / 2 && bto - ato >= -pi && dis < 1.5)
            {
                robot[a].radv = pi / 2;
                robot[b].radv = pi / 2;
                // ab都往各自的左边躲
                if (x * x + y * y < 4)
                {
                    robot[a].v -= 3;
                    robot[b].v -= 3;
                }
            }
            // b机器朝a的右前方，不会撞，以防万一给a减点速
            if (bto - ato <= 0 && bto - ato >= -pi / 2 && x * x + y * y < 4)
            {
                robot[a].v -= 1;
            }
        }
        // b机器在a机器左前方
        if (robot[a].onto - degreeb >= -pi / 2 && robot[a].onto - degreeb <= 0)
        {
            // b机器朝向a的右前方，会撞
            if (bto - ato <= 0 && bto - ato >= -pi / 2)
            {
                if (x * x + y * y < 4)
                {
                    robot[a].v -= 3;
                    robot[a].radv = -pi / 2;
                    robot[b].radv = pi / 2;
                    robot[b].v = 6;
                }
                else
                {
                    robot[a].radv = pi;
                    // robot[b].radv=-pi/3;
                }
                // a减速往右躲，b往左躲
            }
            // b机器朝向a的右后方，会撞,距离较近时需要减速
            if (bto - ato <= -pi / 2 && bto - ato >= -pi)
            {
                robot[a].radv = -pi;
                robot[b].radv = -pi;
                if (dis < 1.5 && x * x + y * y < 10)
                {
                    robot[a].v -= 3;
                    robot[b].v -= 3;
                }
                if (dis < 1 && x * x + y * y < 3)
                {
                    robot[a].v = robot[b].v = 0;
                }
                // ab都往各自的右边躲
            }
            // b机器朝a的左后方，看起来不会撞，但如果距离很近还是要绕一下
            if (bto - ato >= pi / 2 && bto - ato <= pi && dis < 1.5)
            {
                robot[a].radv = -pi / 2;
                robot[b].radv = -pi / 2;
                if (x * x + y * y < 4)
                {
                    robot[a].v -= 3;
                    robot[b].v -= 3;
                }
            }
            // b机器朝a的左前方，不会撞，以防万一给a减点速
            if (bto - ato >= 0 && bto - ato <= pi / 2 && x * x + y * y < 4)
            {
                robot[a].v -= 1;
            }
        }
    }
    else
    {
        if (1)
        {
            // 距离远时不用管
            if (cal_dis_rob_point(i, {robot[p].x, robot[p].y}) > 6)
                return 0;
            int a = i, b = p;
            double degree = abs(robot[a].onto - robot[b].onto);
            if (degree > pi)
                degree = 2 * pi - degree; // 机器人ab前进的方向的夹角
            double x = robot[b].x - robot[a].x, y = robot[b].y - robot[a].y;
            double degreeb = atan2(y, x); // 向量a->b的方位角
            if (robot[a].tardeg - degreeb > pi)
                degreeb += 2 * pi;
            if (robot[a].tardeg - degreeb < -pi)
                degreeb -= 2 * pi;
            // b在a后面就不调整
            if (abs(robot[a].tardeg - degreeb) > pi / 2)
                return 0;
            // 比较平行就不调整
            double dis = sqrt(x * x + y * y) * sin(abs(robot[a].tardeg - degreeb));
            if (dis > 2)
                return 0;
            if (cal_dis_rob_point(i, {robot[p].x, robot[p].y}) < 3)
                robot[i].v = 3;
            if (cal_dis_rob_point(i, {robot[p].x, robot[p].y}) < 1.5)
                robot[i].v = 1;
            // if (cal_dis_rob_point(i, { robot[p].x,robot[p].y }) < 1.2)robot[i].v = -2;
        }
        int ansx, ansy;
        double maxf = 0;
        for (int j = -pace; j <= pace; j++)
            for (int k = -pace; k <= pace; k++)
            {
                if (x + j < 0 || x + j >= N || y + k < 0 || y + k >= N)
                    continue;
                if (!check_direct(x, y, x + j, y + k, op, p))
                    continue;
                if (fun(i, x + j, y + k, op) > maxf)
                {
                    maxf = fun(i, x + j, y + k, op);
                    ansx = j;
                    ansy = k;
                }
            }
        if (1)
        {
            double degree = atan2(ansx, ansy);
            if (abs(robot[p].onto - degree) > pi)
                degree += degree > 0 ? -2 * pi : 2 * pi;
            // 调整角度速度
            if (abs(robot[p].onto - degree) > eps)
            {
                robot[p].radv = min(pi, abs(robot[p].onto - degree) / 0.02);
                if (robot[p].onto > degree)
                    robot[p].radv *= -1;
                robot[p].v = 0;
                if (abs(robot[p].onto - degree) < pi / 5)
                    robot[p].v = 6;
            }
            else
            {
                robot[p].radv = 0;
                robot[p].v = 6;
            }
        }
    }
    return 1;
}
void avoid_crash(int p)
{
    // p让路
    robot[p].avoid = 1;
    if (robot[p].kind)
    {
        vector<int> temp;
        for (int i = 0; i < p; i++)
            if (i != p && robot[i].kind)
            {
                temp.push_back(i);
            }
        for (auto v : temp)
        {
            if (A_C(v, p))
                return;
        }
    }
    else
    {
        vector<int> temp;
        for (int i = 0; i < 4; i++)
            if (i != p && robot[i].kind)
                temp.push_back(i);
        for (int i = 0; i < p; i++)
            if (i != p && !robot[i].kind)
                temp.push_back(i);
        for (auto v : temp)
            if (A_C(v, p))
                return;
    }
    robot[p].avoid = 0;
    return;
}
int cnttab = 0, cntrob = 0;
bool readmap()
{
    char s[101][101];
    for (int i = 99; i >= 0; i--)
    {
        scanf("%s", s[i]);
        for (int j = 0; j < 100; j++)
            if (s[i][j] == 'A')
            {
                robot[cntrob].x = j * 0.5 + 0.25;
                robot[cntrob].y = i * 0.5 + 0.25;
                cntrob++;
            }
            else if (isdigit(s[i][j]))
            {
                table[cnttab].kind = s[i][j] - '0';
                table[cnttab].x = j * 0.5 + 0.25;
                table[cnttab].y = i * 0.5 + 0.25;
                product_tab[table[cnttab].kind].push_back(cnttab);
                cnttab++;
            }
            else if (s[i][j] == '#')
            {
                block[i * pace / 2][j * pace / 2] = 1;                       // 左下角
                block[i * pace / 2 + pace / 2][j * pace / 2] = 1;            // 右下角
                block[i * pace / 2 + pace / 2][j * pace / 2 + pace / 2] = 1; // 右上角
                block[i * pace / 2][j * pace / 2 + pace / 2] = 1;            // 左上角
                block[i * pace / 2 + pace / 4][j * pace / 2] = 1;            // 下面
                block[i * pace / 2][j * pace / 2 + pace / 4] = 1;            // 左边
                block[i * pace / 2 + pace / 4][j * pace / 2 + pace / 2] = 1; // 上面
                block[i * pace / 2 + pace / 2][j * pace / 2 + pace / 4] = 1; // 右边
                block[i * pace / 2 + pace / 4][j * pace / 2 + pace / 4] = 1; // 中间
            }
    }
    // 边缘设置为障碍物防止寻路出界或机器人撞墙
    for (int i = 0; i < N; i++)
        block[i][0] = block[i][N - 1] = block[0][i] = block[N - 1][i] = 1;
    scanf("%s", s[0]);
    if (s[0][0] == 'O' && s[0][1] == 'K')
        return 1;
    return 0;
}
bool readUntilOK()
{
    scanf("%*d%d", &k);
    memset(num, 0, sizeof(num));
    for (int i = 0; i < k; i++)
    {
        scanf("%d%lf%lf%d%d%d", &table[i].kind, &table[i].x, &table[i].y, &table[i].time, &table[i].material, &table[i].product);
        if (table[i].product)
            num[table[i].kind]++;
    }
    for (int i = 0; i < 4; i++)
    {
        scanf("%d%d%lf%lf%*lf", &robot[i].tableid, &robot[i].kind, &robot[i].time, &robot[i].hit);
        scanf("%lf%lf%lf%lf%lf", &robot[i].vx, &robot[i].vy, &robot[i].onto, &robot[i].x, &robot[i].y);
        if (robot[i].target != -1 && robot[i].kind == 0)
            num[table[robot[i].target].kind]--;
        if (robot[i].shuntarget != -1)
            num[table[robot[i].shuntarget].kind]--;
    }
    char s[3];
    scanf("%s", s);
    if (s[0] == 'O' && s[1] == 'K')
        return true;
    return false;
}
int check_block(int x, int y, int op, int p = -1)
{
    // 遍历2m*2m范围内的点，判断有没有小于半径的障碍物。
    for (int i = -pace; i <= pace; i++)
        for (int j = -pace; j <= pace; j++)
        {
            if (x + i < 0 || x + i >= N || y + j < 0 || y + j >= N)
                continue;
            if (!block[x + i][y + j])
                continue;
            if (pow(i, 2) + pow(j, 2) <= (op == 1 ? 0.53 * 0.53 * pace * pace : 0.45 * 0.45 * pace * pace))
                return 0; // 不能走
        }
    if (p != -1)
    {
        double X = 1.0 * y / pace, Y = 1.0 * x / pace;
        for (int i = 0; i < 4; i++)
            if (i != p && !robot[i].avoid)
            {
                if (cal_dis_rob_point(i, {X, Y}) < 0.9)
                    return 0;
            }
    }

    return 1;
}
int close_block(int x, int y)
{
    for (int i = -pace; i <= pace; i++)
        for (int j = -pace; j <= pace; j++)
            if (x + i >= 0 && x + i < N && y + j >= 0 && y + j < N && block[x + i][y + j])
                return 1;
    return 0;
}
int Close_block(int x, int y)
{
    for (int i = -pace / 2; i <= pace / 2; i++)
        for (int j = -pace / 2; j <= pace / 2; j++)
            if (x + i >= 0 && x + i < N && y + j >= 0 && y + j < N && block[x + i][y + j])
                return 1;
    return 0;
}
// tab出发单源全部最短路，遍历整个图,用dij可以斜着走
void dij_all(int tab, int op)
{
    int X, Y, x, y, cnt = 0;
    x = (int)(table[tab].y * pace);
    y = (int)(table[tab].x * pace);
    for (int i = 0; i < N; i++)
        for (int j = 0; j < N; j++)
            dis[op][tab][i][j] = 1e9;
    dis[op][tab][x][y] = 0;
    priority_queue<pair<double, pair<int, int>>> q; // 大根堆
    q.push({0, {x, y}});
    while (!q.empty())
    {
        auto v = q.top().second;
        x = v.first;
        y = v.second;
        q.pop();
        if (vis[op][tab][x][y])
            continue;
        vis[op][tab][x][y] = 1;
        for (int i = -pace / 2; i <= pace / 2; i++)
            for (int j = -pace / 2; j <= pace / 2; j++)
            {
                if (x + i < 0 || x + i >= N || y + j < 0 || y + j >= N)
                    continue;
                if (check_block(x + i, y + j, op))
                {
                    if (dis[op][tab][x + i][j + y] > dis[op][tab][x][y] + sqrt(pow(1.0 * i / pace, 2) + pow(1.0 * j / pace, 2)))
                    {
                        dis[op][tab][x + i][j + y] = dis[op][tab][x][y] + sqrt(pow(1.0 * i / pace, 2) + pow(1.0 * j / pace, 2));
                        q.push({-dis[op][tab][x + i][y + j], {x + i, y + j}});
                    }
                }
            }
    }
}
// 计算能否到达,op=0表示机器人p到工作台q,op=1表示工作台p到q
void init20()
{
    for (int i = 0; i < cnttab / 2; i++)
    {
        dij_all(i, 0);
        dij_all(i, 1);
    }
}
void init21()
{
    for (int i = cnttab / 2; i < cnttab; i++)
    {
        dij_all(i, 0);
        dij_all(i, 1);
    }
}
void init30()
{
    for (int i = 0; i < cnttab / 3; i++)
    {
        dij_all(i, 0);
        dij_all(i, 1);
    }
}
void init31()
{
    for (int i = cnttab / 3; i < cnttab * 2 / 3; i++)
    {
        dij_all(i, 0);
        dij_all(i, 1);
    }
}
void init32()
{
    for (int i = cnttab * 2 / 3; i < cnttab; i++)
    {
        dij_all(i, 0);
        dij_all(i, 1);
    }
}
void init2Thread()
{
    thread t1 = thread(init20);
    thread t2 = thread(init21);
    t1.join();
    t2.join();
}
void init3Thread()
{
    thread t1 = thread(init30);
    thread t2 = thread(init31);
    thread t3 = thread(init32);
    t1.join();
    t2.join();
    t3.join();
}
// 初始化最短路
void init()
{
    for (int i = 0; i < cnttab; i++)
    {
        dij_all(i, 0);
        dij_all(i, 1);
    }
}
bool do_not_buy(int p)
{
    int target = robot[p].target, nexttarget = robot[p].nexttarget;
    double dis = tab_dis(target, nexttarget, 1);
    double time = dis / 4.5;
    if ((15000 - frameID) * 0.02 < time)
        return true;
    return false;
}
// robot[id]进行交易
void trade(int id)
{
    if (robot[id].target == -1 || robot[id].target != robot[id].tableid)
        return;

    if (robot[id].kind == 0)
    { // 购买物品并更新状态
        if (do_not_buy(id))
        {
            robot[id].pause = 1;
            robot[id].target = robot[id].nexttarget = -1;
            return;
        }
        printf("buy %d\n", id);
        if (table[robot[id].target].mark != -1)
            table[robot[id].target].mark = -1;
        robot[id].kind = table[robot[id].target].kind;
        st[robot[id].target][robot[id].kind] = false;
        robot[id].target = robot[id].nexttarget;
        robot[id].nexttarget = -1;
    }
    else
    { // 出售物品并更新状态
        printf("sell %d\n", id);
        st[robot[id].target][robot[id].kind] = false;
        robot[id].target = robot[id].nexttarget = -1;
        robot[id].kind = 0;
    }
}
// 4个robot都交易
void trade()
{
    for (int i = 0; i < 4; i++)
        trade(i);
}
void cal_empty(int id, int &cnt, int &kind) // 返回编号为id的工作台原料空缺的数量,同时kind表示空缺的材料
{
    cnt = 0;   // cnt表示空缺数目
    kind = -1; // kind表示空缺的材料
    switch (table[id].kind)
    {
    case 4:
        for (int i = 1; i <= 2; i++)
            if (!((table[id].material >> i) & 1) && !st[id][i])
            {
                cnt++;
                kind = i;
            }
        break;
    case 5:
        for (int i = 1; i <= 3; i += 2)
            if (!((table[id].material >> i) & 1) && !st[id][i])
            {
                cnt++;
                kind = i;
            }
        break;
    case 6:
        for (int i = 2; i <= 3; i++)
            if (!((table[id].material >> i) & 1) && !st[id][i])
            {
                cnt++;
                kind = i;
            }
        break;
    case 7:
        for (int i = 4; i <= 6; i++)
            if (!((table[id].material >> i) & 1) && !st[id][i])
            {
                cnt++;
                kind = i;
            }
        break;
    default: // 该情况应该不存在。
        cnt = 4;
        break;
    }
}
// 返回true表示已经找到了目标
bool SearchTable_kind(int p, int &kind, int &tab)
{ // 这个函数用来选中缺物品的7号工作台和需要生产的物品
    static int cnt = 0;
    kind = -1, tab = -1;
    int mincnt = 4;
    for (int i : product_tab[7]) // 遍历7号工作台
    {
        int disI = rob_tab(p, i, 1);
        if (disI > 100000)
            continue;
        int tmpcnt = 0, tmpkind = -1;
        for (int j = 4; j <= 6; j++)                          // 遍历456物品
            if (!((table[i].material >> j) & 1) && !st[i][j]) // 空缺且没有人配送
            {
                bool flag = false;
                for (int k : product_tab[j]) // 查看该空缺物品是否快要生产好，与mark标记有关，mark的更新在SearchTable_target和trade函数中
                    if (table[k].mark == i)
                        flag = true;
                if (flag)
                    continue;      // 如果快要生产后就跳过该物品
                tmpcnt++;          // 否则空缺加1
                if (tmpkind == -1) // 如果没选中空缺物品就选该物品
                    tmpkind = j;
            }
        if (tmpcnt < mincnt && tmpkind != -1)         // 表示空缺少且有缺少物品
            mincnt = tmpcnt, kind = tmpkind, tab = i; // 更新
    }
    if (kind != -1) // 表示选中了缺少物品，返回false
        return false;
    // 如果能到这里，就说明所有7号工作台的所有空缺都有人配送或快要生产好。
    // 也有可能根本没有7号工作台
    // 所有工作台的空缺物品都有人送，就随便选一个物品生产（待优化，但尝试过选最少的物品生产结果效果不好）
    if (product_tab[7].size())
        tab = product_tab[7][0];
    kind = cnt + 4;
    cnt = (cnt + 1) % 3;
    return false;
}
void SearchTable_target(int p, int kind, int tab)
{
    // 该函数根据SearchTable_kind函数选中的空缺物品和空缺7号工作台来调控生产
    // 具体就是根据kind选中要去配送的456工作台，从而确定nexttarget，在去选择对应的123工作台，确定target
    int target = -1, nexttarget = -1, res_cnt = 4, res_kind = -1;
    double mindis = 100000;
    bool flag = false;
    // target表示要去买的123工作台，nexttarget表示要去卖的456工作台，res_kind表示target生产的物品
    for (int i : product_tab[kind]) // 遍历所有生产kind物品的工作台（456）
    {
        double dis = rob_tab(p, i, 1);
        if (dis > 100000)
            continue;
        int tmp_cnt, tmp_kind;           // tmp_cnt tmp_kind分别表示该工作台的空缺位置与空缺物品
        cal_empty(i, tmp_cnt, tmp_kind); // 调用函数
        if (tmp_cnt == 0 || tmp_kind == -1)
            continue; // 如果空缺为0或没有空缺物品就跳过
        // 接下来就是按优先级选工作台和res_kind，优先级为：空缺
        if (res_kind == -1)
            res_kind = tmp_kind, res_cnt = tmp_cnt, nexttarget = i;
        else if (tmp_cnt < res_cnt)
            res_kind = tmp_kind, res_cnt = tmp_cnt, nexttarget = i;
    }
    if (res_kind == -1) // 说明生产kind的工作台满了，机器人没分到任务，给他重新找个任务
    {
        flag = true;
        // 这一步是临时解决方案，用于解决地图2这种只有一个4号工作台的情况
        // 具体就是遍历456物品，随便找一个工作台去分配任务，也不考虑7号工作台是否缺该物品
        for (int i = 4; i <= 6; i++)
            if (i != kind)
            {
                for (int j : product_tab[i])
                { // 步骤和上述一样
                    int dis = rob_tab(p, j, 1);
                    if (dis > 100000)
                        continue;
                    int tmp_cnt, tmp_kind;
                    cal_empty(j, tmp_cnt, tmp_kind);
                    if (tmp_cnt == 0 || tmp_kind == -1)
                        continue;
                    if (res_kind == -1)
                        res_kind = tmp_kind, res_cnt = tmp_cnt, nexttarget = j;
                    else if (tmp_cnt < res_cnt)
                        res_kind = tmp_kind, res_cnt = tmp_cnt, nexttarget = j;
                }
            }
    }
    if (res_kind == -1)
        return;
    // 到此为止，我们应该已经选好了要买的物品res_kind（同时也是要卖的物品），和要卖的工作台。
    mindis = 1e7;
    for (int id : product_tab[res_kind])
    {                                    // 遍历所有生产res_kind物品的工作台,选一个综合距离最近的
        double dis1 = rob_tab(p, id, 0); // 机器人能空手到达该工作台
        double dis2 = tab_dis(id, nexttarget, 1);
        if (dis1 > 100000)
            continue;
        if (dis2 > 100000) // 机器人能从该工作台到nexttarget工作台
            continue;
        if (target == -1)
            target = id, mindis = dis1 + dis2;
        else if (dis1 + dis2 < mindis)
            mindis = dis1 + dis2, target = id;
    }
    if (target == -1) // 这一步很重要，因为可能出现没选到123工作台的情况
        return;
    // 接下来就是一样的步骤
    robot[p].target = target, robot[p].nexttarget = nexttarget;
    st[target][res_kind] = st[nexttarget][res_kind] = true;
    // 这里cal_empty计算该nexttarget是否还有空缺，没有空缺就可以给他的mark赋值
    if (!flag)
    {
        cal_empty(nexttarget, res_cnt, res_kind);
        if (res_cnt == 0)
            table[nexttarget].mark = tab;
    }
}
double cal_f(double x, int maxX, double minRate)
{
    if (x >= maxX)
        return minRate;
    return (1 - sqrt(1 - pow(1 - x / maxX, 2))) * (1 - minRate) + minRate;
}
double potentialValue(int j)
{
    // 8、9号物品没有潜在价值
    if (table[j].kind == 8 || table[j].kind == 9)
        return 0;
    // 设下一个物品的全部潜在价值为其利润的一半
    int potentialv = profit[table[j].kind] / 2;
    // 计算j工作台的空缺
    int emptycnt = 0;
    for (int i : mp[table[j].kind])
        if (((table[j].material >> i) & 1) == 0)
            emptycnt++;
    // 设潜在价值为利润一半再除以空缺数，表示空缺越少，价值应该越高
    potentialv /= emptycnt;
    return potentialv;
}
bool check7_empty(int tabid, int kind)
{
    return ((table[tabid].material >> kind) & 1) == 0;
}
bool check_shun(int p, int tab) // 查看是否有除p以外的机器人正在前往工作台tab
{
    for (int i = 0; i <= 3; i++)
        if (i != p && robot[i].target == tab)
            return true;
    return false;
}
int SearchNear_Nexttarget_for7(int tab) // 返回离该7号工作台最近且能到达的下一级工作台
{
    int dis = 100000, nexttarget = -1;
    for (int i : father[7])
        for (int j : product_tab[i])
            if (tab_dis(tab, j, 1) < dis)
                dis = tab_dis(tab, j, 1), nexttarget = j;
    if (nexttarget != -1)
        return nexttarget;
    return -1; // 按道理不会出现这种情况
}
int SearchNear_Nexttarget_for456(int tab) // 给tab工作台（456类型）找一个下家
{
    // 优先给7送，其次再给9送
    int kind = table[tab].kind;
    for (int i : product_tab[7])
        if (((table[i].material >> kind) & 1) == 0 && !st[i][kind]) // 空缺且没人去送
        {
            int dis = tab_dis(tab, i, 1);
            if (dis > 100000)
                continue;
            return i;
        }
    double mindis = 100000;
    int nexttarget = -1;
    for (int i : product_tab[9])
        if (mindis > tab_dis(tab, i, 1))
            mindis = tab_dis(tab, i, 1), nexttarget = i;
    return nexttarget; // 可能返回-1（但按道理总有下家）
}
bool SearchTable_7(int p) // 考虑是否优先送7
{
    if (product_tab[7].empty()) // 没有7就肯定不考虑了
        return false;
    if (robot[p].tableid != -1 && table[robot[p].tableid].kind == 7) // 保留顺路功能
    {
        int tab = robot[p].tableid;
        if (table[tab].product && !st[tab][7]) // 已经产出且没有人预定
        {
            assign_robot(p, tab, SearchNear_Nexttarget_for7(tab));
            return true;
        }
    }
    // 接下来考虑是否有“不顺路”但有必要去拿的7
    int dis7 = 100000, dis123 = 100000;
    int target = -1, nexttarget = -1;
    for (int i = 1; i <= 3; i++)
        for (int j : product_tab[i])
            if (dis123 > rob_tab(p, j, 0)) // 得到最近且能到达的123工作台距离
                dis123 = rob_tab(p, j, 0);
    for (int i : product_tab[7])
        if (table[i].product && !st[i][7])
        {
            if (rob_tab(p, i, 1) > 100000) // 表示不可到达
                continue;
            if (check_shun(p, i)) // 表示有机器人顺路在去
                continue;
            if (table[i].product && table[i].time >= 0) // 表示该工作台已经堵塞了，必须得优先送
            {
                // SearchNear_Nexttarget_for7返回离7最近且能到达的下一级工作台
                assign_robot(p, i, SearchNear_Nexttarget_for7(i));
                // 按道理这个函数可能返回-1，但按道理不可能，场上有7总得有8或9把
                return true;
            }
            if (rob_tab(p, i, 0) < dis7)
                dis7 = rob_tab(p, i, 0), target = i;
        }
    if (dis7 < dis123) // 如果这个7离的很近，就直接送
    {
        assign_robot(p, target, SearchNear_Nexttarget_for7(target));
        return true;
    }
    return false;
}
bool SearchTable_456(int p)
{
    if (robot[p].tableid != -1) // 保留顺路功能
    {
        int tab = robot[p].tableid;
        if (table[tab].kind >= 4 && table[tab].kind <= 6 && table[tab].product && !st[tab][table[tab].kind])
        {
            int nexttarget = SearchNear_Nexttarget_for456(tab);
            if (nexttarget != -1)
            {
                assign_robot(p, tab, nexttarget);
                return true;
            }
        }
    }
    // 看看7号工作台有没有只剩一个空缺的，就赶紧去送
    for (int i : product_tab[7])
    {
        int target = -1, nexttarget = -1;
        int tmpcnt = 0, tmpkind = -1;
        for (int j = 4; j <= 6; j++)
            if (check7_empty(i, j) && !st[i][j])
                tmpcnt++, tmpkind = j;
        if (tmpcnt == 1 && num[tmpkind]) // 只缺一个，且已经产出
        {
            target = -1, nexttarget = i;
            for (int j : product_tab[tmpkind])
                if (table[j].product && !st[j][tmpkind])
                {
                    if (rob_tab(p, j, 0) > 100000) // 无法抵达
                        continue;
                    if (check_shun(p, j)) // 有机器人顺路过去
                        continue;
                    target = j;
                    break;
                }
            if (target != -1)
            {
                assign_robot(p, target, nexttarget);
                return true;
            }
        }
    }
    double dis123 = 100000, dis456 = 100000, target = -1;
    for (int i = 1; i <= 3; i++)
        for (int j : product_tab[i])
            if (rob_tab(p, j, 0) < dis123) // 最近且能到达的123
                dis123 = rob_tab(p, j, 0);
    // 看有没有已经生产堵塞的456工作台，有就去取
    for (int i = 4; i <= 6; i++)
        for (int j : product_tab[i])
        {
            if (table[j].product && !st[j][i]) // 计算到这个工作台的距离，是能到达的
                if (rob_tab(p, j, 0) < dis456)
                    dis456 = rob_tab(p, j, 0), target = j;
            if (table[j].product && !st[j][i] && table[j].time >= 0)
            {
                if (rob_tab(p, j, 0) > 100000)
                    continue;
                if (check_shun(p, j))
                    continue;
                int nexttarget = SearchNear_Nexttarget_for456(j);
                if (nexttarget != -1)
                {
                    assign_robot(p, j, nexttarget);
                    return true;
                }
            }
        }
    if (dis456 < dis123) // 有456比123近
    {
        int nexttarget = SearchNear_Nexttarget_for456(target);
        if (nexttarget != -1)
        {
            assign_robot(p, target, nexttarget);
            return true;
        }
    }
    return false;
}
int SearchNear_Nexttarget_for456_no7(int tab) // 没有7的情况下，给tab工作台（456类型）找一个下家
{
    // 优先给9送，其次再给7送
    double mindis = 100000;
    int nexttarget = -1;
    for (int i : product_tab[9])
        if (mindis > tab_dis(tab, i, 1))
            mindis = tab_dis(tab, i, 1), nexttarget = i;
    if (nexttarget != -1)
        return nexttarget;
    int kind = table[tab].kind;
    for (int i : product_tab[7])
        if (((table[i].material >> kind) & 1) == 0 && !st[i][kind]) // 空缺且没人去送
        {
            int dis = tab_dis(tab, i, 1);
            if (dis > 100000)
                continue;
            return i;
        }
    return nexttarget; // 可能返回-1（但按道理总有下家）
}
bool SearchTable_456_no7(int p) // 没7情况或7的生产很难的情况，只在顺路的时候或离456很近的时候送456给7或9
{
    if (robot[p].tableid != -1)
    {
        int tab = robot[p].tableid;
        if (table[tab].kind >= 4 && table[tab].kind <= 6 && table[tab].product && !st[tab][table[tab].kind])
        {
            int nexttarget = SearchNear_Nexttarget_for456_no7(tab);
            if (nexttarget != -1)
            {
                assign_robot(p, tab, nexttarget);
                return true;
            }
        }
    }
    double dis123 = 100000, dis456 = 100000;
    for (int i = 1; i <= 3; i++)
        for (int j : product_tab[i])
            if (rob_tab(p, j, 0) < dis123) // 最近且能到达的123
                dis123 = rob_tab(p, j, 0);
    int target = -1;
    for (int i = 4; i <= 6; i++)
        for (int j : product_tab[i])
            if (table[j].product && !st[j][i]) // 最近且能到达的456
                if (rob_tab(p, j, 0) < dis456)
                    dis456 = rob_tab(p, j, 0), target = j;
    if (dis456 < dis123)
    {
        int nexttarget = SearchNear_Nexttarget_for456_no7(target);
        if (nexttarget != -1)
        {
            assign_robot(p, target, nexttarget);
            return true;
        }
    }
    return false;
}
void SearchTable_123_no7(int p) // 没7情况或7的生产很难的情况，就找最近的123送给456
{
    double mindis = 1e9;
    int target = -1, nexttarget = -1, kind;
    for (int i = 1; i <= 3; i++)
        for (int j : product_tab[i])
            for (int k : father[i])
                for (int g : product_tab[k])
                    if (((table[g].material >> i) & 1) == 0 && !st[g][i])
                    {
                        double dis = rob_tab(p, j, 0) + tab_dis(j, g, 1);
                        if (dis > 100000)
                            continue;
                        if (dis < mindis)
                            mindis = dis, target = j, nexttarget = g;
                    }
    if (target != -1)
        assign_robot(p, target, nexttarget);
    // fprintf(fp, "robot%d:target%d->nexttarget%d\n", p, robot[p].target, robot[p].nexttarget);
}
void check_7_easytoget(void) // 返回true表示好生产7，按有7决策，返回false表示不好生产7或没有7，按没有7决策
{
    if (product_tab[7].empty())
    {
        easy7 = false;
        return;
    }
    double disto7[10] = {0};
    int cnt[10] = {0};
    int tab7 = product_tab[7][0];
    for (int i = 4; i <= 6; i++)
        for (int j : product_tab[i])
            if (tab_dis(j, tab7, 1) < 1000000)
            {
                disto7[i] += tab_dis(j, tab7, 1);
                cnt[i]++;
            }
    for (int i = 4; i <= 6; i++)
    {
        if (cnt[i] == 0)
        {
            easy7 = false;
            return;
        }
        disto7[i] /= cnt[i];
        if (disto7[i] > 240)
        {
            easy7 = false;
            return;
        }
    }
}
void SearchTable(int p)
{
    if (easy7)
    {
        if (SearchTable_7(p))
            ;
        else if (SearchTable_456(p))
            ;
        else
        {
            int kind = -1, tab = -1;
            if (!SearchTable_kind(p, kind, tab))
                SearchTable_target(p, kind, tab);
        }
    }
    else
    {
        if (SearchTable_456_no7(p))
            ;
        else
            SearchTable_123_no7(p);
    }
}
void changeTarget()
{
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            if (i == j)
                continue;
            // 当两个都没拿物品或拿着一样的物品
            if (robot[i].kind == robot[j].kind)
            {
                // 两机器人应该都有目标
                if (robot[i].target == -1 || robot[j].target == -1)
                    continue;
                // 现在两机器人到他们目标的总距离
                int dis = rob_tab(i, robot[i].target, robot[i].kind == 0 ? 0 : 1) + rob_tab(j, robot[j].target, robot[j].kind == 0 ? 0 : 1);
                // 交换目标后的总距离
                int newDis = rob_tab(i, robot[j].target, robot[j].kind == 0 ? 0 : 1) + rob_tab(j, robot[i].target, robot[i].kind == 0 ? 0 : 1);
                if (newDis < dis)
                {
                    // 交换目标
                    int tmp = robot[i].target;
                    robot[i].target = robot[j].target;
                    robot[j].target = tmp;
                    tmp = robot[i].nexttarget;
                    robot[i].nexttarget = robot[j].nexttarget;
                    robot[j].nexttarget = tmp;
                }
            }
        }
    }
}
int main()
{
    fp = fopen("log.doc", "w");
    readmap();
    init3Thread();
    check_7_easytoget();
    // init();
    puts("OK");
    fflush(stdout);
    while (scanf("%d", &frameID) != EOF)
    {
        readUntilOK();
        printf("%d\n", frameID);
        double lineSpeed, angleSpeed;
        for (int robotId = 0; robotId < 4; robotId++)
            if (robot[robotId].target == -1 && frameID >= 40 && robot[robotId].pause == 0)
                SearchTable(robotId);
        changeTarget();
        for (int robotId = 0; robotId < 4; robotId++)
        {
            setrob(robotId);
        }
        for (int i = 0; i < 4; i++)
            avoid_crash(i);
        for (int robotId = 0; robotId < 4; robotId++)
        {
            lineSpeed = robot[robotId].v;
            angleSpeed = robot[robotId].radv;
            if (unsafe(robotId))
                lineSpeed = 1;
            // if(robotId)lineSpeed=0;
            printf("forward %d %f\n", robotId, lineSpeed);
            printf("rotate %d %f\n", robotId, angleSpeed);
        }
        trade();
        printf("OK\n");
        fflush(stdout);
    }
    fclose(fp);
    return 0;
}
