#ifndef _PORT_H_
#define _PORT_H_

#include "common.h"

using std::string;

class Atlas;

class Port {
public:
    int id, x, y, T, velocity;
    int dis[MapSize][MapSize];
    int totalItemCnt = 0, nowItemCnt = 0;
    bool isbooked = false;
    // int totalItemValue = 0, nowItemValue = 0;

    bool openstatus = true;
    Port() = default;
    ~Port() = default;
    bool operator < (const Port& p) const {
        return nowItemCnt < p.nowItemCnt;
    }
    /**
     * @brief Initize the dis
     */
    void PortDisInit(Atlas* atlas);

    int GetDis(int aimX, int aimY);
    bool arrive(int askx, int asky);
    bool isopen();
    void open();
    void close();
    void pull(int value);
    void load(int actualLoadCnt);
    /************Below variables and functions are for debug***************/
    void PrintDis(int x_, int y_);

};

#endif