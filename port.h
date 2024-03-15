#ifndef _PORT_H_
#define _PORT_H_

#include "common.h"

using std::string;

class Port {
public:
    int id, x, y, T, velocity;
    int totalItemCnt = 0, nowItemCnt = 0;
    bool isbooked = false;
    bool openstatus = true;

    Port() = default;
    ~Port() = default;

    bool operator < (const Port& p) const {
        return nowItemCnt < p.nowItemCnt;
    }

    bool arrive(int askx, int asky);
    bool isopen();
    void open();
    void close();
    void pull(int value);
    void load(int actualLoadCnt);

    /************Below variables and functions are for debug***************/

};

void PortDisInit(int PortX, int PortY, int id);
int PortGetDis(int aimX, int aimY, int id);

#endif