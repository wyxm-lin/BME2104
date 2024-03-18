#ifndef _ITEM_H_
#define _ITEM_H_

#include "common.h"
class Item {
public:
    int BirthFrame, x, y, value;
    int destination;
    bool booked;
    bool operator != (const Item &a) const;
    bool operator == (const Item &a) const;
    /**
     * @brief Construct function with full parmas
    */
    Item(int frame, int x, int y, int val, int des);
    Item(int frame, int x, int y, int val): BirthFrame(frame), x(x), y(y), value(val), destination(-1), booked(false) {}

    bool isbooked();
    void book();
    void release();

    Item() = default;
    ~Item() = default;

    /************Below variables and functions are for debug***************/
    void ItemPrintItSelf(std::fstream& out);
};

const Item EmptyItem = Item(0, 0, 0, 0, 0);

#endif