#ifndef _ITEM_H_
#define _ITEM_H_

class Item {
public:
    int BirthFrame, x, y, value;
    /**
     * @brief Construct function with full parmas
    */
    Item(int fream, int x, int y, int val);

    Item() = default;
    ~Item() = default;
};

#endif