#ifndef _CONTROLLER_H_
#define _CONTROLLER_H_

#include "common.h"
#include "item.h"
#include "robot.h"
#include "port.h"
#include "ship.h"
#include "atlas.h"
#include "order.h"

using std::queue;

class Controller {
public:
    Robot robot[RobotNumber];
    Port port[PortNumber];
    Ship ship[ShipNumber];
    Atlas atlas;
    queue <Item> ItemList;
    Item ItemMap[MapSize][MapSize];

    Controller() = default;
    ~Controller() = default;

    /**
    * @brief Initize the controller
    */
    void Init();

    /**
     * @brief stage preprocess
    */
    void PreProcess();

    /**
    * @brief Run the program frame by frame
    */
    void RunByFrame();

    /**
     * @brief Update Item infomation according to the input
    */
    void ItemUpdateByFrame(int frameID);

    /**
     * @brief pop time-out-items from ItemList queue
    */
    void ItemTimeOutDisappear(int frameID);

    /**
     * @brief 
    */
    void Schedule();

    /**
     * @brief Print
    */
    void Print();
};

#endif