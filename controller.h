#ifndef _CONTROLLER_H_
#define _CONTROLLER_H_

#include "common.h"
#include "item.h"
#include "robot.h"
#include "port.h"
#include "ship.h"
#include "atlas.h"
#include "order.h"
#include "shipOrder.h"

using std::queue;
using std::pair;

class Controller {
public:
    Robot robot[RobotNumber];
    Port port[PortNumber];
    Ship ship[ShipNumber];
    queue <Item> ItemList;
    queue <pair<int, int>> ItemPosList;
    Item ItemMap[MapSize][MapSize];
    int NowFrame = 0;

    /************Below variable is for debug**************/
    int RobotStopFrame = 0;

    //ItemMap should be cleared after the robot actually take the item, it is a up-to-time map

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
    void ItemUpdateByFrame();

    /**
     * @brief pop time-out-items from ItemList queue
    */
    void ItemTimeOutDisappear();

    /**
     * @brief Robot All possible action
    */
    void RobotPull();
    void RobotGet();
    void RobotMove();
    void RobotFakePull();
    void RobotFakeGet();
    void RobotRealPull();
    void RobotRealGet();

    /**
     * @brief Maintain port items and ship items and such things
     */
    void AutoShipLoad();
    

    /**
     * @brief Operate ship movements
     */
    void ShipMoveOrSell();
    void ShipSchedule();

    void ShipMoveNew();
    void AutoShipLoadNew();

    /**********************not use in this project********************************/
    void RobotUnavailableSearchNewPath();
};

#endif