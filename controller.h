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
    int NowFrame = 0;

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
    void ItemUpdateByFrame(int frameID);

    /**
     * @brief pop time-out-items from ItemList queue
    */
    void ItemTimeOutDisappear(int frameID);

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
    void ShipSchedule();

    void ShipScheduleNew();

    /**
     * @brief Operate ship movements in last frames. 
     * Go to the LOWER 5 ports at FrameLess5Ports, then go to the UPPER 5 ports at FrameMoer5Ports
     */
    void ShipScheduleLast();

    /**********************not use in this project********************************/
    void RobotUnavailableSearchNewPath();
};

#endif