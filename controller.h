#ifndef _CONTROLLER_H_
#define _CONTROLLER_H_

#include "common.h"
#include "item.h"
#include "robot.h"
#include "port.h"
#include "ship.h"
#include "atlas.h"

using std::queue;

class Controller {
public:
    Robot robot[RobotNumber];
    Port port[PortNumber];
    Ship ship[ShipNumber];
    Atlas atlas;
    queue<Item> ItemList;
    int ItemValue[MapSize][MapSize];

    Controller() = default;
    ~Controller() = default;

    /**
    * @brief Initize the controller
    */
    void Init();

    /**
     * @brief stage preprocess
    */
    void PreProcess(); // TODO First, port need to call Port.DisInit()

    /**
     * @brief pop time-out-items from ItemList queue
    */
    void ItemTimeOutDisappear(int frameID);

    /**
    * @brief Run the program frame by frame
    */
    void RunByFrame();


};

#endif