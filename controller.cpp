#include "util.h"
#include "controller.h"


using std::string;
using std::cin;
using std::cout;
using std::endl;
using std::pair;
using std::priority_queue;
using std::fstream; // TODO remove this line
using std::max;
using std::unordered_set;
using std::multiset;
using std::vector;
using std::min;

int AllItemValue = 0;
int AllItemNum = 0;
double AllItemAveValue = 0;
int TotalPullCount = 0;
int TotalPullValue = 0;

#ifdef LOG
    /*below are used for debug*/
    multiset<int>AllValue[205];

    vector <pair <pair<int, int>, int> > robotPathSize[RobotNumber];
    vector <pair<int, int> > robotItemValue[RobotNumber];
    vector <pair <pair<int, int>, int > > shipPathSize[ShipNumber];
    vector <int> portShutDown;
    /*above are used for debug*/
#endif

extern MapStatus atlas[MapSize][MapSize];

void Controller::Init() {
    for(int i = 0, robotid = -1; i < MapSize; i++) {
        string line;
        cin >> line;
        for(int j = 0; j < MapSize; j++) {
            switch (line[j]) {
            case '.':
                atlas[i][j] = EMPTY;
                break;
            case '*':
                atlas[i][j] = WATER;
                break;
            case '#':
                atlas[i][j] = WALL;
                break;
            case 'A':
                atlas[i][j] = EMPTY;
                ++robotid;
                robot[robotid].id = robotid;
                robot[robotid].update(i, j, false, true, 0);
                break;
            case 'B':
                atlas[i][j] = PORT;
                break;
            default:
                break;
            }
        }
    }
    // atlas information finished

    for(int i = 0; i < PortNumber; i++) {
        int id, x, y, T, v;
        cin >> id >> x >> y >> T >> v;
        port[id].id = id;
        port[id].x = x;
        port[id].y = y;
        port[id].T = T;
        port[id].velocity = v;
        RobotStopFrame = max(RobotStopFrame, TotalFrame - T); // NOTE
    }
    // port information finished

    int cap; cin >> cap;
    for(int i = 0; i < ShipNumber; i++) {
        ship[i].id = i;
        ship[i].capacity = cap;
    }
    // ship information finished

    PreProcess();
    // pre-process

    string OKstring;
    cin >> OKstring;
    cout << "OK" << endl;
    // End stage preprocess here
    // endl would automatically fflush the std output
}

void Controller::PreProcess() {
    ColorAtlas();
    // DegreeInit();
    for (int id = 0; id <PortNumber; id ++) {
        PortDisInit(port[id].x, port[id].y, id);
    }
    for (int id = 0; id < RobotNumber; id++) {
        RobotDisUpdate(robot[id].nowx, robot[id].nowy, id);
    }
    ShipMoveDecision(port, ship);
}

void Controller::RunByFrame() {
    int nowamoney = 0;
    while(NowFrame < FrameLimit) {
        cin >> NowFrame >> nowamoney;
        ItemUpdateByFrame();

        for(int i = 0; i < RobotNumber; i++) {
            int carry, x, y, status;
            cin >> carry >> x >> y >> status;
            robot[i].update(x, y, carry, status, NowFrame);
        }
        // Robots Data Update

        for(int i = 0; i < ShipNumber; i++) {
            int status, tarid;
            cin >> status >> tarid;
            ship[i].update((ShipStatus)status, tarid, NowFrame);
            // ship[i].NowFrame = NowFrame;
            // the status is accordingly assigned to integers, in 'common.h'
        }
        // Ship Data Update
        string OKstring;
        cin >> OKstring;
        // Read 'OK'

        if (NowFrame == 13500) {
            for (int i = 0; i < PortNumber; i ++) {
                port[i].close();
            }
            for (int i = 0; i < ShipNumber; i++) {
                if (ship[i].target == -1)
                    continue;
                port[ship[i].target].open();
            }
        }

        if (NowFrame <= RobotStopFrame) {
            {
                // origin version
                RobotPull();
                // GenerateOrdersNew(robot, ItemPosList, port, ItemMap, NowFrame);
                GenerateOrdersVersion4(robot, ItemPosList, port, ItemMap, NowFrame);
                RobotGet();
                // avoidCollison(robot, atlas); // NOTE this function 
                RobotMove();
            }   

            // {
            //     // new version (but the score almost less than origin version, need to think about it)
            //     RobotRealPull(); // when the robot real get the pos, switch state
            //     GenerateOrdersNew(robot, ItemPosList, port, ItemMap, NowFrame);
            //     RobotRealGet(); // when the robot real get the pos, switch state
            //     // avoidCollison(robot, atlas); // NOTE this function 
            //     RobotMove();
            //     RobotFakePull();
            //     RobotFakeGet();
            // }
        }
        
        // AutoShipLoad();
        // ShipSchedule();
        // GenerateShipOrdersNew(port, ship, NowFrame);
        // ShipMoveNew();
        ShipMoveInFixTurn();
        AutoShipLoadNew();
    
        {
#ifdef DEBUG
            fstream out;

            out.open("port.txt", std::ios::app);
            out << "NowFrame is " << NowFrame << endl;
            out << "port isopen is" << " ";
            for(int i = 0; i < PortNumber; i++){
                out << port[i].isopen() << " ";
            }
            out << endl;
            out << "port nowItemCnt is" << " ";
            for (int i = 0; i < PortNumber; i++) {
                out << port[i].nowItemCnt << " ";
            }
            out << endl;
            out << "port totalItemCnt is" << " ";
            for (int i = 0; i < PortNumber; i++) {
                out << port[i].totalItemCnt << " ";
            }
            out << endl;
            out.close();

            out.open("ship.txt", std::ios::app);
            out << "NowFrame is " << NowFrame << endl;
            out << "ShipState is " << endl;
            for (int i = 0; i < ShipNumber; i++) {
                if (ship[i].AtPortLoading) {
                    out << ship[i].target << "Loading ";
                }
                else if (ship[i].AtPortWaiting) {
                    out << ship[i].target << "Waiting ";
                }
                else if (ship[i].AtVirtualPoint) {
                    out << "AtVirtualPoint" << " ";
                }
                else if (ship[i].FromPortToVirtual) {
                    out << ship[i].oldTarget << "->Virtual ";
                }
                else if (ship[i].FromVirtualToPort) {
                    out << "Virtual->" << ship[i].aimPort << " ";
                }
                else if (ship[i].FromPortToPort) {
                    out << ship[i].oldTarget << "->" << ship[i].aimPort << " ";
                }
            }
            out << endl;
            out << "Ship HaveLoad is " << endl;
            for (int i = 0; i < ShipNumber; i++) {
                out << ship[i].HaveLoad << " ";
            }

            out << endl;
            out << "Total Pulled Value: " << TotalPullValue << endl;
            out << "Total Generated Value:" << AllItemValue << endl;
            out << TotalPullCount << " " << AllItemNum << endl;
            out << endl;
            out.close();
#endif
        }

#ifdef LOG
    {
        if(NowFrame == 15000){
            fstream out;
            out.open("robotPath.txt", std::ios::app);
            for(int i = 0; i < RobotNumber; i++){
                out << "Robot " << i << " path size is " << robotPathSize[i].size() << endl;
                for(int j = 0; j < robotPathSize[i].size(); j++){
                    out << robotPathSize[i][j].first.first << " " << robotPathSize[i][j].first.second << " " << robotPathSize[i][j].second << endl;
                }
            }
            out.close();
        }
    }
#endif

        printf("OK\n");
        fflush(stdout);
    }
    // file.close();
}

void Controller::ItemUpdateByFrame() {
    ItemTimeOutDisappear();
    // Kick out disappeared items

    int NewItemCount;
    cin >> NewItemCount;

#ifdef DEBUG
    fstream out;
    out.open("item.txt", std::ios::app);
    out << "NowFrame is " << NowFrame << " NewItemCount is " << NewItemCount << endl;
#endif
    for(int i = 1; i <= NewItemCount; i++) {
        int x, y, val;
        cin >> x >> y >> val;
        AllItemValue += val; // maintain this variable
        AllItemNum ++; // maintain this variable
#ifdef DEBUG
        out << "x is " << x << " y is " << y << " val is " << val << endl;
#endif
        for (int j = 0; j < PortNumber; j ++) {
            if (port[j].isopen() == false) {
                continue;
            }
            int disj = PortGetDis(x, y, j);
            if (disj != -1) {
                ItemPosList.push({x, y});
                ItemMap[x][y] = Item(NowFrame, x, y, val);
                // ItemMap[x][y].destination = ItemChoosePort(ItemMap[x][y], port);
                // AllItemValue += val; // maintain this variable
                // AllItemNum ++; // maintain this variable
                break;
            }
        }
    }
    // Finish new item input
#ifdef DEBUG
    out.close();
#endif
}

void Controller::ItemTimeOutDisappear() {
    while (ItemPosList.size()) {
        int x = ItemPosList.front().first, y = ItemPosList.front().second;
        if (ItemMap[x][y].BirthFrame + ExistFrame <= NowFrame) {
            ItemMap[x][y] = EmptyItem;
            ItemPosList.pop();
        }
        else {
            break;
        }
    }
}

void Controller::RobotPull() {
    for (int i = 0; i < RobotNumber; i++) {
        if (robot[i].IsAvailable == false || robot[i].IsWorking == false || robot[i].IsCarry == false) {
            continue;
        }
        int aimport = robot[i].targetport;
        if (port[aimport].arrive(robot[i].nowx, robot[i].nowy)) {
            port[aimport].pull(robot[i].carryItem.value);
            robot[i].pull();
#ifdef DEBUG
            TotalPullCount ++; // maintain this variable
            TotalPullValue += robot[i].carryItem.value; // maintain this variable
            // int Value = robot[i].carryItem.value;
            // AllValue[Value].insert(NowFrame); // maintain this variable
#endif
        }
    }
}

void Controller::RobotGet() {
    for (int i = 0; i < RobotNumber; i ++) {
        if (robot[i].IsAvailable == false || robot[i].IsWorking == false || robot[i].IsCarry == true) {
            continue;
        }
        if (robot[i].nowx == robot[i].targetX && robot[i].nowy == robot[i].targetY) {
            ItemMap[robot[i].targetX][robot[i].targetY] = EmptyItem; // kick out 
            int aimport = robot[i].targetport; // task switch
            robot[i].get(port[aimport].x, port[aimport].y);
            AstarTimeEpsilonWithConflict(robot[i], EPSILON, robot); // search path because task switch
#ifdef LOG
            robotPathSize[i][robotPathSize[i].size() - 1].first.second = robot[i].pathWithTime.size();
#endif
        }
    } 
}

void Controller::RobotFakePull() {
    for (int i = 0; i < RobotNumber; i++) {
        if (robot[i].IsAvailable == false || robot[i].IsWorking == false || robot[i].IsCarry == false) {
            continue;
        }
        robot[i].FakePull(port[robot[i].targetport].x, port[robot[i].targetport].y);
    }
}

void Controller::RobotRealPull() {
    for (int i = 0; i < RobotNumber; i++) {
        if (robot[i].IsAvailable == false || robot[i].IsWorking == false) {
            continue;
        }
        int aimport = robot[i].targetport;
        if (port[aimport].arrive(robot[i].nowx, robot[i].nowy)) {
            port[aimport].pull(robot[i].carryItem.value);
            robot[i].RealPull();
        }
    }
}

void Controller::RobotFakeGet() {
    for (int i = 0; i < RobotNumber; i++) {
        if (robot[i].IsAvailable == false || robot[i].IsWorking == false || robot[i].IsCarry == true) {
            continue;
        }
        robot[i].FakeGet();
    }
}

void Controller::RobotRealGet() {
    for (int i = 0; i < RobotNumber; i++) {
        if (robot[i].IsAvailable == false || robot[i].IsWorking == false) {
            continue;
        }
        if (robot[i].nowx == robot[i].targetX && robot[i].nowy == robot[i].targetY) { 
            ItemMap[robot[i].targetX][robot[i].targetY] = EmptyItem; // kick out 
            robot[i].RealGet(port[robot[i].targetport].x, port[robot[i].targetport].y);
            AstarTimeEpsilonWithConflict(robot[i], EPSILON, robot); // search path because task switch
        }
    }
}

void Controller::RobotMove() {
    for (int i = 0; i < RobotNumber; i++) {
        if (robot[i].IsAvailable == false) {
            if (robot[i].UnavailableMoment == 0) {
                robot[i].UnavailableMoment = NowFrame;
                AstarTimeEpsilonWithConflict(robot[i], EPSILON, robot); // search new path because this robot is unavailable // TODO 
            }
            else if (robot[i].UnavailableMoment + 20 <= robot[i].NowFrame) { // new crash occured
                robot[i].UnavailableMoment += 10; // FIXME estimate after 10 frames, a new crash occured
                AstarTimeEpsilonWithConflict(robot[i], EPSILON, robot); // search new path because a new crash occured
            }
            // FIXME not sure if this is the right way to handle this
            robot[i].NextX = robot[i].nowx;
            robot[i].NextY = robot[i].nowy;
            tryRetakeOrder(robot[i]);
            continue;
        }
        robot[i].UnavailableMoment = 0; // recover this flag
        if (robot[i].IsWorking == false) {
            robot[i].NextX = robot[i].nowx;
            robot[i].NextY = robot[i].nowy;
            continue;
        }
        robot[i].move();
    }
}

void Controller::AutoShipLoad() {
    for (int i = 0; i < ShipNumber; i++) {
        if (ship[i].status == SHIPPING) {
            if (ship[i].afterSell) {  // at the virtual point
                continue;
            }
            int portId = ship[i].target;
            int v = port[portId].velocity;
            int actualLoadCnt;  // change if the ship is full or port is empty
            if ((ship[i].HaveLoad + v <= ship[i].capacity) && (port[portId].nowItemCnt - v > 0)) {    // ship not full and port not empty NOTE this have a bug
                ship[i].HaveLoad += v;
                actualLoadCnt = v;
            } 
            else if ((ship[i].HaveLoad + v > ship[i].capacity) || (port[portId].nowItemCnt - v <= 0)){  // ship full or port empty
                int shipRemain = ship[i].capacity - ship[i].HaveLoad;
                int portRemain = port[portId].nowItemCnt;
                if (shipRemain == portRemain) { // ship full & port empty
                    ship[i].HaveLoad = ship[i].capacity;
                    ship[i].shipFull = true;
                    ship[i].finishLoad = true;
                    actualLoadCnt = portRemain;
                }
                else if (shipRemain < portRemain) { // ship full & port not empty
                    ship[i].HaveLoad = ship[i].capacity;
                    ship[i].shipFull = true;
                    actualLoadCnt = shipRemain;
                }
                else {                              // port empty & ship not full
                    ship[i].HaveLoad += portRemain;
                    actualLoadCnt = portRemain;
                    ship[i].finishLoad = true;
                }
            }
            port[portId].load(actualLoadCnt); // modify the port's nowItemCnt
        }
    }
}

void Controller::ShipMoveOrSell() {
    for (int i = 0; i < ShipNumber; i++) {
        if (ship[i].status == SHIPPING) {
            if (ship[i].shipFull) {
                ship[i].Sell();
                continue;
            }
            if (ship[i].afterSell) {  // start at the virtual point
                ship[i].afterSell = false;
                if (ship[i].aimPort == -1) {  // no port to go, mainly because it's the last frames
                    continue;
                }
                ship[i].MoveToPort(ship[i].aimPort);
            } 
            else {    // start at a ship[i].target port
                if (ship[i].aimPort == -1) {  // no port to go, don't think it will happen
                    ship[i].Sell();
                    continue;
                }
                if (port[ship[i].target].T + NowFrame >= TotalFrame) {    // last frames
                    ship[i].Sell();
                    continue;
                }
                if (ship[i].aimPort != ship[i].target) {  // need to move to another port
                    if (port[ship[i].aimPort].T + 500 + NowFrame >= TotalFrame) {
                        ship[i].Sell();
                    }
                    else{
                        ship[i].MoveToPort(ship[i].aimPort);
                    }
                }
            }
        }
        else if(ship[i].status == MOVING) {
            // do nothing
        }
        else if(ship[i].status == WAITING) {
            // do nothing
        }
    }
}

void Controller::ShipSchedule(){
    if (NowFrame == 1) {
        // TODO maybe need to change the strategy
        for (int i = 0; i < ShipNumber; i++) {
            ship[i].aimPort = i;
            port[i].isbooked = true;
        }
        return;
    }

    if (NowFrame == FrameLastTimeHandle) { // TODO think twice
        HandleLastFrames(port, ship, NowFrame);
    }

#ifdef LOG
    {
       if(NowFrame == FrameLastTimeHandle + 1){

            fstream out;
            out.open("port.txt", std::ios::app);
            for(int i = 0; i < PortNumber; i++){
                out << port[i].isopen() << " ";
            }
            out << endl;
            out.close();
        }
    }
#endif

    GenerateShipOrders(port, ship, NowFrame);
    ShipMoveOrSell();
    return;
}

void Controller::ShipMoveNew() {
    for (int i = 0; i < ShipNumber; i++) {
        if (ship[i].FromPortToVirtual == true || ship[i].FromVirtualToPort || ship[i].FromPortToPort == true) {
            // TODO maybe do nothing
        }
        else if (ship[i].AtPortWaiting == true) {
            // TODO maybe do nothing (ps. Now this situation won't happen)
        }
        else if (ship[i].AtVirtualPoint == true) {
            if (ship[i].PrintShip == true) {
                ship[i].MoveToPort(ship[i].aimPort);
            }
        }
        else if (ship[i].AtPortLoading == true) {
            if (ship[i].PrintGo == true) {
                ship[i].Sell();
            }
            else if (ship[i].PrintShip == true) {
                ship[i].MoveToPort(ship[i].aimPort);
            }
        }
    }
}

void Controller::ShipMoveInFixTurn() {
    int FrameInTurn = (NowFrame - 1) % TurnFrame + 1;
    for(int i = 0; i < ShipNumber; i++) {
        if(FrameInTurn == 1) {
            printf("ship %d %d\n", i, ship[i].FixedTurnAim[1]);
        }
        if(FrameInTurn == port[ship[i].FixedTurnAim[1]].T + ship[i].WaitTime[1] + 1) {
            printf("ship %d %d\n", i, ship[i].FixedTurnAim[2]);
        }
        if(FrameInTurn == port[ship[i].FixedTurnAim[1]].T + ship[i].WaitTime[1] + 501 + ship[i].WaitTime[2]) {
            printf("go %d\n", i);
        }
    }
    fflush(stdout);
}

void Controller::AutoShipLoadNew() {
    for (int i = 0; i < ShipNumber; i ++) {
        ship[i].ShouldLeaveNowPort = false;
        if (ship[i].AtPortLoading) {
            int portId = ship[i].target;
            int actualLoadCnt = min(min(ship[i].capacity - ship[i].HaveLoad, port[portId].nowItemCnt), port[portId].velocity);
            ship[i].HaveLoad += actualLoadCnt;
            port[portId].load(actualLoadCnt);
            if (ship[i].HaveLoad == ship[i].capacity) { // ship is full
                ship[i].ShouldLeaveNowPort = true;
                port[portId].isbooked = false;
            }
            else if (port[portId].nowItemCnt == 0) { // port is empty
                if (NowFrame >= 13500) {
                    // do nothing, just wait for items at this port
                }
                else {
                    ship[i].ShouldLeaveNowPort = true;
                    port[portId].isbooked = false;
                }
            }
            else {
                ship[i].ShouldLeaveNowPort = false;
            }
        }
    }
}

/*****************************not use in this project****************************/
void Controller::RobotUnavailableSearchNewPath() {
    for (int i = 0; i < RobotNumber; i++) {
        if (robot[i].IsAvailable == false) {
            if (robot[i].UnavailableMoment == 0) {
                robot[i].UnavailableMoment = NowFrame;
                AstarTimeEpsilonWithConflict(robot[i], EPSILON, robot); // search new path because this robot is unavailable
            }
            else if (robot[i].UnavailableMoment + 20 <= robot[i].NowFrame) { // new crash occured
                robot[i].UnavailableMoment += 10; // FIXME estimate after 10 frames, a new crash occured
                AstarTimeEpsilonWithConflict(robot[i], EPSILON, robot); // search new path because a new crash occured
            }
        }
    }
}
