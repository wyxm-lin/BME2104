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

int MyTotalTransport = 0;
int MyTotalVal = 0;

// unordered_set<int>Allvalue[205];
multiset<int>AllValue[205];
double AllItemAveValue = 0;
int AllItemValue = 0;
int AllItemNum = 0;
vector <pair<int, int>> robotPathSize[RobotNumber];
vector <int> robotItemValue[RobotNumber];

void Controller::Init() {
    for(int i = 0, robotid = -1; i < MapSize; i++) {
        string line;
        cin >> line;
        for(int j = 0; j < MapSize; j++) {
            switch (line[j]) {
            case '.':
                atlas.atlas[i][j] = EMPTY;
                break;
            case '*':
                atlas.atlas[i][j] = WATER;
                break;
            case '#':
                atlas.atlas[i][j] = WALL;
                break;
            case 'A':
                atlas.atlas[i][j] = EMPTY;
                ++robotid;
                robot[robotid].id = robotid;
                robot[robotid].update(i, j, false, true, 0);
                break;
            case 'B':
                atlas.atlas[i][j] = PORT;
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
        RobotStopFrame = max(RobotStopFrame, TotalFrame - T);
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
    atlas.ColorAtlas();
    atlas.DegreeInit();
    for(int i = 0; i < PortNumber; i++){
        port[i].PortDisInit(&atlas);
    }
}

void Controller::RunByFrame() {

    int nowamoney = 0;
    while(NowFrame < FrameLimit) {
        cin >> NowFrame >> nowamoney;
        ItemUpdateByFrame(NowFrame);

        for(int i = 0; i < RobotNumber; i++) {
            int carry, x, y, status;
            cin >> carry >> x >> y >> status;
            robot[i].update(x, y, carry, status, NowFrame);
        }
        // Robots Data Update

        for(int i = 0; i < ShipNumber; i++) {
            int status, tarid;
            cin >> status >> tarid;
            ship[i].update((ShipStatus)status, tarid);
            // the status is accordingly assigned to integers, in 'common.h'
        }
        // Ship Data Update
        string OKstring;
        cin >> OKstring;
        // Read 'OK'

        if (NowFrame < RobotStopFrame) {
            {
                // origin version
                RobotPull();
                GenerateOrders(robot, ItemList, port, ItemMap, atlas, NowFrame);
                RobotGet();
                // avoidCollison(robot, atlas); // NOTE this function 
                RobotMove();
            }   

            // {
            //     // new version (but the score almost less than origin version, need to think about it)
            //     RobotRealPull(); // when the robot real get the pos, switch state
            //     GenerateOrders(robot, ItemList, port, ItemMap, atlas, NowFrame);
            //     RobotRealGet(); // when the robot real get the pos, switch state
            //     // avoidCollison(robot, atlas); // NOTE this function 
            //     RobotMove();
            //     RobotFakePull();
            //     RobotFakeGet();
            // }
        }

        {
            fstream out;
            out.open("log.txt", std::ios::app);
            out << "NowFrame is " << NowFrame << endl;
            out << "port.totalItemCnt is ";
            for (int i = 0; i < PortNumber; i++) {
                out << port[i].totalItemCnt << " ";
            }
            out << endl;
            out << "port.nowItemCnt is ";
            for (int i = 0; i < PortNumber; i++) {
                out << port[i].nowItemCnt << " ";
            }
            out << endl;
            out.close();
        }
        
        AutoShipLoad();
        // ShipSchedule();
        ShipScheduleNew();
        
        if (NowFrame == 15000) {
            {
                fstream out;
                out.open("log.txt", std::ios::app);
                out << MyTotalTransport << endl; // robot have pulled
                out << MyTotalVal << endl;
                out << "ship.haveLoad is ";
                for (int i = 0; i < ShipNumber; i++) {
                    out << ship[i].HaveLoad << " ";
                }
                out << endl;
                out << "port.nowItemCnt is ";
                for (int i = 0; i < PortNumber; i++) {
                    out << port[i].nowItemCnt << " ";
                }
                out << endl;
                out << "AllValue is ";
                for (int i = 1; i <= 200; i++) {
                    out << AllValue[i].size() << " ";
                    if ( i % 20 == 0)
                        out << endl;
                }
                out << endl;
                out.close();
            }
            {
                fstream fout;
                fout.open("robotPath.txt", std::ios::app);
                for (int i = 0; i < RobotNumber; i++) {
                    fout << "robot " << i << " path size is " << robotPathSize[i].size() << endl;
                    for (int j = 0; j < robotPathSize[i].size(); j++) {
                        fout << robotPathSize[i][j].first << "\t" << robotPathSize[i][j].second << "\t";
                        fout << robotItemValue[i][j] << endl;
                    }
                    fout << endl;
                }
                fout.close();
            }
        }

        printf("OK\n");
        fflush(stdout);
    }
    // file.close();
}

void Controller::ItemUpdateByFrame(int frameID) {
    ItemTimeOutDisappear(frameID);
    // Kick out disappeared items

    int NewItemCount;
    cin >> NewItemCount;

    fstream out;
    out.open("log.txt", std::ios::app);
    out << "NowFrame is " << frameID << " NewItemCount is " << NewItemCount << endl;
    
    for(int i = 1; i <= NewItemCount; i++) {
        int x, y, val;
        cin >> x >> y >> val;
        out << "x is " << x << " y is " << y << " val is " << val << endl;
        int aimid = -1, nowadis = INF;
        for(int j = 0; j < PortNumber; j++) { // search for the nearest port
            if (port[j].isopen() == false) continue; // NOTE this port is closed
            int disj = port[j].GetDis(x, y); // get the distance to the port j (this is color)
            if(disj == -1) continue; // unreachable
            if(disj < nowadis) nowadis = disj , aimid = j;
        }
        if(aimid == -1) continue;
        AllItemValue += val;
        AllItemNum ++;
        ItemList.emplace(Item(frameID, x, y, val, aimid));
        ItemMap[x][y] = Item(frameID, x, y, val, aimid);
    }
    // Finish new item input

    out.close();
}

void Controller::ItemTimeOutDisappear(int frameID) {
    while(ItemList.size()) {
        Item it = ItemList.front();
        if(it.BirthFrame + ExistFrame <= frameID) {
            ItemMap[it.x][it.y] = EmptyItem;
            ItemList.pop();
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
            MyTotalTransport ++;
            MyTotalVal += robot[i].carryItem.value;
            int Value = robot[i].carryItem.value;
            AllValue[Value].insert(NowFrame);
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
            AstarTimeEpsilonWithConflict(robot[i], atlas, EPSILON, robot); // search path because task switch
            robotPathSize[i][robotPathSize[i].size() - 1].second = robot[i].pathWithTime.size();
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
            AstarTimeEpsilonWithConflict(robot[i], atlas, EPSILON, robot); // search path because task switch
        }
    }
}

void Controller::RobotMove() {
    for (int i = 0; i < RobotNumber; i++) {
        if (robot[i].IsAvailable == false) {
            if (robot[i].UnavailableMoment == 0) {
                robot[i].UnavailableMoment = NowFrame;
                AstarTimeEpsilonWithConflict(robot[i], atlas, EPSILON, robot); // search new path because this robot is unavailable
            }
            else if (robot[i].UnavailableMoment + 20 <= robot[i].NowFrame) { // new crash occured
                robot[i].UnavailableMoment += 10; // FIXME estimate after 10 frames, a new crash occured
                AstarTimeEpsilonWithConflict(robot[i], atlas, EPSILON, robot); // search new path because a new crash occured
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

void Controller::AutoShipLoad(){
    for(int i = 0; i < ShipNumber; i++){
        if(ship[i].status == SHIPPING){
            if(ship[i].afterSell){  // at the virtual point
                continue;
            }
            int portId = ship[i].target;
            int v = port[portId].velocity;
            int actualLoadCnt;  // change if the ship is full or port is empty
            if((ship[i].HaveLoad + v <= ship[i].capacity) && (port[portId].nowItemCnt - v > 0)){    // ship not full and port not empty
                ship[i].HaveLoad += v;
                actualLoadCnt = v;
            }else if ((ship[i].HaveLoad + v > ship[i].capacity) || (port[portId].nowItemCnt - v <= 0)){  // ship full or port empty
                int shipRemain = ship[i].capacity - ship[i].HaveLoad;
                int portRemain = port[portId].nowItemCnt;
                if(shipRemain == portRemain){
                    ship[i].HaveLoad = ship[i].capacity;
                    ship[i].shipFull = true;
                    ship[i].finishLoad = true;
                    actualLoadCnt = portRemain;
                }else if(shipRemain < portRemain){    // ship can be full
                    ship[i].HaveLoad = ship[i].capacity;
                    ship[i].shipFull = true;
                    actualLoadCnt = shipRemain;
                }else{                                // port can be empty
                    ship[i].HaveLoad += portRemain;
                    actualLoadCnt = portRemain;
                    ship[i].finishLoad = true;
                }
            }
            port[portId].load(actualLoadCnt); // modify the port's nowItemCnt
            // MyTotalTransport += actualLoadCnt;
        }
    }
}

void Controller::ShipSchedule(){
    if (NowFrame == 1) {
        for (int i = 0; i < ShipNumber; i++) {
            ship[i].MoveToPort(i);
            port[i].isbooked = true;
        }
        return;
    }
    // TODO without considering the distance between the ship and the port
    // TODO last some frames, shut down 5 ports
    {
        if (NowFrame == 13500) {
            for (int i = 5; i < PortNumber; i ++) {
                port[i].close();
            }
        }
    }
    priority_queue <Port> heap;
    for (int i = 0; i < PortNumber; i++) {
        if (port[i].isbooked) { // NOTE thick twice, observe the Time ship from Virtual point to port
            continue;
        }
        if (port[i].isopen() == false)
            continue;
        heap.push(port[i]);
    }

    for (int i = 0; i < ShipNumber; i++) {
        if (ship[i].status == SHIPPING) {
            if (ship[i].afterMove) { // ship just arrive at a new port
                {
                    fstream out;
                    out.open("log.txt", std::ios::app);
                    out << "ship " << i << " arrive at " << ship[i].target << endl;
                    out << port[ship[i].target].T << endl;
                    out.close();
                }
                ship[i].afterMove = false;
            }
            else if (ship[i].afterSell) { // ship just arrive at the virtual point
                {
                    fstream out;
                    out.open("log.txt", std::ios::app);
                    out << "ship " << i << " arrive at virtual point" << endl;
                    out.close();
                }
                ship[i].HaveLoad = 0; // NOTE
                while (!heap.empty()) {
                    int Id = heap.top().id;
                    heap.pop(); // add this line
                    if (port[Id].T * 2 + NowFrame >= TotalFrame) { // add this line
                        continue;
                    }
                    ship[i].afterSell = false;
                    port[Id].isbooked = true;
                    ship[i].MoveToPort(Id);
                    break;
                }
            }
            else if (ship[i].shipFull) { // ship is full, go to sell
                {
                    fstream out;
                    out.open("log.txt", std::ios::app);
                    out << "ship " << i << " shipFull" << endl;
                    out.close();
                }
                port[ship[i].target].isbooked = false;
                ship[i].Sell();
            }
            else if (ship[i].finishLoad) { // ship is not full, but the port now is empty
                {
                    fstream out;
                    out.open("log.txt", std::ios::app);
                    out << "ship " << i << " finishLoad" << endl;
                    out.close();
                }
                port[ship[i].target].isbooked = false;
                {
                    if (port[i].T + NowFrame == TotalFrame) {
                        fstream out;
                        out.open("log.txt", std::ios::app);
                        out << "ship " << i << " sell at " << ship[i].target << endl;
                        out.close();
                    }
                }
                if (port[i].T + NowFrame == TotalFrame) {
                    ship[i].Sell();
                }
                else {
                    // if (ship[i].HaveLoad < ship[i].capacity / 2) {  // FIXME adjust parameter
                        bool flag = false;
                        if (!heap.empty()) {
                            int Id = heap.top().id;
                            heap.pop();
                            if (port[Id].T + 500 + NowFrame >= TotalFrame) {
                                continue;
                            }
                            port[Id].isbooked = true;
                            ship[i].MoveToPort(Id);
                            ship[i].finishLoad = false;
                            flag = true;
                        }
                        if (!flag) {
                            ship[i].Sell();
                        }
                    // }
                    // else {
                    //     ship[i].Sell();
                    // }
                }
                
            }
            else { // ship is simply loading
                if (ship[i].target != -1) {
                    if (port[ship[i].target].T + NowFrame == TotalFrame - 1) { // NOTE
                        ship[i].Sell();
                        {
                            fstream out;
                            out.open("log.txt", std::ios::app);
                            out << "ship " << i << " sell at " << ship[i].target << endl;
                            out.close();
                        
                        }
                    }
                }
                
            }
        }
        else if (ship[i].status == MOVING) {
            // TODO maybe do nothing
        }
        else if (ship[i].status == WAITING) {
            // TODO
        }
    }
}

// TODO not be called now
void Controller::ShipScheduleNew(){
    if (NowFrame == 1) {
        for (int i = 0; i < ShipNumber; i++) {
            ship[i].aimPort = i;
            port[i].isbooked = true;
        }
        return;
    }
    GenerateShipOrders(port, ship, NowFrame);

    ShipMoveOrSell();
}

void Controller::ShipMoveOrSell(){
    for(int i = 0; i < ShipNumber; i++){
        if(ship[i].status == SHIPPING){
            if(ship[i].shipFull){
                ship[i].Sell();
                continue;
            }
            if(ship[i].afterSell){  // start at the virtual point
                ship[i].afterSell = false;
                if(ship[i].aimPort == -1){  // no port to go, mainly because it's the last frames
                    continue;
                }
                ship[i].MoveToPort(ship[i].aimPort);
            } else {    // start at a ship[i].target port
                if(ship[i].aimPort == -1){  // no port to go, don't think it will happen
                    ship[i].Sell();
                    continue;
                }
                if(port[ship[i].target].T + NowFrame >= TotalFrame){    // last frames
                    ship[i].Sell();
                    continue;
                }
                if(ship[i].aimPort != ship[i].target){  // need to move to another port
                    if(port[ship[i].aimPort].T + 500 + NowFrame >= TotalFrame){
                        ship[i].Sell();
                    }else{
                        ship[i].MoveToPort(ship[i].aimPort);
                    }
                }
            }
        }else if(ship[i].status == MOVING){
            // do nothing
        }else if(ship[i].status == WAITING){
            // do nothing
        }
    }
    
}



/*****************************not use in this project****************************/
void Controller::RobotUnavailableSearchNewPath() {
    for (int i = 0; i < RobotNumber; i++) {
        if (robot[i].IsAvailable == false) {
            if (robot[i].UnavailableMoment == 0) {
                robot[i].UnavailableMoment = NowFrame;
                AstarTimeEpsilonWithConflict(robot[i], atlas, EPSILON, robot); // search new path because this robot is unavailable
            }
            else if (robot[i].UnavailableMoment + 20 <= robot[i].NowFrame) { // new crash occured
                robot[i].UnavailableMoment += 10; // FIXME estimate after 10 frames, a new crash occured
                AstarTimeEpsilonWithConflict(robot[i], atlas, EPSILON, robot); // search new path because a new crash occured
            }
        }
    }
}
