#include "util.h"
#include "controller.h"


using std::string;
using std::cin;
using std::cout;
using std::endl;
using std::pair;

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
    for(int i = 0; i < PortNumber; i++){
        port[i].PortDisInit(&atlas);
    }
}

void Controller::RunByFrame() {

    int nowamoney = 0;
    while(NowFrame < FrameLimit) {
        cin >> NowFrame >> nowamoney;
        ItemUpdateByFrame(NowFrame);

        //FIXME
        // if(NowFrame == 1) {
        //     for(int i = 0; i < 5; i++) {
        //         ship[i].MoveToPort(i);
        //     }
        // }
        
        // if(NowFrame == 13500) {
        //     for(int i = 0; i < 5; i++) {
        //         ship[i].Sell();
        //     }
        // }

        //FIXME

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

        RobotPull();
        GenerateOrders(robot, ItemList, port, ItemMap, atlas, NowFrame);
        RobotGet();
        // avoidCollison(robot, atlas); // FIXME this function have bugs
        RobotMove();
        
        ShipSchedule();

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
    for(int i = 1; i <= NewItemCount; i++) {
        int x, y, val;
        cin >> x >> y >> val;
        int aimid = -1, nowadis = INF;
        for(int j = 0; j < PortNumber; j++) { // search for the nearest port
            if (port[j].isopen() == false) continue; // this port is closed
            int disj = port[j].GetDis(x, y); // get the distance to the port j (this is color)
            if(disj == -1) continue; // unreachable
            if(disj < nowadis) nowadis = disj , aimid = j;
        }
        if(aimid == -1) continue;
        ItemList.emplace(Item(frameID, x, y, val, aimid));
        ItemMap[x][y] = Item(frameID, x, y, val, aimid);
    }
    // Finish new item input
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
            robot[i].pull();
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
            AstarTimeEpsilonWithConflict(robot[i], atlas, 1.0, robot); // search path because task switch
        }
    } 
}

void Controller::RobotMove() {
    for (int i = 0; i < RobotNumber; i++) {
        if (robot[i].IsAvailable == false) {
            if (robot[i].UnavailableMoment == 0) {
                robot[i].UnavailableMoment = NowFrame;
                AstarTimeEpsilonWithConflict(robot[i], atlas, 1.0, robot); // search new path because this robot is unavailable
            }
            else if (robot[i].UnavailableMoment + 20 <= robot[i].NowFrame) { // new crash occured
                robot[i].UnavailableMoment += 10; // FIXME estimate after 10 frames, a new crash occured
                AstarTimeEpsilonWithConflict(robot[i], atlas, 1.0, robot); // search new path because a new crash occured
            }
            continue;
        }
        robot[i].UnavailableMoment = 0; // recover this flag
        if (robot[i].IsWorking == false) {
            continue;
        }
        robot[i].move();
    }
}

void Controller::ShipSchedule() {
    if (NowFrame == 1) {
        for (int i = 0; i < ShipNumber; i++) {
            ship[i].MoveToPort(i);
        }
        return;
    }
    for (int i = 0; i < ShipNumber; i++) {
        if (ship[i].status == SHIPPING) {
            if (ship[i].target == -1) {
                ship[i].MoveToPort(i);
            }
            else {
                if (ship[i].NotMoveMoment == -1)
                    ship[i].NotMoveMoment = NowFrame;
                if (ship[i].HaveLoad < ship[i].capacity) {
                    ship[i].HaveLoad += port[i].velocity;
                    if (ship[i].HaveLoad >= ship[i].capacity || ship[i].NotMoveMoment + 50 <= NowFrame) {
                        ship[i].Sell();
                        ship[i].HaveLoad = 0;
                        ship[i].NotMoveMoment = -1;
                    }
                }
            }
        }
        else if (ship[i].status == MOVING) {
            // do nothing
        }
        else if (ship[i].status == WAITING) {
            // TODO
        }
    }
}