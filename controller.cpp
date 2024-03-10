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

    PreProcess();

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
        if(NowFrame == 1) {
            for(int i = 0; i < 5; i++) {
                ship[i].MoveToPort(i);
            }
        }
        
        if(NowFrame == 13500) {
            for(int i = 0; i < 5; i++) {
                ship[i].Sell();
            }
        }

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

        // avoidCollison(robot, atlas);
        RobotActByFrame();
        GenerateOrders(robot, ItemList, port, ItemMap, atlas, NowFrame);

        printf("OK\n");
        fflush(stdout);
    }
}

void Controller::ItemUpdateByFrame(int frameID) {
    int NewItemCount;
    cin >> NewItemCount;
    for(int i = 1; i <= NewItemCount; i++) {
        int x, y, val;
        cin >> x >> y >> val;
        int aimid = -1, nowadis = INF;
        for(int j = 0; j < PortNumber; j++) { // search for the nearest port
            int disj = port[j].GetDis(x, y); // get the distance to the port j (this is color)
            if(disj == -1) continue; // unreachable
            if(disj < nowadis) nowadis = disj , aimid = j;
        }
        if(aimid == -1) continue;
        ItemList.emplace(Item(frameID, x, y, val, aimid));
        ItemMap[x][y] = Item(frameID, x, y, val, aimid);
    }
    // Finish new item input
    ItemTimeOutDisappear(frameID);
    // Kick out disappeared items
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

void Controller::RobotActByFrame() {
    for(int i = 0; i < RobotNumber; i++) {
        if(robot[i].IsAvailable == false) {
            if (robot[i].RecoverFlag == false) {
                robot[i].RecoverFlag = true;
                robot[i].pathIndex --;
            }
            continue;
        }
        robot[i].RecoverFlag = false;
        if(robot[i].IsWorking == false) {
            continue;
        }
        if(robot[i].IsCarry) {
            int aimport = robot[i].targetport;
            if(port[aimport].arrive(robot[i].nowx, robot[i].nowy)) {
                robot[i].DropItem();
            }
            else {
                robot[i].move();
            }
        }
        else {
            if(robot[i].nowx == robot[i].targetX && robot[i].nowy == robot[i].targetY) {
                ItemMap[robot[i].targetX][robot[i].targetY] = EmptyItem;
                int aimport = robot[i].targetport;
                robot[i].TakeItem(port[aimport].x, port[aimport].y);
                // SearchPath(robot[i], atlas);
                // AstarTest(robot, atlas, 1.0, NowFrame); // FIXME
                AstarTimeEpsilon(robot[i], atlas, 1.0);
            }
            robot[i].move();
        }
    }
}