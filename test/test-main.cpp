#include "controller.h"

using namespace std;

int main() {
    fstream out;
    out.open("log.txt", ios::out);
    out.close();
    out.open("robotPath.txt", ios::out);
    out.close();
    out.open("shipPath.txt", ios::out);
    out.close();
    out.open("port.txt", ios::out);
    out.close();
    out.open("ship.txt", ios::out);
    out.close();
    out.open("item.txt", ios::out);
    out.close();
    out.open("order.txt", ios::out);
    out.close();
    Controller BME2104;
    BME2104.Init();
    BME2104.RunByFrame();
    return 0;
}