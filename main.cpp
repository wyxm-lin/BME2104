#include "controller.h"

using namespace std;

int main() {
    Controller BME2104;
    BME2104.Init();
    BME2104.PreProcess();
    BME2104.RunByFrame();
    return 0;
}