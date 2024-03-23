#include "controller.h"

using namespace std;

int main() {
    Controller BME2104;
    BME2104.Init();
    // BME2104.RunByFrame();
    BME2104.InterStellar();
    return 0;
}