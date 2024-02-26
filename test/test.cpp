#include "test.h"
#include <iostream>

using namespace std;

int main() {
    cout << "this is a example" << endl;
    Test testExample;
    testExample.setA(1);
    testExample.setB(2);
    cout << "a + b = " << testExample.sum() << endl;
    return 0;
}