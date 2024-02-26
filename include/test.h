#ifndef __TEST_H__
#define __TEST_H__

class Test {
public:
    Test(){};
    ~Test(){};

    void setA(int a);
    void setB(int b);

    int getA();
    int getB();

    int sum();

private:
    int a_, b_;
};

#endif