#include <iostream>
// File Description:
using namespace std;
class Base{
public:
    virtual void mf1() = 0;
    virtual void mf1(int);
    virtual void mf2();
    void mf3();
    void mf3(double);
private:
    int x_;
};

class Derived : private Base{
public:
    virtual void mf1(double)
    {Base::mf1(int);};
    
    void mf3(char *);



private:

};

int main()
{
    Derived d;
    int x = 0;
    d.mf1();
    return 0;
}