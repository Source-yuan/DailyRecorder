#include <string.h>
#include <vector>
#include <iostream>
// #define V3d* V3dPtr;
using namespace std;
class Box{
    public:
        static int objectCount;
        int ID_;
        friend void prt_boxnum();
        friend void prt_boxvlm(Box box);

        Box(double x = 2.0, double y = 2.0, double z = 2.0):length_(x), weight_(y), height_(z){
            volume_ = length_ * weight_ * height_;
            ID_ = objectCount;
            // cout << "the object has been created" << endl;
            objectCount++;
        };
        ~Box(){
            // cout << "the object has been destried" << endl;
        };

        bool ch_weight(double y);
        double getVolume(){return length_ * weight_ * height_;};

;       double get_weight(){return weight_;};

        double set_weight(double _weight){weight_ = _weight; return true;};

    private:
        double length_;
        double weight_;
        double height_;
        double volume_;
};

int Box::objectCount = 0;






struct V3d
{
    /* data */
    V3d(double x_ = 0,double y_ = 0,double z_ = 0):x(x_), y(y_), z(z_){};
    double x, y, z;
};


//  Posebag1 = new vector<V>
//数组里可以放自定义类、、指针的阅读方法：先[]为数组 再*为寸处对象是指针。
//new关键字返回一个指针/// *pint = new [i]//



int main()
{   
    int a = 2;
    int* b = new int[5];

    V3d* e = new V3d();

    delete []b;
    delete e;



vector<V3d> *pint[2];
pint[0] = new vector<V3d>;
pint[0]->push_back(V3d());
cout<<"aaaa"<<pint[0]->front().x<<endl;

vector<V3d>


char *vch = new char[20];


return 0;
}


//  p是指针  = new 数组里有七个指针 