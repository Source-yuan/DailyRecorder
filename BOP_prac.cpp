#include <iostream>
// File Description:初始化一个Box，输出体积，修改weith，再次打印体积，输出Box类的实例总个数
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
        double getVolume();


    private:
        double length_;
        double weight_;
        double height_;
        double volume_;
};

int Box::objectCount = 0;

double Box::getVolume(){
    return length_ * weight_ * height_;
}

bool Box::ch_weight(double y){
    weight_ = y;
}


void prt_boxnum(){
    cout<<"Here are "<<Box::objectCount<<" boxes."<<endl;
}

void prt_boxvlm(Box box){
    cout << "The box"<<box.ID_<<"'s volum is "<<box.getVolume()<<endl;
}

int main()
{
    Box box_a(1.0,1.0,1.0);
    Box box_b(2.0,3.0,4.0);
    prt_boxvlm(box_a);
    cout << "The box"<<box_b.ID_<<"'s volum is "<<box_b.getVolume()<<endl;
    box_a.ch_weight(2.0);
    cout << "The changed box"<<box_a.ID_<<"'s volum is "<<box_a.getVolume()<<endl;
    prt_boxnum();
    
    return 0;
}