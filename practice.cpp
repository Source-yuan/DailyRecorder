#include <vector>
#include <deque>
#include <list>
#include <iostream>
#include <string>

using namespace std;

class waltermelon
{
private:
    double price;
public:
    waltermelon(double price) : price(price){};
    ~waltermelon(){};
    double getPrice(){ return price; };
};




class Humanbing
{
private:
    /* data */
    // enum sexual{human = 0, man = 1};
    int sexual_;
    string job_;
    double money_;
    double life_value_;
    double attack_value_;
    double defense_value_;
    bool alive_;
    int comitte_star_;
public:
    Humanbing(int sexual,string job, double money, double life_value , double attack_value,double defense_value, bool alive):sexual_(sexual),job_(job),money_(money),life_value_(life_value),attack_value_(attack_value),defense_value_(defense_value),alive_(alive){comitte_star_ = 0;};
    ~Humanbing(){};
    bool trade(waltermelon gua);
    bool attack(Humanbing* target);
    void drive();
    int getcomitte_star(){return comitte_star_;};
};

bool Humanbing::attack(Humanbing* target){
    double harme_value;
    harme_value = this->attack_value_ - target->defense_value_;
    if ((target->life_value_ - harme_value) < 0)
    {
        target->alive_ = false;
        this->comitte_star_++;
    }
    return true;
}

void Humanbing::drive(){
    cout << "Drive away~"<<endl;
}

bool Humanbing::trade(waltermelon gua){
    
    if(this->money_ > gua.getPrice()){
        cout << "Trade successed."<<endl;
        return true;
    };
    return false;
}

int main(){
    Humanbing huaqiang(0,"Big Boss",29.0,350.0,300.0,200.0,true);
    Humanbing guafan_(0,"little boss",3000.0,200.0,50.0,5.0,true);
    Humanbing* guafan = &guafan_;
    waltermelon gua(30.0);
    bool trade_flag = huaqiang.trade(gua);
    if (!trade_flag)
    {   
        cout << "trade failure"<<endl;
        huaqiang.attack(guafan);
        cout <<"attack! "<<endl;
        
        if (huaqiang.getcomitte_star())
        {
            huaqiang.drive();
        }
    }
    else
        cout<<"Nothing happend"<<endl;

    return 0;
}