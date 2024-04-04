#include "pid.hpp"
#include <iostream>
#include <thread>
using namespace PidController;
int main(int argc, char const *argv[])
{
    float kp=0.625,ki=0.125,kd=0.0;
    PID pid(kp,ki,kd);
    pid.out_limit(-100,100);
    pid.setTargetValue(90);
    float data=0;
    for(int i=0;i<10000;i++){
        data=pid.update(data);
        std::cout<<"data="<<data<<std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(200));

    }
    return 0;
}
