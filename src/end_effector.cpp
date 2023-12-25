//
// Created by aung on 2023/12/8.
//

#include "end_effector.h"

bool communicationStatus = true;
bool initStatus = false;

int zhaZhenSpeed =300;        //自定义扎针速度  单位度/秒
int initAngle = 38000;
int detAngle = 0;             //扎针角度
int motor_1 = 0;
int motor_2 = 1;

void controlMotorOrder(end_effector::endEffector end_effector)
{
    if(communicationStatus){
        detAngle = initAngle-detAngle;
        end_effector.sendCommand(motor_2,zhaZhenSpeed,detAngle);
        detAngle = 0;
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv,"end_effector");
    end_effector::endEffector end_effector;
    if(end_effector.Init_CAN1())
    {
       while(1)
        {
            controlMotorOrder(end_effector);
            usleep(20000);
        }
    }
    return 0;
}