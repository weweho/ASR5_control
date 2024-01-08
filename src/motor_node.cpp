//
// Created by aung on 2024/1/7.
//
#include "fsm.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv,"motor_node");
    ros::NodeHandle nh;
    setlocale(LC_ALL, "");
    setlocale(LC_CTYPE, "zh_CN.utf8");
    ros::Rate r(50);

    end_effector::endEffector end_effector;
    file_operator::fileOperator file_operator;
    fsm::FSM fsm;
    end_effector.initCAN1();

    int state_=KEY_INPUT;
    int motor_tc_ = 1;
    int target_speed_=25263;
    int duration_=1;
    while(ros::ok()&&end_effector.CAN1isOpen())
    {
        fsm.testMotorAccuracy(&end_effector,&file_operator,&state_,motor_tc_,target_speed_,(target_speed_*1.5)/100,duration_);
        r.sleep();
    }
    return 0;
}