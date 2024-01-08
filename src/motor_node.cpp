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
    ros::Rate r(1);
    end_effector::endEffector end_effector;
    file_operator::fileOperator file_operator;
    fsm::FSM fsm;
    int state_=KEY_INPUT;
    int motor_tc_ = 1;
    while(ros::ok()&&end_effector.CAN1isOpen())
    {
        fsm.testMotorAccuracy(&end_effector,&file_operator,&state_,motor_tc_,25236);
        r.sleep();
    }
    return 0;
}