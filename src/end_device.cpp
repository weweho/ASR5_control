//
// Created by aung on 2023/12/8.
//

#include "fsm.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv,"end_device");
    ros::NodeHandle nh;
    ros::Rate r(20);
    setlocale(LC_ALL, "");
    setlocale(LC_CTYPE, "zh_CN.utf8");

    end_effector::endEffector end_effector;
    end_sensor::endSensor end_sensor;
    end_putter::endPutter end_putter;
    file_operator::fileOperator file_operator;
    fsm::FSM fsm;

    end_sensor.initUSB0();
    end_effector.initCAN1();
    end_putter.initPutter();

    int insert_state_=START_MOVE;
    int state_=DEVICE_TEST;
    int motor_tc_ = 1;
    int motor_nz_ = 0;
    while(ros::ok()&&end_effector.CAN1isOpen()&&end_sensor.USB0isOpen()&&end_putter.putterisOpen())
    {
//        fsm.testAllDevice(&end_effector,&end_sensor, &end_putter, &state_,motor_nz_,motor_tc_);
        fsm.testInsertDirect(&end_effector,&end_sensor,&insert_state_,motor_tc_);
        r.sleep();
    }
    return 0;
}