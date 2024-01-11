//
// Created by aung on 2023/12/8.
//

#include "fsm.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv,"end_device");
    ros::NodeHandle nh;
    ros::Rate r(50);
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

    int state_=KEY_INPUT;
    int motor_tc_ = 1;
    int motor_nz_ = 0;
    int32_t insert_speed_=300;
    double skin_thickness_=3.0;
    double insert_depth_=10.0;
    int tc_depth_=5.0;
    int tc_times_per_min_=120;
    int tc_times_=10;
    double nz_angle_=360.0;
    int nz_times_per_min_=120;
    int nz_times_=10;

    while(ros::ok()&&end_effector.CAN1isOpen()&&end_sensor.USB0isOpen()&&end_putter.putterisOpen())
    {
        fsm.completeProcess(&end_effector,&end_sensor,
                            &state_,motor_tc_,motor_nz_,
                            insert_speed_,insert_depth_,skin_thickness_,
                            tc_depth_,tc_times_per_min_,tc_times_,
                            nz_angle_,nz_times_per_min_,nz_times_);
        r.sleep();
    }
    return 0;
}