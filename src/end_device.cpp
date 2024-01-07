//
// Created by aung on 2023/12/8.
//

#include "fsm.h"
#include "std_msgs/Float64.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv,"end_device");
    ros::NodeHandle nh;
    ros::Rate r(20);

    end_effector::endEffector end_effector;
    end_sensor::endSensor end_sensor;
    end_putter::endPutter end_putter;
    file_operator::fileOperator file_operator;
    fsm::FSM fsm;

    end_sensor.initUSB0();
    end_effector.initCAN1();
    end_putter.initPutter();

    int state_=KEY_INPUT;
    int motor_nz_ = 0;
    int motor_tc_ = 1;
    std_msgs::Float64 sensor_value_;
    ros::Publisher value_pub = nh.advertise<std_msgs::Float64>("/sensor_value", 1000);
    while(ros::ok()&&end_effector.CAN1isOpen()&&end_sensor.USB0isOpen()&&end_putter.putterisOpen())
    {
        fsm.testEndDevice(&end_effector,&end_sensor, &end_putter, &state_,motor_nz_,motor_tc_);
        r.sleep();
    }
    return 0;
}