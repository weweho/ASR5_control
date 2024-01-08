//
// Created by aung on 2024/1/7.
//
#include "fsm.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv,"sensor_node");
    ros::NodeHandle nh;
    setlocale(LC_ALL, "");
    setlocale(LC_CTYPE, "zh_CN.utf8");
    int freq=20;
    ros::Rate r(freq);
    end_sensor::endSensor end_sensor;
    file_operator::fileOperator file_operator;
    fsm::FSM fsm;
    end_sensor.initUSB0();
    while(ros::ok())
    {
        fsm.testSensorData(&end_sensor, &file_operator , freq);
        r.sleep();
    }
    return 0;
}