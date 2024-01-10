//
// Created by aung on 2024/1/7.
//
#include "fsm.h"
#include "std_msgs/Float64.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv,"sensor_node");
    ros::NodeHandle nh;
    setlocale(LC_ALL, "");
    setlocale(LC_CTYPE, "zh_CN.utf8");
    int freq=50;
    ros::Rate r(freq);

    end_sensor::endSensor end_sensor;
    file_operator::fileOperator file_operator;
    fsm::FSM fsm;
    end_sensor.initUSB0();

    std_msgs::Float64 sensor_value_;
    ros::Publisher value_pub = nh.advertise<std_msgs::Float64>("/sensor_value", 1000);
    while(ros::ok())
    {
//        end_sensor.getSensorData(&sensor_value_.data);
//        value_pub.publish(sensor_value_);
        fsm.testSensorData(&end_sensor, &file_operator , freq);
        r.sleep();
    }
    return 0;
}