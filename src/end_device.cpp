//
// Created by aung on 2023/12/8.
//

#include "end_effector.h"
#include "end_sensor.h"

short zhaZhenSpeed =50;        //自定义扎针速度  单位度/秒
int detAngle = -100;
double sensor_data{};
int putter_angle=90;
int motor_nz = 0;
int motor_tc = 1;

MOTER_DATA motor_data{};
PID pid{};
int main(int argc, char** argv)
{
    ros::init(argc, argv,"end_device");
    ros::NodeHandle nh;
    end_effector::endEffector end_effector;
    end_sensor::endSensor end_sensor;
    ros::Rate r(50);
    end_sensor.initUSB0();
    while(end_sensor.USB0isOpen()&&ros::ok())
    {
        end_sensor.send_string("1 1 45\n");
        end_sensor.readESPData();
        r.sleep();
    }
    return 0;
}