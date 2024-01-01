//
// Created by aung on 2023/12/8.
//

#include "end_effector.h"
#include "end_sensor.h"

short zhaZhenSpeed =500;        //自定义扎针速度  单位度/秒
int detAngle = -1000;
double sensor_data{};
int putter_angle=45;
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
    while(end_effector.initCAN1()&&end_sensor.initUSB0()&&ros::ok())
    {
        end_effector.readMotorData(motor_nz,&motor_data);
        end_effector.readPidParam(motor_nz,&pid);
        end_effector.sendAngleCommand(motor_nz,zhaZhenSpeed,detAngle);

        end_sensor.getSensorData(&sensor_data);
        end_sensor.sendPutterCommand(true,putter_angle);
        r.sleep();
    }
    return 0;
}