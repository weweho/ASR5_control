//
// Created by aung on 2023/12/8.
//

#include "end_effector.h"
short zhaZhenSpeed =500;        //自定义扎针速度  单位度/秒
int detAngle = -1000;
int motor_nz = 0;
int motor_tc = 1;

MOTER_DATA motor_data{};
PID pid{};
int main(int argc, char** argv)
{
    ros::init(argc, argv,"end_effector");
    ros::NodeHandle nh;
    end_effector::endEffector end_effector;
    ros::Rate r(50);
    while(end_effector.Init_CAN1()&&ros::ok())
    {
        end_effector.readMotorData(motor_nz,&motor_data);
        end_effector.readPidParam(motor_nz,&pid);
//        ROS_INFO("pid.anglePidKp:%d,pid.anglePidKi:%d,pid.speedPidKp:%d,pid.speedPidKi:%d,pid.iqPidKp:%d,pid.iqPidKi:%d\r\n",pid.anglePidKp,pid.anglePidKi,pid.speedPidKp,pid.speedPidKi,pid.iqPidKp,pid.iqPidKi);
//        ROS_INFO("temp:%d,iq:%d,speed:%d,encoder:%d\r\n",motor_data.temp,motor_data.iq,motor_data.speed,motor_data.encoder);
        end_effector.sendAngleCommand(motor_nz,zhaZhenSpeed,detAngle);
        r.sleep();
    }
    return 0;
}