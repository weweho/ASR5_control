//
// Created by aung on 2023/12/8.
//

#include "end_effector.h"
bool initStatus = false;
bool initStatus2 = false;
int zhaZhenSpeed =300;        //自定义扎针速度  单位度/秒
int initAngle = 38000;
int detAngle = 0;             //扎针角度
int motor_1 = 0;
int motor_2 = 1;

void controlMotorOrder(end_effector::endEffector end_effector)
{
    MOTER_DATA motor_data=end_effector.readMotorData(motor_2);
    PID read_pid=end_effector.readPidParam(motor_2);
    ROS_INFO("speed:%d,temp:%d,encoder:%d,iq:%d\r\n",motor_data.speed,motor_data.temp,motor_data.encoder,motor_data.iq);
    ROS_INFO("anglePidKp:%d,anglePidKp:%d,speedPidKp:%d,speedPidKp:%d,iqPidKp:%d,iqPidKi:%d\r\n",read_pid.anglePidKp,read_pid.anglePidKp,read_pid.speedPidKp,read_pid.speedPidKi,read_pid.iqPidKp,read_pid.iqPidKi);
    if(!initStatus)
        detAngle = initAngle-detAngle;
    else
        detAngle = 0;
    initStatus=end_effector.sendAngleCommand(motor_2,zhaZhenSpeed,detAngle);

    PID write_pid=read_pid;
    write_pid.anglePidKp=0;
    if(end_effector.writePidToRAM(motor_2,write_pid))
    {
        if(!initStatus2)
            detAngle = detAngle-initAngle;
        else
            detAngle = 0;
        initStatus2=end_effector.sendAngleCommand(motor_2,zhaZhenSpeed,detAngle);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv,"end_effector");
    end_effector::endEffector end_effector;

    while(end_effector.Init_CAN1())
    {
        controlMotorOrder(end_effector);
        usleep(20000);
    }

    return 0;
}