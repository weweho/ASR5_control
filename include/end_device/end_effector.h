//
// Created by aung on 2023/12/25.
//

#ifndef SRC_END_EFFECTOR_H
#define SRC_END_EFFECTOR_H
#include "controlcan.h"
#include <ros/ros.h>

struct MOTOR_DATA{
    uint8_t temp;
    short iq;
    short speed;
    short encoder;
};

struct PID{
    uint8_t anglePidKp;
    uint8_t anglePidKi;
    uint8_t speedPidKp;
    uint8_t speedPidKi;
    uint8_t iqPidKp;
    uint8_t iqPidKi;
};

namespace end_effector
{
class endEffector
{
public:
    endEffector();
    bool initCAN1() const;
    bool CAN1isOpen() const;
    bool readMotorData(int motor_ip ,MOTOR_DATA *motor_data) const;
    bool readPidParam(int motor_ip, PID *pid) const;
    bool writePidToRAM(int motor_ip,PID pid) const;
    bool sendAngleCommand(int motor_ip, uint16_t speed, int32_t angle) const;
    bool sendSpeedCommand(int motor_ip,int32_t speed) const;
    bool readRawData() const;

private:
    bool hasRecData() const;
    bool receiveData(VCI_CAN_OBJ *send,VCI_CAN_OBJ *rec) const;
    int nDeviceType = 4; //device type：CANalyst-II(4)
    int nDeviceInd = 0; //device num：one USB-CAN
    int nCANInd = 0;//CAN1
};

} // end_device

#endif //SRC_END_EFFECTOR_H
