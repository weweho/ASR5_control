//
// Created by aung on 2023/12/25.
//

#ifndef SRC_END_EFFECTOR_H
#define SRC_END_EFFECTOR_H
#include "controlcan.h"
#include <ros/ros.h>
#include <QObject>
namespace end_effector
{
class endEffector
{
public:
    endEffector();
    bool Init_CAN1() const;
    void inquireMotor1Status();
    void inquireMotor2Status();
    bool sendCommand(int motor_ip,int speed, int angle) const;

private:
    static QString decimalToHex(int decimalNumber);
    int nDeviceType = 4; //device type：CANalyst-II(4)
    int nDeviceInd = 0; //device num：one USB-CAN
    int nCANInd = 0;//CAN1
};

} // end_effector

#endif //SRC_END_EFFECTOR_H
