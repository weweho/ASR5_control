//
// Created by aung on 2023/12/8.
//
#include <ros/ros.h>
#include <std_msgs/String.h>

#include "controlcan.h"
#include <QObject>

using namespace std;
int nDeviceType0 = 4; //device type：CANalyst-II(4)
int nDeviceInd0 = 0; //device num：one USB-CAN
int nCANInd0 = 0;//CAN1

bool communicationStatus = true;
bool initStatus = false;

int zhaZhenSpeed =300;        //自定义扎针速度  单位度/秒
int initAngle = 38000;
int detAngle = 0; //扎针角度

bool Init_CAN1(int nDeviceType, int nDeviceInd, int nCANInd, VCI_INIT_CONFIG config) {
    VCI_ResetCAN(nDeviceType, nDeviceInd, nCANInd);
    if (VCI_InitCAN(nDeviceType, nDeviceInd, nCANInd, &config) != 1)
    {
        VCI_CloseDevice(nDeviceType, nDeviceInd);
        ROS_INFO_STREAM(">>Init CAN1 error");
        return -1;
    }
    VCI_ClearBuffer(nDeviceType, nDeviceInd, nCANInd);
    if (VCI_StartCAN(nDeviceType, nDeviceInd, nCANInd) != 1)
    {
        VCI_CloseDevice(nDeviceType, nDeviceInd);
        ROS_INFO_STREAM(">>Start CAN1 error");
        return -1;
    }
    else
    {
        ROS_INFO_STREAM(">>Start CAN1 success");
        return -1;
    }
}

void inquireMotor1Status()
{
    VCI_CAN_OBJ send[1];
    send[0].ID = 321; //帧ID
    send[0].SendType = 0; //发送帧类型：0为正常发送
    send[0].RemoteFlag = 0; //0为数据帧，1为远程帧
    send[0].ExternFlag = 0; //0为标准帧，1为拓展帧
    send[0].DataLen = 8; //数据长度8字节
    send[0].Data[0] = 0x9A;
    send[0].Data[1] = 0x00;
    send[0].Data[2] = 0x00;
    send[0].Data[3] = 0x00;
    send[0].Data[4] = 0x00;
    send[0].Data[5] = 0x00;
    send[0].Data[6] = 0x00;
    send[0].Data[7] = 0x00;

    if(VCI_Transmit(nDeviceType0, nDeviceInd0, nCANInd0, send, 1)){

    }
    else
        ROS_INFO_STREAM("motor1 state abnormal!");

}

void inquireMotor2Status()
{
    VCI_CAN_OBJ send[1];
    send[0].ID = 322; //帧ID
    send[0].SendType = 0; //发送帧类型：0为正常发送
    send[0].RemoteFlag = 0; //0为数据帧，1为远程帧
    send[0].ExternFlag = 0; //0为标准帧，1为拓展帧
    send[0].DataLen = 8; //数据长度8字节
    send[0].Data[0] = 0x9A;
    send[0].Data[1] = 0x00;
    send[0].Data[2] = 0x00;
    send[0].Data[3] = 0x00;
    send[0].Data[4] = 0x00;
    send[0].Data[5] = 0x00;
    send[0].Data[6] = 0x00;
    send[0].Data[7] = 0x00;

    if(VCI_Transmit(nDeviceType0, nDeviceInd0, nCANInd0, send, 1)){

    }
    else
        ROS_INFO_STREAM("motor2 state abnormal!");

}


QString decimalToHex(int decimalNumber)
{
    auto uDecimalNumber = static_cast<unsigned int>(decimalNumber);
    QString hexString = QString("%1").arg(uDecimalNumber, 8, 16, QChar('0'));
    return hexString;
}

void controlMotorOrder()
{
    QString angleHexString;
    QString speedHexString;
    int needleSpeed;
    int needleSpinSpeed;

    VCI_CAN_OBJ send[1];

    if(communicationStatus){
        if(!initStatus)
            detAngle = initAngle-detAngle;
        else
            detAngle = -detAngle;
        needleSpeed = zhaZhenSpeed;

        angleHexString = decimalToHex(detAngle);
        speedHexString = decimalToHex(needleSpeed);

        //回到扎针起点
        send[0].ID = 322; //帧ID
        send[0].SendType = 0; //发送帧类型：0为正常发送
        send[0].RemoteFlag = 0; //0为数据帧，1为远程帧
        send[0].ExternFlag = 0; //0为标准帧，1为拓展帧
        send[0].DataLen = 8; //数据长度8字节
        send[0].Data[0] = 0xA8;
        send[0].Data[1] = 0x00;
        send[0].Data[2] = speedHexString.midRef(6, 2).toUInt(nullptr, 16);
        send[0].Data[3] = speedHexString.midRef(4, 2).toUInt(nullptr, 16);
        send[0].Data[4] = angleHexString.midRef(6, 2).toUInt(nullptr, 16);
        send[0].Data[5] = angleHexString.midRef(4, 2).toUInt(nullptr, 16);
        send[0].Data[6] = angleHexString.midRef(2, 2).toUInt(nullptr, 16);
        send[0].Data[7] = angleHexString.midRef(0, 2).toUInt(nullptr, 16);
        detAngle = 0;


        if(VCI_Transmit(nDeviceType0, nDeviceInd0, nCANInd0, send, 1)){
            initStatus = true;
            ROS_INFO("initial succeed\r\n");

        }
        else{
            initStatus = false;
            communicationStatus = false;
            ROS_INFO("initial fail\r\n");
        }
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv,"can_publisher");

    //open device
    if(VCI_OpenDevice(nDeviceType0,nDeviceInd0,0)==1)
    {
        ROS_INFO_STREAM(">>open deivce success!");
    }else
    {
        ROS_INFO_STREAM(">>open deivce error!");
        exit(1);
    }

    //CAN config
    VCI_INIT_CONFIG config;
    config.AccCode = 0x80000000;
    config.AccMask = 0xFFFFFFFF;
    config.Filter = 1;
    config.Timing0 = 0x00;
    config.Timing1 = 0x14; //baud rate:1mbps
    config.Mode = 0; //normal mode


    if (Init_CAN1(nDeviceType0, nDeviceInd0, nCANInd0, config) )
    {
        while(1)
        {
            controlMotorOrder();
            usleep(20000);//20ms
        }
    }
    return 0;
}