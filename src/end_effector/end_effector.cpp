//
// Created by aung on 2023/12/25.
//

#include "end_effector.h"

namespace end_effector
{
    endEffector::endEffector()
    {
        //open device
        if(VCI_OpenDevice(nDeviceType,nDeviceInd,0)==1)
        {
            ROS_INFO_STREAM(">>open deivce success!");
        }else
        {
            ROS_INFO_STREAM(">>open deivce error!");
        }
    }

    QString endEffector::decimalToHex(int decimalNumber)
    {
        auto uDecimalNumber = static_cast<unsigned int>(decimalNumber);
        QString hexString = QString("%1").arg(uDecimalNumber, 8, 16, QChar('0'));
        return hexString;
    }

    bool endEffector::Init_CAN1() const
    {
        //CAN config
        VCI_INIT_CONFIG config;
        config.AccCode = 0x80000000;
        config.AccMask = 0xFFFFFFFF;
        config.Filter = 1;
        config.Timing0 = 0x00;
        config.Timing1 = 0x14; //baud rate:1mbps
        config.Mode = 0; //normal mode

        VCI_ResetCAN(nDeviceType, nDeviceInd, nCANInd);
        if (VCI_InitCAN(nDeviceType, nDeviceInd, nCANInd, &config) != 1)
        {
            VCI_CloseDevice(nDeviceType, nDeviceInd);
            ROS_INFO_STREAM(">>Init CAN1 error");
            return false;
        }
        VCI_ClearBuffer(nDeviceType, nDeviceInd, nCANInd);
        if (VCI_StartCAN(nDeviceType, nDeviceInd, nCANInd) != 1)
        {
            VCI_CloseDevice(nDeviceType, nDeviceInd);
            ROS_INFO_STREAM(">>Start CAN1 error");
            return false;
        }
        else
        {
            ROS_INFO_STREAM(">>Start CAN1 success");
            return true;
        }
    }

    void endEffector::inquireMotor1Status()
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

        if(VCI_Transmit(nDeviceType, nDeviceInd, nCANInd, send, 1)){}
        else
            ROS_INFO_STREAM("motor1 state abnormal!");
    }

    void endEffector::inquireMotor2Status()
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

        if(VCI_Transmit(nDeviceType, nDeviceInd, nCANInd, send, 1)){}
        else
            ROS_INFO_STREAM("motor2 state abnormal!");
    }

    void endEffector::sendCommand(int motor_ip, int speed, int angle) const
    {
        VCI_CAN_OBJ send[1];
        QString angleHexString;
        QString speedHexString;

        angleHexString = decimalToHex(angle);
        speedHexString = decimalToHex(speed);

        send[0].ID = 321+motor_ip; //帧ID
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

        if(VCI_Transmit(nDeviceType, nDeviceInd, nCANInd, send, 1))
            ROS_INFO("send command succeed\r\n");
        else
            ROS_INFO("send command fail\r\n");
    }

} // end_effector