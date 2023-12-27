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
            ROS_INFO_STREAM(">>open deivce success");
        }else
        {
            ROS_INFO_STREAM(">>open deivce fail!");
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
            ROS_INFO_STREAM(">>Init CAN1 fail!");
            return false;
        }
        VCI_ClearBuffer(nDeviceType, nDeviceInd, nCANInd);
        if (VCI_StartCAN(nDeviceType, nDeviceInd, nCANInd) != 1)
        {
            VCI_CloseDevice(nDeviceType, nDeviceInd);
            ROS_INFO_STREAM(">>Start CAN1 fail!");
            return false;
        }
        else
        {
            ROS_INFO_STREAM(">>Start CAN1 success");
            return true;
        }
    }

    bool endEffector::receiveData(VCI_CAN_OBJ *send,VCI_CAN_OBJ *rec) const
    {
        int num_ = 50;
        VCI_CAN_OBJ rec_[num_];
        unsigned int reclen_;
        int i,j;
        //VCI_Receive(设备类型、设备索引、CAN通道索引0就是CAN1、接收数组的首指针rec、接收数组的长度50、保留参数)
        if((reclen_ = VCI_Receive(nDeviceType,nDeviceInd,nCANInd,rec_,num_,200)) >= 0)//调用接收函数，如果有数据，则进行处理
        {
            for (i = 0; i < reclen_; i++) {
                for (j = 0; j < rec_[i].DataLen; j++) {
                    ROS_INFO(" %02X", rec_[i].Data[j]); //%02X表示用2个位置数据一个16进制数
                    if(send[0].Data[0]==rec_[i].Data[0]&&send[0].ID==rec_[i].ID)
                        rec[0]=rec_[i];
                }
                ROS_INFO("\n");
            }
            return true;
        }
        else
            return false;
    }

    bool endEffector::sendAngleCommand(int motor_ip, int speed, int angle) const
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
        send[0].Data[0] = 0xA8; //位置闭环控制命令6
        send[0].Data[1] = 0x00;
        send[0].Data[2] = speedHexString.midRef(6, 2).toUInt(nullptr, 16);
        send[0].Data[3] = speedHexString.midRef(4, 2).toUInt(nullptr, 16);
        send[0].Data[4] = angleHexString.midRef(6, 2).toUInt(nullptr, 16);
        send[0].Data[5] = angleHexString.midRef(4, 2).toUInt(nullptr, 16);
        send[0].Data[6] = angleHexString.midRef(2, 2).toUInt(nullptr, 16);
        send[0].Data[7] = angleHexString.midRef(0, 2).toUInt(nullptr, 16);

        if(VCI_Transmit(nDeviceType, nDeviceInd, nCANInd, send, 1))
        {
            ROS_INFO_ONCE("send command succeed\r\n");
            return true;
        }
        else
        {
            ROS_INFO_ONCE("send command fail!\r\n");
            return false;
        }

    }

    bool endEffector::sendAngleCommand2(int motor_ip, short speed, int angle) const
    {
        VCI_CAN_OBJ send[1];

        send[0].ID = 321+motor_ip; //帧ID
        send[0].SendType = 0; //发送帧类型：0为正常发送
        send[0].RemoteFlag = 0; //0为数据帧，1为远程帧
        send[0].ExternFlag = 0; //0为标准帧，1为拓展帧
        send[0].DataLen = 8; //数据长度8字节
        send[0].Data[0] = 0xA8; //位置闭环控制命令6
        send[0].Data[1] = 0x00;
        send[0].Data[2] = *(uint8_t*)(&speed);
        send[0].Data[3] = *((uint8_t*)(&speed)+1);
        send[0].Data[4] = *((uint8_t*)(&angle));
        send[0].Data[5] = *((uint8_t*)(&angle)+1);
        send[0].Data[6] = *((uint8_t*)(&angle)+2);
        send[0].Data[7] = *((uint8_t*)(&angle)+3);

        if(VCI_Transmit(nDeviceType, nDeviceInd, nCANInd, send, 1))
        {
            ROS_INFO_ONCE("send command succeed\r\n");
            return true;
        }
        else
        {
            ROS_INFO_ONCE("send command fail!\r\n");
            return false;
        }

    }

    MOTER_DATA endEffector::readMotorData(int motor_ip) const
    {
        MOTER_DATA motor_data_{};
        VCI_CAN_OBJ send[1];
        VCI_CAN_OBJ rec[1];
        send[0].ID = 321+motor_ip; //帧ID
        send[0].SendType = 0; //发送帧类型：0为正常发送
        send[0].RemoteFlag = 0; //0为数据帧，1为远程帧
        send[0].ExternFlag = 0; //0为标准帧，1为拓展帧
        send[0].DataLen = 8; //数据长度8字节
        send[0].Data[0] = 0x9C;
        send[0].Data[1] = 0x00;
        send[0].Data[2] = 0x00;
        send[0].Data[3] = 0x00;
        send[0].Data[4] = 0x00;
        send[0].Data[5] = 0x00;
        send[0].Data[6] = 0x00;
        send[0].Data[7] = 0x00;

        if(VCI_Transmit(nDeviceType, nDeviceInd, nCANInd, send, 1))
        {
            while(!receiveData(send,rec));
            motor_data_.temp=rec->Data[1];                              //温度
            motor_data_.iq=(rec -> Data[3] << 8) | rec -> Data[2];      //电流
            motor_data_.speed=(rec -> Data[5] << 8) | rec -> Data[4];   //速度
            motor_data_.encoder=(rec -> Data[7] << 8) | rec -> Data[6]; //位置
            ROS_INFO_ONCE("read encoder succeed\r\n");
        }
        else
            ROS_INFO_STREAM("read encoder fail!\r\n");
        return motor_data_;
    }

    PID endEffector::readPidParam(int motor_ip) const
    {
        PID pid_{};
        VCI_CAN_OBJ send[1];
        VCI_CAN_OBJ rec[1];
        send[0].ID = 321+motor_ip; //帧ID
        send[0].SendType = 0; //发送帧类型：0为正常发送
        send[0].RemoteFlag = 0; //0为数据帧，1为远程帧
        send[0].ExternFlag = 0; //0为标准帧，1为拓展帧
        send[0].DataLen = 8; //数据长度8字节
        send[0].Data[0] = 0x30;
        send[0].Data[1] = 0x00;
        send[0].Data[2] = 0x00;
        send[0].Data[3] = 0x00;
        send[0].Data[4] = 0x00;
        send[0].Data[5] = 0x00;
        send[0].Data[6] = 0x00;
        send[0].Data[7] = 0x00;

        if(VCI_Transmit(nDeviceType, nDeviceInd, nCANInd, send, 1))
        {
            while(!receiveData(send,rec));
            pid_.anglePidKp=rec->Data[2];       //位置环P参数
            pid_.anglePidKi=rec->Data[3];       //位置环I参数
            pid_.speedPidKp=rec->Data[4];       //速度环P参数
            pid_.speedPidKi=rec->Data[5];       //速度环I参数
            pid_.iqPidKp=rec->Data[6];          //转矩环P参数
            pid_.iqPidKi=rec->Data[7];          //转矩环I参数
            ROS_INFO_ONCE("read pid succeed\r\n");
        }
        else
            ROS_INFO_STREAM("read pid fail!\r\n");

        return pid_;
    }

    bool endEffector::writePidToRAM(int motor_ip,PID pid) const
    {
        VCI_CAN_OBJ send[1];
        send[0].ID = 321+motor_ip; //帧ID
        send[0].SendType = 0; //发送帧类型：0为正常发送
        send[0].RemoteFlag = 0; //0为数据帧，1为远程帧
        send[0].ExternFlag = 0; //0为标准帧，1为拓展帧
        send[0].DataLen = 8; //数据长度8字节
        send[0].Data[0] = 0x31;
        send[0].Data[1] = 0x00;
        send[0].Data[2] = *(uint8_t*)(&pid.anglePidKp);
        send[0].Data[3] = *(uint8_t*)(&pid.anglePidKi);
        send[0].Data[4] = *(uint8_t*)(&pid.speedPidKp);
        send[0].Data[5] = *(uint8_t*)(&pid.speedPidKi);
        send[0].Data[6] = *(uint8_t*)(&pid.iqPidKp);
        send[0].Data[7] = *(uint8_t*)(&pid.iqPidKi);

        if(VCI_Transmit(nDeviceType, nDeviceInd, nCANInd, send, 1))
        {
            ROS_INFO_ONCE("send pid succeed\r\n");
            return true;
        }
        else
        {
            ROS_INFO_ONCE("send pid fail!\r\n");
            return false;
        }
    }

} // end_effector