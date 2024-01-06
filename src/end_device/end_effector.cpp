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
            ROS_INFO_ONCE(">>open deivce success");
        }else
        {
            ROS_INFO_ONCE(">>open deivce fail!");
        }
    }

    bool endEffector::initCAN1() const
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
            ROS_INFO_ONCE(">>Init CAN1 fail!");
            return false;
        }
        return true;
    }

    bool endEffector::CAN1isOpen() const
    {
        VCI_ClearBuffer(nDeviceType, nDeviceInd, nCANInd);
        if (VCI_StartCAN(nDeviceType, nDeviceInd, nCANInd) != 1)
        {
            VCI_CloseDevice(nDeviceType, nDeviceInd);
            ROS_INFO_ONCE(">>Start CAN1 fail!");
            return false;
        }
        else
        {
            ROS_INFO_ONCE(">>Start CAN1 success");
            return true;
        }
    }

    bool endEffector::hasRecData() const {
        VCI_CAN_OBJ rec_[50];
        if(((VCI_Receive(nDeviceType, nDeviceInd, nCANInd, rec_, 50, 100))) >= 1)//调用接收函数，如果有数据，则进行处理
            return true;
        else
            return false;
    }

    bool endEffector::receiveData(VCI_CAN_OBJ *send,VCI_CAN_OBJ *rec) const
    {
        unsigned int reclen_;
        VCI_CAN_OBJ rec_[50];
        if((reclen_=(VCI_Receive(nDeviceType,nDeviceInd,nCANInd,rec_,50,100)))>= 1)//调用接收函数，如果有数据，则进行处理
        {
            for(int i=0; i < reclen_; i++)
            {
                if(rec_[i].Data[0]==send[0].Data[0]&&rec_[i].ID==send[0].ID)
                    rec[0]=rec_[i];
            }
            return true;
        }
        return false;
    }

    bool endEffector::readRawData() const
    {
        unsigned int reclen_;
        VCI_CAN_OBJ rec_[50];
        if((reclen_=(VCI_Receive(nDeviceType,nDeviceInd,nCANInd,rec_,50,100)))>= 1)//调用接收函数，如果有数据，则进行处理
        {
            for(int i=0; i < reclen_; i++)
            {
                printf("reclen_:%d,data:0x",reclen_);
                for (int j = 0; j < rec_[i].DataLen; j++)
                {
                    printf(" %02X", rec_[i].Data[j]); //%02X表示用2个位置数据一个16进制数
                }
                printf("\n");
            }
            return true;
        }
        return false;
    }


    bool endEffector::sendAngleCommand(int motor_ip, uint16_t speed, int32_t angle) const
    {
        if(hasRecData())
            usleep(150); //0.15ms，算出来的不是瞎写的
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
            usleep(150);
            ROS_INFO_ONCE("send angle command succeed\r\n");
            return true;
        }
        else
        {
            ROS_INFO_ONCE("send angle command fail!\r\n");
            return false;
        }
    }

    bool endEffector::sendSpeedCommand(int motor_ip, int32_t speed) const
    {
        if(hasRecData())
            usleep(150);
        VCI_CAN_OBJ send[1];
        send[0].ID = 321+motor_ip; //帧ID
        send[0].SendType = 0; //发送帧类型：0为正常发送
        send[0].RemoteFlag = 0; //0为数据帧，1为远程帧
        send[0].ExternFlag = 0; //0为标准帧，1为拓展帧
        send[0].DataLen = 8; //数据长度8字节
        send[0].Data[0] = 0xA2; //速度闭环控制命令
        send[0].Data[1] = 0x00;
        send[0].Data[2] = 0x00;
        send[0].Data[3] = 0x00;
        send[0].Data[4] = *((uint8_t*)(&speed));
        send[0].Data[5] = *((uint8_t*)(&speed)+1);
        send[0].Data[6] = *((uint8_t*)(&speed)+2);
        send[0].Data[7] = *((uint8_t*)(&speed)+3);

        if(VCI_Transmit(nDeviceType, nDeviceInd, nCANInd, send, 1))
        {
            usleep(150);
            ROS_INFO_ONCE("send speed command succeed\r\n");
            return true;
        }
        else
        {
            ROS_INFO_ONCE("send speed command fail!\r\n");
            return false;
        }
    }

    bool endEffector::readMotorData(int motor_ip ,MOTOR_DATA *motor_data) const
    {
        if(hasRecData())
            usleep(150);
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
            usleep(150);
            if(receiveData(send,rec))
            {
                ROS_INFO_ONCE("send read MotorData command succeed\r\n");
                motor_data->temp=rec[0].Data[1];                               //温度，1℃/LSB
                motor_data->iq=(rec[0].Data[3] << 8) | rec[0].Data[2];         //电流，16.1mA/LSB
                motor_data->speed=(rec[0].Data[5] << 8) | rec[0].Data[4];      //速度，1dps/LSB
                motor_data->encoder=(rec[0].Data[7] << 8) | rec[0].Data[6];    //位置，0.022°/LSB
                ROS_INFO("motor_ip:%d, temp:%d ℃, iq:%f mA, speed:%d dps, encoder:%f °",motor_ip,motor_data->temp,(double)motor_data->iq*16.1,motor_data->speed,(double)motor_data->encoder*0.022);
                return true;
            }
        }
        else
            ROS_INFO_ONCE("read encoder fail!\r\n");
        return false;
    }

    bool endEffector::readPidParam(int motor_ip, PID *pid) const
    {
        if(hasRecData())
            usleep(150);
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
            usleep(150);
            if(receiveData(send,rec))
            {
                ROS_INFO_ONCE("send read PidData command succeed\r\n");
                pid->anglePidKp=rec[0].Data[2];       //位置环P参数
                pid->anglePidKi=rec[0].Data[3];       //位置环I参数
                pid->speedPidKp=rec[0].Data[4];       //速度环P参数
                pid->speedPidKi=rec[0].Data[5];       //速度环I参数
                pid->iqPidKp=rec[0].Data[6];          //转矩环P参数
                pid->iqPidKi=rec[0].Data[7];          //转矩环I参数
                ROS_INFO("anglePidKp:%d,anglePidKi:%d,speedPidKp:%d;",pid->anglePidKp,pid->anglePidKi,pid->speedPidKp);
                ROS_INFO("speedPidKi:%d,iqPidKp:%d,iqPidKi:%d.",pid->speedPidKi,pid->iqPidKp,pid->iqPidKi);
            }
            return true;
        }
        else
            ROS_INFO_ONCE("read pid fail!\r\n");

        return false;
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
            usleep(150);
            ROS_INFO_ONCE("set pid succeed\r\n");
            return true;
        }
        else
        {
            ROS_INFO_ONCE("set pid fail!\r\n");
            return false;
        }
    }



} // end_device