//
// Created by aung on 2023/12/8.
//
#include <ros/ros.h>
#include <std_msgs/String.h>

#include "controlcan.h"

using namespace std; //声明命名空间

//初始化启动CAN1并启动
bool Init_CAN1(int nDeviceType, int nDeviceInd, int nCANInd, VCI_INIT_CONFIG config) {
    VCI_ResetCAN(nDeviceType, nDeviceInd, nCANInd);
    if (VCI_InitCAN(nDeviceType, nDeviceInd, nCANInd, &config) != 1)
    {
        VCI_CloseDevice(nDeviceType, nDeviceInd);
        ROS_INFO_STREAM(">>Init CAN1 error");
        return(0);
    }
    VCI_ClearBuffer(nDeviceType, nDeviceInd, nCANInd);
    if (VCI_StartCAN(nDeviceType, nDeviceInd, nCANInd) != 1)
    {
        VCI_CloseDevice(nDeviceType, nDeviceInd);
        ROS_INFO_STREAM(">>Start CAN1 error");
        return(0);
    }
    else
    {
        ROS_INFO_STREAM(">>Start CAN1 success");
        return(1);
    }
}

//初始化启动CAN2并启动
bool Init_CAN2(int nDeviceType, int nDeviceInd, int nCANInd, VCI_INIT_CONFIG config) {
    VCI_ResetCAN(nDeviceType, nDeviceInd, nCANInd);
    if (VCI_InitCAN(nDeviceType, nDeviceInd, nCANInd, &config) != 1)
    {
        VCI_CloseDevice(nDeviceType, nDeviceInd);
        ROS_INFO_STREAM(">>Init CAN2 error");
        return(0);
    }
    VCI_ClearBuffer(nDeviceType, nDeviceInd, nCANInd);
    if (VCI_StartCAN(nDeviceType, nDeviceInd, nCANInd) != 1)
    {
        VCI_CloseDevice(nDeviceType, nDeviceInd);
        ROS_INFO_STREAM(">>Start CAN2 error");
        return(0);
    }
    else
    {
        ROS_INFO_STREAM(">>Start CAN2 success");
        return(1);
    }
}


int main(int argc, char** argv)
{
    //初始化，节点名为can_publisher
    ros::init(argc, argv,"can_publisher");

    //根据遥控器指令发送相应的控制指令
    const unsigned char a0[] = { 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00 }; //默认
    const unsigned char a1[] = { 0xF4,0x01,0x00,0x00,0x00,0x00,0x00,0x00 }; //↑
    const unsigned char a2[] = { 0x00,0x00,0xF4,0x01,0x00,0x00,0x00,0x00 }; //↓
    const unsigned char a3[] = { 0x00,0x00,0x00,0x00,0xF4,0x01,0x00,0x00 }; //←
    const unsigned char a4[] = { 0x00,0x00,0x00,0x00,0x00,0x00,0xF4,0x01 }; //→

    //打开和初始化设备的一些参数配置
    int nDeviceType = 4; //设备类型：CANalyst-II就用4
    int nDeviceInd = 0; //设备索引：1个USB-CAN适配器就是0
    int nCANInd0 = 0;//CAN1
    int nCANInd1 = 1;//CAN2

    //打开设备：注意一个设备只能打开一次
    if(VCI_OpenDevice(nDeviceType,nDeviceInd,0)==1)
    {
        ROS_INFO_STREAM(">>open deivce success!");//打开设备成功
    }else
    {
        ROS_INFO_STREAM(">>open deivce error!");//打开设备失败
        exit(1); //退出整个程序，终止进程：返回1给操作系统说明是非正常运行导致程序退出
    }

    //配置CAN
    VCI_INIT_CONFIG config;
    config.AccCode = 0x80000000;
    config.AccMask = 0xFFFFFFFF;
    config.Filter = 2;   //只接收标准帧
    config.Timing0 = 0x01;
    config.Timing1 = 0x1C; //波特率定为250kps  0x01 0x1C
    config.Mode = 0; //正常模式


    //初始化用来接收的数据帧，帧结构体数组的长度设置为50
    int num = 50;
    VCI_CAN_OBJ rec[num];
    int reclen = 0;
    int i,j,k, order;

    //初始化用来发送的数据帧，帧结构体数组的长度设置为1
    VCI_CAN_OBJ send[1];
    send[0].ID = 0; //帧ID
    send[0].SendType = 0; //发送帧类型：0为正常发送
    send[0].RemoteFlag = 0; //0为数据帧，1为远程帧
    send[0].ExternFlag = 0; //0为标准帧，1为拓展帧
    send[0].DataLen = 8; //数据长度8字节

    //CAN1和CAN2双通道初始化(设备类型、设备索引、CAN通道索引、CAN配置参数)
    if (Init_CAN1(nDeviceType, nDeviceInd, nCANInd0, config) && Init_CAN2(nDeviceType, nDeviceInd, nCANInd1, config))
    {
        while(1)
        {
            //延时20ms：在满足应用的时效性情况下,尽量降低调用VCI_Receive频率，每隔30ms调用一次VCI_Receive为宜
            usleep(20000);

            //VCI_Receive(设备类型、设备索引、CAN通道索引0就是CAN1、接收数组的首指针rec、接收数组的长度50、保留参数)
            if((reclen = VCI_Receive(nDeviceType,nDeviceInd,nCANInd0,rec,num,200)) >= 0)//调用接收函数，如果有数据，则进行处理
            {

                for(i=0; i < reclen; i++)
                {
                    //接收到的遥控器指令
                    order = (int)rec[i].Data[1] + (int)rec[i].Data[2];

                    //先将接收到的遥控器指令打印到终端
                    printf("data:0x");
                    for( j= 0; j < rec[i].DataLen; j++ )
                    {
                        printf(" %02X", rec[i].Data[j]); //%02X表示用2个位置数据一个16进制数
                    }
                    printf("\n");

                    //然后根据遥控器的指令，让CAN2发布相应的控制指令
                    switch (order)
                    {
                        case 0: //默认
                        {
                            for (k = 0; k < 8; k++)
                            {
                                send[0].Data[k] = a0[k];
                            }
                            VCI_Transmit(nDeviceType, nDeviceInd, nCANInd1, send, 1);
                            break;
                        }
                        case 1: //↑
                        {
                            for (k = 0; k < 8; k++)
                            {
                                send[0].Data[k] = a1[k];
                            }
                            VCI_Transmit(nDeviceType, nDeviceInd, nCANInd1, send, 1);
                            break;
                        }
                        case 2: //↓
                        {
                            for (k = 0; k < 8; k++)
                            {
                                send[0].Data[k] = a2[k];
                            }
                            VCI_Transmit(nDeviceType, nDeviceInd, nCANInd1, send, 1);
                            break;
                        }
                        case 4: //←
                        {
                            for (k = 0; k < 8; k++)
                            {
                                send[0].Data[k] = a3[k];
                            }
                            VCI_Transmit(nDeviceType, nDeviceInd, nCANInd1, send, 1);
                            break;
                        }
                        case 8: //→
                        {
                            for (k = 0; k < 8; k++)
                            {
                                send[0].Data[k] = a4[k];
                            }
                            VCI_Transmit(nDeviceType, nDeviceInd, nCANInd1, send, 1);
                            break;
                        }
                        default: //检查和处理错误情况
                        {
                            for (k = 0; k < 8; k++)
                            {
                                send[0].Data[k] = a0[k];
                            }
                            VCI_Transmit(nDeviceType, nDeviceInd, nCANInd1, send, 1);
                            break;
                        }
                    }

                }
            }
        }
    }
    return 0;
}