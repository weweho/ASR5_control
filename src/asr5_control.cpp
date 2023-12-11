//
// Created by aung on 23-12-8.
//
//ROS
#include "ros/ros.h"
#include <sstream>

//Other
#include <cstdio>
#include"ar5.h"
#include "websocket_endpoint.h"

using namespace std;
unique_ptr<ar5> Arm = make_unique<ar5>();
void asr5_control()
{
    std::string input;
    kagula::websocket_endpoint endpoint;
    endpoint.connect("ws://127.0.0.1:1880");
    double para[6];
    while (1)
    {
        string s;
        int a, b = 0;
        cout << "1 上电、2 下电、3 获取状态信息、 4 获取位置点信息、 5 机械笔启动、6 机械笔停止 7 机械笔自由拖动 8 停止机械笔自由拖动 9 机械臂xyz给进" << endl;
        cout << "10机械臂关节给进  11 机械臂旋转给进 12暂停运动 13恢复运动  14停止运动 15移动到目标位置" << endl;
        printf("Input integer number:");
        scanf("%d", &a);
        switch (a) {
            case 1: printf("上电\n");
                s = Arm->To_CBOR(Arm->Power(true));
                endpoint.send(s);
                break;
            case 2: printf("下电\n");
                s = Arm->To_CBOR(Arm->Power(false));
                endpoint.send(s);
                break;
            case 3: printf("获取状态信息\n");
                s = Arm->To_CBOR(Arm->GetInfo());
                endpoint.send(s);
                break;
            case 4: printf("获取位置点信息\n");
                s = Arm->To_CBOR(Arm->GetGeom());
                endpoint.send(s);
                break;
            case 5: printf("机械臂启动\n");
                s = Arm->To_CBOR(Arm->Start(true));
                endpoint.send(s);
                break;
            case 6: printf("机械臂停止\n");
                s = Arm->To_CBOR(Arm->Start(false));
                endpoint.send(s);
                break;
            case 7: printf("机械臂自由托动\n");
                s = Arm->To_CBOR(Arm->SwitchToFree(true));
                endpoint.send(s);
                break;
            case 8:printf("停止机械臂自由托动\n");
                s = Arm->To_CBOR(Arm->SwitchToFree(false));
                endpoint.send(s);
                break;
            case 9:printf("机械臂给进\n");
                s=Arm->To_CBOR(Arm->startCartFeedTrans(1, 0.5));
                endpoint.send(s);
                break;
            case 10:printf("机械臂关节给进\n");
                s=Arm->To_CBOR(Arm->startJointFeedRot(0, 0.5));
                endpoint.send(s);
                break;
            case 11:printf("机械臂旋转给进\n");
                s = Arm->To_CBOR(Arm->startCartFeedRot(1, 0.5));
                endpoint.send(s);
                break;
            case 12:printf("暂停运动\n");
                s = Arm->To_CBOR(Arm->pauseMove());
                endpoint.send(s);
                break;
            case 13:printf("恢复运动\n");
                s = Arm->To_CBOR(Arm->resumeMove());
                endpoint.send(s);
                break;
            case 14:printf("停止运动\n");
                s = Arm->To_CBOR(Arm->abortMove());
                endpoint.send(s);
                break;
            case 15:printf("移动到目标位置\n");
                s=Arm->To_CBOR(Arm->servoJ(para,5,false));
                endpoint.send(s);
                break;
            default:printf("error\n");
                b = 1;
                break;
        }
        if (b == 1)
        {
            break;
        }

    }
    endpoint.close();
}


int main(int argc, char  *argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"asr5_control");
    ros::NodeHandle nh;
    ros::Rate loop_rate(500);
    while(ros::ok())
    {
        asr5_control();
        loop_rate.sleep();
    }

    return 0;
}
