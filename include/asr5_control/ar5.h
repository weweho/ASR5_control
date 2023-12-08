#pragma once
#
#include <iostream>
#include"string"
#include"json.hpp"

#ifndef _AR5_H
#define _AR5_H
#endif
using json = nlohmann::json;
using namespace std;
using namespace nlohmann;


class ar5
{
private:
    string c = "robot"; //固定填写robot
    string a;//函数名称
    // std::string param_st;//命令参数
    json param_json;
    int param_int;
    bool param_bool;
    int n; //状态码编号
    json g;


public:
    ar5();
    virtual ~ar5();
    string To_CBOR(json J);

    json GetInfo();//获取机械臂所有状态信息W
    json GetGeom(); //获取位置点信息
    json GetVersion();
    json Power(bool state);//true 上电, false  下电
    json Start(bool state);//机械臂 启动/停止 true 启动, false 停止
    json SwitchToFree(bool state);//机械臂 自由拖动 true：可拖动，false：不可拖动
    json SwitchToCloseRobot();  //关闭机械臂
    json pauseMove();  //机械臂 暂停运动
    json resumeMove();//机械臂 恢复运动
    json abortMove();//机械臂 停止运动
    json isMoveEnd();//机械臂 运动是否停止  会输出一个bool类型的参数 true：运动停止，false：运动继续
    json startCartFeedTrans(float dir, float delta); //机械臂 单位给进函数 dir是轴方向 1--X  2--Y 3--Z  delta是给进量 取值范围是（-5，+5）
    json startJointFeedRot(int jointNo, float delta); //机械臂 机械臂 关节进给运动函数 jointNo是关节编号 0--5  delta是给进量 取值范围是（-5，+5）
    json startCartFeedRot(float dir, float delta); //机械臂 机械臂 旋转进给运动函数  delta是给进量 取值范围是（-5，+5）
};