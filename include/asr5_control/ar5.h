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
    string c = "robot"; //�̶���дrobot
    string a;//��������
    // std::string param_st;//�������
    json param_json;
    int param_int;
    bool param_bool;
    int n; //״̬����
    json g;


public:
    ar5();
    virtual ~ar5();
    string To_CBOR(json J);

    json GetInfo();//��ȡ��е������״̬��ϢW
    json GetGeom(); //��ȡλ�õ���Ϣ
    json GetVersion();
    json Power(bool state);//true �ϵ�, false  �µ�
    json Start(bool state);//��е�� ����/ֹͣ true ����, false ֹͣ
    json SwitchToFree(bool state);//��е�� �����϶� true�����϶���false�������϶�
    json SwitchToCloseRobot();  //�رջ�е��
    json pauseMove();  //��е�� ��ͣ�˶�
    json resumeMove();//��е�� �ָ��˶�
    json abortMove();//��е�� ֹͣ�˶�
    json isMoveEnd();//��е�� �˶��Ƿ�ֹͣ  �����һ��bool���͵Ĳ��� true���˶�ֹͣ��false���˶�����
    json startCartFeedTrans(float dir, float delta); //��е�� ��λ�������� dir���᷽�� 1--X  2--Y 3--Z  delta�Ǹ����� ȡֵ��Χ�ǣ�-5��+5��
    json startJointFeedRot(int jointNo, float delta); //��е�� ��е�� �ؽڽ����˶����� jointNo�ǹؽڱ�� 0--5  delta�Ǹ����� ȡֵ��Χ�ǣ�-5��+5��
    json startCartFeedRot(float dir, float delta); //��е�� ��е�� ��ת�����˶�����  delta�Ǹ����� ȡֵ��Χ�ǣ�-5��+5��
    json servoJ(const double para[6],double intervalSecond,bool bSync);
};