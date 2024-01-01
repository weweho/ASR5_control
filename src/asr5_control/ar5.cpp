#include"ar5.h"

ar5::ar5(/* args */)
{
}

ar5::~ar5()
{
    //delete this;
}


string ar5::To_CBOR(json J)
{
    // json j3={{"a","power"},{"c","robot"},{"d",true},{"n",1137}};
    cout << J.dump() << endl;
    std::vector<std::uint8_t> v_cbor = json::to_cbor(J);
    string code_str;
    for (int i = 0; i < v_cbor.size(); i++)
    {
        //����16��������"ʮλ"�͡���λ��
        char s1 = char(v_cbor[i] >> 4);
        char s2 = char(v_cbor[i] & 0xf);
        //������õ�������ת���ɶ�Ӧ��ASCII�룬���ֺ���ĸ�ֿ���ͳһ����Сд����
        s1 > 9 ? s1 += 87 : s1 += 48;
        s2 > 9 ? s2 += 87 : s2 += 48;
        //������õ��ַ����뵽string��
        code_str.append(1, s1);
        code_str.append(1, s2);
    }
    cout << "code_str=" << code_str << endl;
    return code_str;
}





json ar5::GetInfo()
{
    this->c = "robot";
    this->a = "getInfo";
    this->param_int = 0;
    this->n = 1227;

    json j = {
                {"c",c},
                {"a",a},
                {"d",param_int},
                {"n",n}
    };
    cout << "j=   " << j.dump() << endl;
    return j;
}

json ar5::GetGeom()
{
    this->c = "robot";
    this->a = "getGeom";
    this->param_int = 0;
    this->n = 1104;

    json j = {
                {"c",c},
                {"a",a},
                {"d",param_int},
                {"n",n}
    };
    cout << "j=   " << j.dump() << endl;
    return j;
}

json ar5::GetVersion()
{
    this->c = "robot";
    this->a = "getVersions";
    // this->param_int=0;
    this->n = 10006;

    json j = {
                {"c",c},
                {"a",a},
                // {"d",param_int},
                 {"n",n}
    };
    cout << "j=   " << j.dump() << endl;
    return j;
}

json ar5::Power(bool state)
{
    this->c = "robot";
    this->a = "power";
    this->param_bool = state;
    this->n = 1137;

    json j = {
                {"c",c},
                {"a",a},
                {"d",param_bool},
                {"n",n}
    };
    cout << "j=   " << j.dump() << endl;
    return j;
}


json ar5::Start(bool state)
{
    this->c = "robot";
    this->a = "start";
    this->param_bool = state;
    this->n = 1199;

    json j = {
                {"c",c},
                {"a",a},
                {"d",param_bool},
                {"n",n}
    };
    cout << "j=   " << j.dump() << endl;
    return j;
}

json ar5::SwitchToFree(bool state)
{

    if (state == 1)
    {

        this->param_json = { 0,true };
    }
    else
    {

        this->param_json = { 0,false };
    }

    this->c = "robot";
    this->a = "switchToFree";
    this->param_bool = state;
    this->n = 1013;

    json j = {
                {"c",c},
                {"a",a},
                {"d",param_json},
                {"n",n}
    };
    cout << "j=   " << j.dump() << endl;
    return j;
}

json ar5::SwitchToCloseRobot()
{
    this->c = "robot";
    this->a = "SwitchToCloseRobot";

    this->n = 163837;

    json j = {
                {"c",c},
                {"a",a},

                {"n",n}
    };
    cout << "j=   " << j.dump() << endl;
    return j;
}

json ar5::pauseMove()
{
    this->c = "robot";
    this->a = "pauseMove";
    this->param_bool = true;
    this->n = 1136;

    json j = {
                {"c",c},
                {"a",a},
                {"d",param_bool},
                {"n",n}
    };
    cout << "j=   " << j.dump() << endl;
    return j;
}

json ar5::resumeMove()
{
    this->c = "robot";
    this->a = "resumeMove";
    this->param_bool = true;
    this->n = 1141;

    json j = {
                {"c",c},
                {"a",a},
                {"d",param_bool},
                {"n",n}
    };
    cout << "j=   " << j.dump() << endl;
    return j;
}
json ar5::abortMove()
{
    this->c = "robot";
    this->a = "abortMove";
    // this->param_bool=true;
    this->n = 1001;

    json j = {
                {"c",c},
                {"a",a},
                // {"d",param_bool},
                {"n",n}
    };
    cout << "j=   " << j.dump() << endl;
    return j;
}

json ar5::isMoveEnd()
{
    this->c = "robot";
    this->a = "isMoveEnd";
    this->param_int = 0;
    this->n = 145954;

    json j = {
                {"c",c},
                {"a",a},
                {"d",param_int},
                {"n",n}
    };
    cout << "j=   " << j.dump() << endl;
    return j;
}

json ar5::startCartFeedTrans(float dir, float delta)
{
    this->c = "robot";
    this->a = "startCartFeedTrans";
    this->param_json = { 0,dir,delta };
    this->n = 1178;

    json j = {
                {"c",c},
                {"a",a},
                {"d",param_json},
                {"n",n}
    };
    cout << "j=   " << j.dump() << endl;
    return j;

}

json ar5::startJointFeedRot(int jointNo, float delta)
{
    this->c = "robot";
    this->a = "startJointFeedRot";
    this->param_json = { jointNo,delta,0 };
    this->n = 1198;

    json j = {
                {"c",c},
                {"a",a},
                {"d",param_json},
                {"n",n}
    };
    cout << "j=   " << j.dump() << endl;
    return j;
}

json ar5::startCartFeedRot(float dir, float delta)
{
    this->c = "robot";
    this->a = "startCartFeedRot";
    this->param_json = { 0,dir,delta };
    this->n = 1198;

    json j = {
                {"c",c},
                {"a",a},
                {"d",param_json},
                {"n",n}
    };
    cout << "j=   " << j.dump() << endl;
    return j;
}

json ar5::servoJ(const double para[6],double intervalSecond,bool bSync)
{
    json servoJPose={

    };
    this->c = "robot";
    this->a = "servoJ";
    this->param_json={
            {"equip",0},
            {"bSync",bSync},
            {"interval",intervalSecond},
            {"geom",{para[0],para[1],para[2],
                     para[3],para[4],para[5],
                     para[0],para[1],para[2],
                     para[3],para[4],para[5]}
            }
    };
    this->n = 1126;

    json j = {
            {"c",c},
            {"a",a},
            {"d",param_json},
            {"n",n}
    };
    cout << "j=   " << j.dump() << endl;
    return j;
}