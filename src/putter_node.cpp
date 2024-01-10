//
// Created by aung on 2024/1/8.
//
#include "fsm.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv,"putter_node");
    ros::NodeHandle nh;
    ros::Rate r(20);
    setlocale(LC_ALL, "");
    setlocale(LC_CTYPE, "zh_CN.utf8");

    end_putter::endPutter end_putter;
    file_operator::fileOperator file_operator;

    end_putter.initPutter();

    while(ros::ok()&&end_putter.putterisOpen())
    {
        if(end_putter.send_string("1 1 1\n"))
            break;
        r.sleep();
    }
    return 0;
}