//
// Created by aung on 2023/12/9.
//
#include <ros/ros.h>
#include <serial/serial.h>
#include <iostream>
#include "end_sensor.h"
#include <QByteArray>
#include <QString>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "serial_port");
    ros::NodeHandle node;

    serial::Serial sp;
    serial::Timeout to = serial::Timeout::simpleTimeout(100);
    sp.setPort("/dev/ttyUSB0");
    sp.setBaudrate(115200);
    sp.setTimeout(to);

    try
    {
        sp.open();
    }
    catch(serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port.");
        return -1;
    }

    if(sp.isOpen())
    {
        ROS_INFO_STREAM("/dev/ttyUSB0 is opened.");
    }
    else
    {
        return -1;
    }

    ros::Rate loop_rate(500);
    while(ros::ok())
    {
        if (sp.available())
        {
            std::string dataStr = sp.read(sp.available());
            QByteArray data = QByteArray::fromStdString(dataStr);
            QString hexStr=data.mid(3, 4).toHex();
            bool conversionOK;
            auto intForceValue = static_cast<qint16>(hexStr.toUInt(&conversionOK, 16));
            if (!conversionOK) {
                ROS_ERROR_STREAM("Conversion fail .");
                return -1;
            }
            if (intForceValue > 0x7FFF) {
                intForceValue -= 0x10000;
            }
            double doubleForceValue = static_cast<double>(intForceValue)*0.01;
            ROS_INFO("end_sensor value: %f \r\n",doubleForceValue);
        }
        sp.write("010300500002C41A");
        loop_rate.sleep();
    }

    sp.close();

    return 0;
}