//
// Created by aung on 2023/12/9.
//
#include <ros/ros.h>
#include <serial/serial.h>
#include <iostream>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "serial_port");
    ros::NodeHandle node;

    serial::Serial sp;
    serial::Timeout to = serial::Timeout::simpleTimeout(100);
    sp.setPort("/dev/ttyUSB0");
    sp.setBaudrate(115200);
    sp.setTimeout(to);
    uint8_t send_data[8]{};
    uint8_t rec_data[9]{};

    try
    {
        sp.open();
    }
    catch(serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port.");
        return false;
    }

    if(sp.isOpen())
    {
        ROS_INFO_STREAM("/dev/ttyUSB0 is opened.");
    }
    else
    {
        return false;
    }

    ros::Rate loop_rate(50);
    while(ros::ok()&sp.isOpen())
    {
        if (sp.available())
        {
            sp.read(rec_data,sp.available());
            for(int i = 0;i<sizeof(rec_data); i++)
            {
                printf("%02X ",rec_data[i]);
            }
            printf("\n");
            double value = (rec_data[3]<<24|rec_data[4]<<16|rec_data[5]<<8|rec_data[6])*0.01;
            ROS_INFO("rec_data :%f",value );
        }

        send_data[0]=0X01;
        send_data[1]=0X03;
        send_data[2]=0X00;
        send_data[3]=0X50;
        send_data[4]=0X00;
        send_data[5]=0X02;
        send_data[6]=0XC4;
        send_data[7]=0X1A;
        sp.write(send_data,8);
        loop_rate.sleep();
    }

    sp.close();

    return 0;
}