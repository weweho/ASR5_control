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
    uint8_t end_sensor_send[8]{};
    uint8_t end_sensor_rec[9]{};

    end_sensor_send[0]=0X01;
    end_sensor_send[1]=0X03;
    end_sensor_send[2]=0X00;
    end_sensor_send[3]=0X50;
    end_sensor_send[4]=0X00;
    end_sensor_send[5]=0X02;
    end_sensor_send[6]=0XC4;
    end_sensor_send[7]=0X1A;

    uint8_t pushrod_send[3]{};
    pushrod_send[0]=1;
    pushrod_send[1]=1;
    pushrod_send[2]=45;

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
            sp.read(end_sensor_rec,sp.available());
            double value = (end_sensor_rec[3]<<24|end_sensor_rec[4]<<16|end_sensor_rec[5]<<8|end_sensor_rec[6])*0.01;
            ROS_INFO("rec_data :%f",value );
        }
        sp.write(end_sensor_send,8);
        sp.write(pushrod_send,3);
        loop_rate.sleep();
    }

    sp.close();

    return 0;
}