//
// Created by aung on 2024/1/1.
//

#include "end_sensor.h"
namespace end_sensor
{
    endSensor::endSensor()
    {
        to = serial::Timeout::simpleTimeout(100);
        sp.setPort("/dev/ttyUSB0");
        sp.setBaudrate(115200);
        sp.setTimeout(to);
        end_sensor_send[0]=0X01;
        end_sensor_send[1]=0X03;
        end_sensor_send[2]=0X00;
        end_sensor_send[3]=0X50;
        end_sensor_send[4]=0X00;
        end_sensor_send[5]=0X02;
        end_sensor_send[6]=0XC4;
        end_sensor_send[7]=0X1A;
    }

    bool endSensor::initUSB0()
    {
        try
        {
            sp.open();
        }
        catch(serial::IOException& e)
        {
            ROS_ERROR_STREAM("Unable to open port.");
            return false;
        }
        return true;
    }

    bool endSensor::USB0isOpen()
    {
        if(sp.isOpen())
        {
            ROS_INFO_ONCE("/dev/ttyUSB0 is opened.");
            return true;
        }
        else
        {
            ROS_INFO_ONCE("/dev/ttyUSB0 open fail.");
            return false;
        }
    }

    bool endSensor::getSensorData(double *value)
    {
        sp.write(end_sensor_send,8);
        if (sp.isOpen()&&sp.available())
        {
            usleep(100); //0.1ms
            sp.read(end_sensor_rec,sp.available());
            *value = (end_sensor_rec[3]<<24|end_sensor_rec[4]<<16|end_sensor_rec[5]<<8|end_sensor_rec[6])*0.01;
            ROS_INFO("rec_data :%f",*value );
            return true;
        }
        return false;
    }

    void endSensor::send_string(const char *str)
    {
        if(sp.isOpen())
            sp.write(str);
    }

    bool endSensor::readESPData()
    {
        if (sp.isOpen()&&sp.available())
        {
            std::string str_;
            int num{};
            sp.read(esp_rec,sp.available());
            for(uint8_t i : esp_rec)
            {
                str_[num]=i;
                if(i=='\n')
                {
                    printf("%s",str_.c_str());
                    num=0;
                }
            }
            return true;
        }
        return false;
    }

}