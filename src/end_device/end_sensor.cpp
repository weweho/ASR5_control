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

    void endSensor::storeSensorData(std::deque<SensorData>& buffer, const SensorData& newData, size_t maxSize) {
        if (buffer.size() >= maxSize) {
            buffer.pop_front();
        }
        buffer.push_back(newData);
    }

    int endSensor::detectPressureTrends(int64_t now_motor_angle,size_t max_size ,int threshold,int64_t *return_motor_angle)
    {
        double pressure_{};
        int rising_cumulative_count_{},dropping_cumulative_count_{};
        int return_state_{};
        if(getSensorData(&pressure_))
        {
            SensorData new_data_{};
            new_data_.angle=now_motor_angle;
            new_data_.value=pressure_;
            storeSensorData(sensor_data_buffer, new_data_, max_size);
            for (const auto& data : sensor_data_buffer)
            {
                std::cout << "(" << data.angle << ", " << data.value << ") ";
                if(data.value>last_value&&last_value>second_last_value)         //Rising
                {
                    rising_cumulative_count_++;
                    dropping_cumulative_count_=0;
                }
                else if(data.value<last_value&&last_value<second_last_value)    //Constant
                {
                    dropping_cumulative_count_++;
                    rising_cumulative_count_=0;
                }
                else if(data.value==last_value&&last_value==second_last_value)  //Dropping
                {
                    dropping_cumulative_count_=0;
                    rising_cumulative_count_=0;
                }
                second_last_value=last_value;
                last_value=data.value;
            }
            std::cout << std::endl;
            if(rising_cumulative_count_>=threshold)
            {
                *return_motor_angle=sensor_data_buffer[0].angle;
                return_state_= 1;
            }
            else if(dropping_cumulative_count_>=threshold)
            {
                *return_motor_angle=sensor_data_buffer[max_size-1].angle;
                return_state_= 2;
            }
            else
                return_state_=0;
        }
        return return_state_;
    }
}