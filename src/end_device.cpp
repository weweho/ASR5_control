//
// Created by aung on 2023/12/8.
//

#define DO_NOTHING  0
#define INIT_DEVICE 1
#define INSERT      2
#define TWIST       3
#define TEST        4

#include "end_effector.h"
#include "end_sensor.h"

int motor_nz = 0;
int motor_tc = 1;

void controlEndDevice(end_effector::endEffector *end_effector,end_sensor::endSensor *end_sensor, int *state)
{
    switch(*state)
    {
        case INIT_DEVICE:
        {
            ROS_INFO("INIT_DEVICE\r\n");
            int tc_speed_{};
            int tc_angle_{};
            if(end_sensor->send_string("1 1 45\n")&&end_effector->sendAngleCommand(motor_tc,tc_speed_,tc_angle_))
            {
                *state = INSERT;
            }
        }
        break;
        case INSERT:
        {
            ROS_INFO("INSERT\r\n");
            int tc_speed_{};
            double value_;
            end_sensor->getSensorData(&value_);
            if(value_>1.0)
            {
                end_effector->sendSpeedCommand(motor_tc,0);
                *state = TWIST;
            }
            else
                end_effector->sendSpeedCommand(motor_tc,tc_speed_);
        }
        break;
        case TWIST:
        {
            ROS_INFO("TWIST\r\n");
            int nz_speed_{};
            int nz_angle_{};
            int nz_times_=5;
            for(int i = 0 ; i< nz_times_*2 ; i++)
            {
                if(end_effector->sendAngleCommand(motor_nz,nz_speed_,nz_angle_))
                    nz_angle_=nz_angle_*-1;
            }
            *state = DO_NOTHING;
        }
        break;
        case TEST:
        {
            ROS_INFO("TEST\r\n");
            if(end_sensor->send_string("1 1 45\n"))
                *state = DO_NOTHING;
        }
        break;
        default:
        {
            ROS_INFO("DO_NOTHING\r\n");
        }
        break;
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv,"end_device");
    ros::NodeHandle nh;
    end_effector::endEffector end_effector;
    end_sensor::endSensor end_sensor;
    ros::Rate r(50);
    end_sensor.initUSB0();
    end_effector.initCAN1();
    int state=TEST;
    while(ros::ok()&&end_effector.CAN1isOpen()&&end_sensor.USB0isOpen())
//    while(ros::ok()&&end_effector.CAN1isOpen())
//    while(ros::ok()&&end_sensor.USB0isOpen())
//    while(ros::ok())
    {
        controlEndDevice(&end_effector,&end_sensor, &state);
        r.sleep();
    }
    return 0;
}