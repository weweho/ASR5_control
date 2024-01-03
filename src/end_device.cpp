//
// Created by aung on 2023/12/8.
//

#define DO_NOTHING  0
#define INIT_DEVICE 1
#define INSERT      2
#define TWIST       3
#define TEST        4

#define DPS2SPEED_COMMAND       0.01    // 0.01 dps/LSB
#define DPS2ANGLE_COMMAND       1       // 1 dps/LSB
#define DEGREE2ANGLE_COMMAND    0.01    // 0.01 degree/LSB

#include "end_effector.h"
#include "end_sensor.h"

int motor_nz = 0;
int motor_tc = 1;
int last_state{};

void controlEndDevice(end_effector::endEffector *end_effector,end_sensor::endSensor *end_sensor, int *state)
{
    if(*state!=last_state)
    {
        ROS_INFO("NOW STATE:%d\r\n",*state);
        last_state=*state;
    }

    switch(*state)
    {
        case INIT_DEVICE:
        {
            int tc_speed_=30;
            double tc_angle_=-360.0;
            if(end_effector->sendAngleCommand(motor_tc,(uint16_t)(tc_speed_/DPS2ANGLE_COMMAND),(int32_t)(tc_angle_/DEGREE2ANGLE_COMMAND)))
            {
                *state = INSERT;
            }
        }
        break;
        case INSERT:
        {
            int tc_speed_=-30;
            double value_;
            end_sensor->getSensorData(&value_);
            if(value_>1.0)
            {
                end_effector->sendSpeedCommand(motor_tc,0);
                *state = TWIST;
            }
            else
                end_effector->sendSpeedCommand(motor_tc,(int32_t)(tc_speed_/DPS2SPEED_COMMAND));
        }
        break;
        case TWIST:
        {
            int nz_speed_=20;
            double nz_angle_=360.0;
            int nz_times_=5;
            for(int i = 0 ; i< nz_times_*2 ; i++)
            {
                if(end_effector->sendAngleCommand(motor_nz,(uint16_t)(nz_speed_/DPS2ANGLE_COMMAND),(int32_t)(nz_angle_/DEGREE2ANGLE_COMMAND)))
                    nz_angle_=nz_angle_*-1;
            }
            *state = DO_NOTHING;
        }
        break;
        case TEST:
        {
            if(end_sensor->send_string("1 1 45\n"))
                *state = DO_NOTHING;
        }
        break;
        default:
        {
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