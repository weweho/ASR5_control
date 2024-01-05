//
// Created by aung on 2023/12/8.
//

#define DO_NOTHING  0
#define INIT_DEVICE 1
#define INSERT      2
#define TWIST       3
#define TEST        4
#define KEY_INPUT   5
#define DEVICE_TEST 6
#define SENSOR_TEST 7

#define DPS2SPEED_COMMAND       0.01    // 0.01 dps/LSB
#define DPS2ANGLE_COMMAND       1       // 1 dps/LSB
#define DEGREE2ANGLE_COMMAND    0.01    // 0.01 degree/LSB

#include "end_effector.h"
#include "end_sensor.h"
#include "file_operator.h"
#include "std_msgs/Float64.h"

int motor_nz = 0;
int motor_tc = 1;
int last_state{};

void controlEndDevice(end_effector::endEffector *end_effector,end_sensor::endSensor *end_sensor, int *state)
{
    if(*state!=last_state)
    {
        switch(*state)
        {
            case 1:
                ROS_INFO("INIT_DEVICE");
                break;
            case 2:
                ROS_INFO("INSERT");
                break;
            case 3:
                ROS_INFO("TWIST");
                break;
            case 4:
                ROS_INFO("TEST");
                break;
            case 5:
                ROS_INFO("KEY_INPUT");
                break;
            default:
                ROS_INFO("DO_NOTHING");
                break;
        }
        last_state=*state;
    }

    switch(*state)
    {
        case INIT_DEVICE:
        {
            int tc_speed_=30;
            double tc_angle_=360.0;
            if(end_sensor->send_string("1 1 45\n")
            &&end_effector->sendAngleCommand(motor_tc,(uint16_t)(abs(tc_speed_)/DPS2ANGLE_COMMAND),(int32_t)(tc_angle_/DEGREE2ANGLE_COMMAND)))
                *state = INSERT;
        }
        break;
        case INSERT:
        {
            int tc_speed_=-30;
            int tc_angle_=-120;
            double max_force_=1.0;
            double min_force_=0.5;
            double interval_second_=0.2;
            if(end_sensor->insertDetect(max_force_,min_force_,interval_second_))
            {
                if(end_effector->sendSpeedCommand(motor_tc,0))
                {
                    usleep(1);
                    if(end_effector->sendAngleCommand(motor_tc,(uint16_t)(abs(tc_speed_)/DPS2ANGLE_COMMAND),(int32_t)(tc_angle_/DEGREE2ANGLE_COMMAND)))
                        *state = TWIST;
                }
            }
            else
                end_effector->sendSpeedCommand(motor_tc,(int32_t)(tc_speed_/DPS2SPEED_COMMAND));
        }
        break;
        case TWIST:
        {
            MOTOR_DATA motor_data_{};
            end_effector->readMotorData(motor_tc,&motor_data_);
            usleep(4);
            end_effector->readMotorData(motor_tc,&motor_data_);
            int nz_speed_=20;
            double nz_angle_=60.0;
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
            double max_force_=1.0;
            double min_force_=0.5;
            double interval_second_=0.2;
            if(end_sensor->insertDetect(max_force_,min_force_,interval_second_))
                *state=DO_NOTHING;
        }
        break;
        case KEY_INPUT:
        {
            int insert_angle_;
            std::cout<<"请输入要刺入的角度:";
            std::cin>>insert_angle_;
            std::cout<<"你输入的角度是："<<insert_angle_<<std::endl;
            *state=DO_NOTHING;
        }
        break;

        case DEVICE_TEST:
        {
            int tc_speed_=45;
            double tc_angle_=45.0;
            int nz_speed_=45;
            double nz_angle_=45.0;
            if(end_sensor->send_string("1 1 45\n"))
            {
                ROS_INFO("Test putter\r\n");
                usleep(5);
                if(end_effector->sendAngleCommand(motor_tc,(uint16_t)(tc_speed_/DPS2ANGLE_COMMAND),(int32_t)(tc_angle_/DEGREE2ANGLE_COMMAND)))
                {
                    ROS_INFO("Test motor_tc 1\r\n");
                    usleep(1);
                    if(end_effector->sendAngleCommand(motor_nz,(uint16_t)(tc_speed_/DPS2ANGLE_COMMAND),(int32_t)(-tc_angle_/DEGREE2ANGLE_COMMAND)))
                    {
                        ROS_INFO("Test motor_tc 2\r\n");
                        usleep(1);
                        if(end_effector->sendAngleCommand(motor_nz,(uint16_t)(nz_speed_/DPS2ANGLE_COMMAND),(int32_t)(nz_angle_/DEGREE2ANGLE_COMMAND)))
                        {
                            ROS_INFO("Test motor_nz 1\r\n");
                            usleep(1);
                            if(end_effector->sendAngleCommand(motor_nz,(uint16_t)(nz_speed_/DPS2ANGLE_COMMAND),(int32_t)(-nz_angle_/DEGREE2ANGLE_COMMAND)))
                            {
                                ROS_INFO("Test motor_nz 2\r\n");
                                *state=SENSOR_TEST;
                            }
                        }
                    }
                }
            }
        }
        break;
        case SENSOR_TEST:
        {
            MOTOR_DATA motor_tc_data_{},motor_nz_data_{};
            double sensor_value_{};
            end_effector->readMotorData(motor_tc,&motor_tc_data_);
            end_effector->readMotorData(motor_nz,&motor_nz_data_);
            end_sensor->getSensorData(&sensor_value_);
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
    file_operator::fileOperator file_operator;
    ros::Rate r(20);
    std::vector<std::string> string;
    file_operator.writeToTXT(10.0,1.0,1.0);
    file_operator.readToTXT(&string);
    end_sensor.initUSB0();
    end_effector.initCAN1();
    int state_=DEVICE_TEST;
    std_msgs::Float64 sensor_value_;
    ros::Publisher value_pub = nh.advertise<std_msgs::Float64>("/sensor_value", 1000);
    while(ros::ok()&&end_effector.CAN1isOpen()&&end_sensor.USB0isOpen())
    {
        end_sensor.getSensorData(&sensor_value_.data);
        value_pub.publish(sensor_value_);
        controlEndDevice(&end_effector,&end_sensor, &state_);
        r.sleep();
    }
    return 0;
}