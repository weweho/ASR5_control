//
// Created by aung on 2024/1/7.
//

#include "fsm.h"

namespace fsm
{
    void FSM::infoState(const int *state)
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
                case 6:
                    ROS_INFO("DEVICE_TEST");
                    break;
                case 7:
                    ROS_INFO("SENSOR_TEST");
                    break;
                default:
                    ROS_INFO("DO_NOTHING");
                    break;
            }
            last_state=*state;
        }
    }

    void FSM::controlEndDevice(end_effector::endEffector *end_effector, end_sensor::endSensor *end_sensor,
                          end_putter::endPutter *end_putter, file_operator::fileOperator *file_operator,
                          int *state,int motor_nz,int motor_tc)
    {
        infoState(state);
        switch(*state)
        {
            case INIT_DEVICE:
            {
                int tc_speed_=30;
                double tc_angle_=360.0;
                if(end_putter->send_string("1 1 45\n")
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
            default:
            {
            }
            break;
        }
    }

    void FSM::testEndDevice(end_effector::endEffector *end_effector,end_sensor::endSensor *end_sensor, end_putter::endPutter *end_putter,
                               int *state,int motor_nz,int motor_tc)
    {
        infoState(state);
        switch(*state)
        {
            case DEVICE_TEST:
            {
                int tc_speed_=80;
                double tc_angle_=120.0;
                int nz_speed_=80;
                double nz_angle_=120.0;
                if(end_putter->send_string("1 1 45\n"))
                {
                    ROS_INFO("Test putter\r\n");
                    sleep(10);
                    if(end_effector->sendAngleCommand(motor_tc,(uint16_t)(tc_speed_/DPS2ANGLE_COMMAND),(int32_t)(tc_angle_/DEGREE2ANGLE_COMMAND)))
                    {
                        ROS_INFO("Test motor_tc 1\r\n");
                        sleep(2);
                        if(end_effector->sendAngleCommand(motor_tc,(uint16_t)(tc_speed_/DPS2ANGLE_COMMAND),(int32_t)(-tc_angle_/DEGREE2ANGLE_COMMAND)))
                        {
                            ROS_INFO("Test motor_tc 2\r\n");
                            sleep(2);
                            if(end_effector->sendAngleCommand(motor_nz,(uint16_t)(nz_speed_/DPS2ANGLE_COMMAND),(int32_t)(nz_angle_/DEGREE2ANGLE_COMMAND)))
                            {
                                ROS_INFO("Test motor_nz 1\r\n");
                                sleep(2);
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
                double sensor_value_{};
                end_sensor->getSensorData(&sensor_value_);
            }
            break;

            case TEST:
            {
                end_putter->send_string("1 2 45\n");
                end_putter->readESPData();
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

            default:
            {
            }
            break;
        }
    }

}