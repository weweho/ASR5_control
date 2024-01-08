//
// Created by aung on 2024/1/7.
//

#include "fsm.h"

namespace fsm
{
    FSM::FSM()
    {
        last_motor_test_state=INSERT;
    }

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
                case 8:
                    ROS_INFO("PULL");
                    break;
                case 9:
                    ROS_INFO("READ_MOTOR");
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

    void FSM::testMotorAccuracy(end_effector::endEffector *end_effector,file_operator::fileOperator *file_operator,int *state,int motor_ip , int encoder_data ,int dps ,int duration)
    {
        infoState(state);
        switch(*state)
        {
            case INSERT:
                if(end_effector->sendAngleCommand(motor_ip,(uint16_t)(dps/DPS2ANGLE_COMMAND),(int32_t)(-encoder_data)))
                {
                    sleep(duration);
                    *state=KEY_INPUT;
                }
            break;
            case PULL:
                if(end_effector->sendAngleCommand(motor_ip,(uint16_t)(dps/DPS2ANGLE_COMMAND),(int32_t)(encoder_data)))
                {
                    sleep(duration);
                    *state=KEY_INPUT;
                }
            break;
            case KEY_INPUT:
            {
                int state_;
                if(last_motor_test_state==PULL)
                    state_=INSERT;
                else
                    state_=PULL;
                int64_t now_angle_{};
                double diff_angle_[2]{};
                usleep(100);
                if(end_effector->readMotorAngle(motor_ip,&now_angle_))
                {
                    if(state_==PULL|state_==INSERT)
                    {
                        ROS_INFO("当前角度 :%ld ; 上一次的角度: %ld \r\n",now_angle_,last_angle);
                        if(state_==PULL) //此时逆时针转，对于motor_angle来说是减小（说明书上说的）
                        {
                            diff_angle_[0]=8.0;
                            diff_angle_[1]=(double)((last_angle-now_angle_)*0.01);
                            ROS_INFO("下压时的角度差值：%f",diff_angle_[1]);
                        }
                        else //此时顺时针转，对于motor_angle来说是增加
                        {
                            diff_angle_[0]=2.0;
                            diff_angle_[1]=(double)((now_angle_-last_angle)*0.01);
                            ROS_INFO("上拉时的角度差值：%f",diff_angle_[1]);
                        }
                        file_operator->writeToExcel<double>(diff_angle_,2,"encoder.csv");
                        last_angle=now_angle_;
                        last_motor_test_state=state_;
                        *state=state_;
                    }
                    else
                    {
                        *state=DO_NOTHING;
                        ROS_INFO("不知道你按了啥，我先退了");
                    }
                }
                else
                    ROS_INFO("没收到电机回复，再按一次");
            }
            break;
            default:
            {
            }
            break;
        }
    }

    void FSM::testSensorData(end_sensor::endSensor *end_sensor, file_operator::fileOperator *file_operator, int freq)
    {
        double sensor_data_[2]{};
        if(end_sensor->getSensorData(&sensor_data_[1]))
        {
            sensor_data_[0]+=1.0/freq;
            file_operator->writeToExcel<double>(sensor_data_,2,"force.csv");
        }
    }


}//fsm