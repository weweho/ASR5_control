//
// Created by aung on 2024/1/7.
//

#include "fsm.h"

namespace fsm
{
    FSM::FSM()
    {
        last_motor_test_state=PULL;
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

    void FSM::testAllDevice(end_effector::endEffector *end_effector,end_sensor::endSensor *end_sensor, end_putter::endPutter *end_putter,
                            int *state,int motor_nz,int motor_tc)
    {
        infoState(state);
        switch(*state)
        {
            case DEVICE_TEST:
            {
                int tc_speed_=150;
                double tc_angle_=300.0;
                int nz_speed_=150;
                double nz_angle_=300.0;
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
//        infoState(state);
        switch(*state)
        {
            case WAIT:
            {
                if(end_effector->sendSpeedCommand(motor_ip,0))
                {
                    ROS_INFO("三秒后开始自动插拔\r\n");
                    ROS_INFO("3\r\n");
                    sleep(1);
                    ROS_INFO("2\r\n");
                    sleep(1);
                    ROS_INFO("1\r\n");
                    sleep(1);
                    ROS_INFO("程序开始\r\n");
                    *state=KEY_INPUT;
                }
            }
            break;
            case INSERT:
            {
                if(end_effector->sendAngleCommand(motor_ip,(uint16_t)(dps/DPS2ANGLE_COMMAND),(int32_t)(-encoder_data)))
                {
                    sleep(duration);
                    *state=KEY_INPUT;
                }
            }
            break;
            case PULL:
            {
                if(end_effector->sendAngleCommand(motor_ip,(uint16_t)(dps/DPS2ANGLE_COMMAND),(int32_t)(encoder_data)))
                {
                    sleep(duration);
                    *state=KEY_INPUT;
                }
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
//                        ROS_INFO("当前角度 :%ld ; 上一次的角度: %ld \r\n",now_angle_,last_angle);
                        if(state_==PULL) //此时逆时针转，对于motor_angle来说是减小（说明书上说的）
                        {
                            diff_angle_[0]=8.0;
                            diff_angle_[1]=((last_angle-now_angle_)*0.01);
                            ROS_INFO("下压时的角度差值：%f",diff_angle_[1]);
                        }
                        else //此时顺时针转，对于motor_angle来说是增加
                        {
                            diff_angle_[0]=2.0;
                            diff_angle_[1]=((now_angle_-last_angle)*0.01);
                            ROS_INFO("上拉时的角度差值：%f",diff_angle_[1]);
                        }
                        file_operator->writeToExcel<double>(diff_angle_,2,"encoder.csv");
                        last_angle=now_angle_;
                        last_motor_test_state=state_;
                        *state=state_;
                    }
                }
                else
                    ROS_INFO("没收到电机回复，再发送一次");
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
        if(end_sensor->getSensorData(&sensor_data[1]))
        {
            sensor_data[0]+=1.0/freq;
            file_operator->writeToExcel<double>(sensor_data,2,"force.csv");
        }
    }

    void FSM::testInsertDirect(end_effector::endEffector *end_effector,end_sensor::endSensor *end_sensor,int *insert_state,int motor_ip,int tc_speed,double skin_thickness)
    {
        switch(*insert_state)
        {
            case WAIT:
            {
                if(end_effector->sendSpeedCommand(motor_ip,0))
                {
                    sleep(3);
                    *insert_state=START_MOVE;
                }
            }
            break;
            case START_MOVE:
            {
                if(end_effector->sendSpeedCommand(motor_ip,(tc_speed/DPS2SPEED_COMMAND)))
                    *insert_state=RISING_DETECT;
            }
            break;
            case RISING_DETECT:
            {
                int64_t now_motor_angle_{};
                size_t rising_deque_max_size_=5;
                int rising_threshold_=3;
                if(end_effector->readMotorAngle(motor_ip,&now_motor_angle_))
                {
                    if(end_sensor->detectPressureTrends(now_motor_angle_,rising_deque_max_size_,rising_threshold_,&rising_angle)==1)
                    {
                        ROS_INFO("检测到压力值上升!上升开始时角度值：%ld\r\n",rising_angle);
                        detected_rising_time=ros::Time().now().toSec();
                        *insert_state=DROPPING_DETECT;
                    }
                }
            }
            break;
            case DROPPING_DETECT:
            {
                int64_t now_motor_angle_{};
                size_t dropping_deque_max_size_=3;
                int dropping_threshold_=2;
                double duration_after_rise_=skin_thickness/( abs(tc_speed)*( 28.5/360.0) );
                if( ros::Time().now().toSec()-detected_rising_time>duration_after_rise_)
                {
                    ROS_INFO("理论上已经刺破皮肤，已停止\r\n");
                    detect_dropping=false;
                    *insert_state=DETECT_FINISH;
                }
                else if(end_effector->readMotorAngle(motor_ip,&now_motor_angle_))
                {
                    if(end_sensor->detectPressureTrends(now_motor_angle_,dropping_deque_max_size_,dropping_threshold_,&dropping_angle)==2)
                    {
                        ROS_INFO("检测到压力值下降! 下降开始时角度值：%ld\r\n",dropping_angle);
                        detect_dropping=true;
                        *insert_state=DETECT_FINISH;
                    }
                }
            }
            break;
            case DETECT_FINISH:
            {
                if(end_effector->sendSpeedCommand(motor_ip,0))
                {
                    int64_t now_angle_;
                    if(end_effector->readMotorAngle(motor_ip,&now_angle_))
                    {
                        ROS_INFO("刺穿表皮，电机停止！皮的厚度：%f",detect_dropping?(dropping_angle-rising_angle):skin_thickness);
                        dropping_angle=0;
                        rising_angle=0;
                        *insert_state=CURVED_DETECT;
                    }
                }
            }
            break;
            case CURVED_DETECT:
            {
                double now_pressure_;
                if(end_sensor->getSensorData(&now_pressure_))
                {
                    if(now_pressure_>=0.85)
                        ROS_INFO("弯针了！\r\n");
                    else
                        ROS_INFO("顺利刺入！\r\n");
                }
                *insert_state=DO_NOTHING;
            }
        }
    }

}//fsm