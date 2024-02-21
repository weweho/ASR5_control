//
// Created by aung on 2024/1/7.
//

#include "fsm.h"

namespace fsm
{
    FSM::FSM()
    {
        curved_threshold=1.3;
        tc_coefficient=6.0;
        nz_coefficient=6.0;
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

    void FSM::testMotorAccuracy(end_effector::endEffector *end_effector,file_operator::fileOperator *file_operator,int *state,int motor_ip , int encoder_data ,int dps ,double duration)
    {
        infoState(state);
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
                    usleep(duration*1000000);
                    *state=KEY_INPUT;
                }
            }
            break;
            case PULL:
            {
                if(end_effector->sendAngleCommand(motor_ip,(uint16_t)(dps/DPS2ANGLE_COMMAND),(int32_t)(encoder_data)))
                {
                    usleep(duration*1000000);
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
                        ROS_INFO("当前角度 :%ld ; 上一次的角度: %ld \r\n",now_angle_,last_angle);
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
            case INSERT_WAIT:
            {
                if(end_effector->sendSpeedCommand(motor_ip,0))
                {
                    ROS_INFO("3\r\n");
                    sleep(1);
                    ROS_INFO("2\r\n");
                    sleep(1);
                    ROS_INFO("1\r\n");
                    sleep(1);
                    ROS_INFO("程序开始\r\n");
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
                    if(end_sensor->detectPressureTrends(now_motor_angle_,rising_deque_max_size_,rising_threshold_,rising_angle,rising_value,&rising_average_value)==RISING)
                    {
                        ROS_INFO("检测到压力值上升!上升开始时角度值：%ld\r\n",rising_angle[0]);
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
                if(ros::Time().now().toSec()-detected_rising_time>duration_after_rise_)
                {
                    ROS_INFO("理论上已经刺破皮肤，已停止\r\n");
                    detect_dropping=false;
                    *insert_state=CURVED_DETECT;
                }
                else if(end_effector->readMotorAngle(motor_ip,&now_motor_angle_))
                {
                    if(end_sensor->detectPressureTrends(now_motor_angle_,dropping_deque_max_size_,dropping_threshold_,dropping_angle,dropping_value,&dropping_average_value)==DROPPING)
                    {
                        ROS_INFO("检测到压力值下降! 下降开始时角度值：%ld\r\n",dropping_angle[0]);
                        *insert_state=CURVED_DETECT;
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
                        ROS_INFO("刺穿表皮，电机停止！去皮后要补偿的角度值：%ld\r\n",detect_dropping?(dropping_angle[0]-now_angle_):(-now_angle_-(int64_t)(skin_thickness/( 28.5/360.0))+rising_angle[0]));
                        *insert_state=DO_NOTHING;
                    }
                }
            }
            break;
            case CURVED_DETECT:
            {
                if(end_effector->sendSpeedCommand(motor_ip,0))
                {
                    if(dropping_value[1]>curved_threshold)
                    {
                        ROS_INFO("发生弯针！！\r\n");
                        *insert_state=DO_NOTHING;
                    }
                    else
                    {
                        detect_dropping=true;
                        *insert_state=DETECT_FINISH;
                    }
                }
            }
            break;
        }
    }

    void FSM::semiCompleteProcess(end_effector::endEffector *end_effector,end_sensor::endSensor *end_sensor,
                              int *state,int motor_tc,int motor_nz,
                              uint16_t insert_speed,double insert_depth,double skin_thickness,
                              double tc_depth,int tc_times_per_min,int tc_times,
                              double nz_angle,int nz_times_per_min,int nz_times)
    {
        switch(*state)
        {
            case KEY_INPUT:
            {
                int64_t motor_tc_initial_angle_=420;
                int32_t motor_tc_initial_speed_=150;
                if(end_effector->sendSpeedCommand(motor_tc,0))
                {
                    std::cout<<"按任意键开始初始化"<<std::endl;
                    std::cin>>putter_target_angle;
                    if(end_effector->sendAngleCommand(motor_tc,(uint16_t)(motor_tc_initial_speed_/DPS2ANGLE_COMMAND),(int32_t)(motor_tc_initial_angle_/DEGREE2ANGLE_COMMAND)))
                    {
                        sleep(3);
                        *state=INIT_DEVICE;
                    }
                }
            }
                break;
            case INIT_DEVICE:
            {
                if(end_effector->sendSpeedCommand(motor_tc,0))
                {
                    std::string input_;
                    std::cout<<"按任意键开始针灸"<<std::endl;
                    std::cin>>input_;
                    *state=START_MOVE;
                }
            }
                break;
            case START_MOVE:
            {
                if(end_effector->sendSpeedCommand(motor_tc,-(insert_speed/DPS2SPEED_COMMAND)))
                    *state=RISING_DETECT;
            }
                break;
            case RISING_DETECT:
            {
                int64_t now_motor_angle_{};
                size_t rising_deque_max_size_=5;
                int rising_threshold_=4;
                if(end_effector->readMotorAngle(motor_tc,&now_motor_angle_))
                {
                    if(end_sensor->detectPressureTrends(now_motor_angle_,rising_deque_max_size_,rising_threshold_,rising_angle,rising_value,&rising_average_value)==RISING)
                    {
                        ROS_INFO("检测到压力值上升!上升开始时角度值：%ld\r\n",rising_angle[0]);
                        detected_rising_time=ros::Time().now().toSec();
                        *state=DROPPING_DETECT;
                    }
                }
            }
                break;
            case DROPPING_DETECT:
            {
                int64_t now_motor_angle_{};
                size_t dropping_deque_max_size_=5;
                int dropping_threshold_=3;

                double duration_after_rise_=skin_thickness/( (insert_speed)*( 28.5/360.0) );
                if(ros::Time().now().toSec()-detected_rising_time>duration_after_rise_)
                {
                    ROS_INFO("理论上已经刺破皮肤，已停止\r\n");
                    detect_dropping=false;
                    *state=CURVED_DETECT;
                }
                else if(end_effector->readMotorAngle(motor_tc,&now_motor_angle_))
                {
                    if(end_sensor->detectPressureTrends(now_motor_angle_,dropping_deque_max_size_,dropping_threshold_,dropping_angle,dropping_value,&dropping_average_value)==DROPPING)
                    {
                        ROS_INFO("检测到压力值下降! 下降开始时角度值：%ld\r\n",dropping_angle[0]);
                        *state=CURVED_DETECT;
                    }
                }
            }
                break;
            case CURVED_DETECT:
            {
                if(end_effector->sendSpeedCommand(motor_tc,0))
                {
                    if(dropping_value[1]>curved_threshold)
                    {
                        ROS_INFO("发生弯针！！\r\n");
                        *state=DO_NOTHING;
                    }
                    else
                    {
                        detect_dropping=true;
                        *state=DETECT_FINISH;
                    }
                }
            }
                break;
            case DETECT_FINISH:
            {

                int64_t now_angle_;
                if(end_effector->readMotorAngle(motor_tc,&now_angle_))
                {
                    replenish_angle=(detect_dropping?(dropping_angle[0]-now_angle_)/100.0:(rising_angle[0]-now_angle_-(int64_t)(skin_thickness/( 28.5/360.0)))/100.0);
                    ROS_INFO("刺穿表皮，电机停止！去皮后要补偿的角度值：%ld\r\n",replenish_angle);
                    sleep(1);
                    *state=INSERT_DEEPER;
                }

            }
                break;
            case INSERT_DEEPER:
            {
                double insert_angle_=insert_depth*360.0/28.5-replenish_angle;
                ROS_INFO("理论要继续下压的角度：%f\r\n",insert_angle_);
                if(end_effector->sendAngleCommand(motor_tc,(uint16_t)(insert_speed/DPS2ANGLE_COMMAND),(int32_t)(-insert_angle_/DEGREE2ANGLE_COMMAND)))
                {
                    ROS_INFO("准备开始提插\r\n");
                    ROS_INFO("3\r\n");
                    sleep(1);
                    ROS_INFO("2\r\n");
                    sleep(1);
                    ROS_INFO("1\r\n");
                    sleep(1);
                    last_motor_test_state=INSERT;
                    *state=INSERT;
                }

            }
                break;
            case INSERT:
            {
                ROS_INFO("提\r\n");
                double tc_angle_=tc_depth*360.0/28.5;
                double tc_dps_=tc_angle_*tc_coefficient;
                if(end_effector->sendAngleCommand(motor_tc,(uint16_t)(tc_dps_/DPS2ANGLE_COMMAND),(int32_t)(tc_angle_/DEGREE2ANGLE_COMMAND)))
                {
                    usleep((60*1000000)/(2*tc_times_per_min));
                    *state=SWITCH_TC_STATE;
                }
            }
                break;
            case PULL:
            {
                ROS_INFO("插\r\n");
                double tc_angle_=tc_depth*360.0/28.5;
                double tc_dps_=tc_angle_*tc_coefficient;
                if(end_effector->sendAngleCommand(motor_tc,(uint16_t)(tc_dps_/DPS2ANGLE_COMMAND),(int32_t)(-tc_angle_/DEGREE2ANGLE_COMMAND)))
                {
                    ROS_INFO("等待时间：%d\r\n",60/(2*tc_times_per_min)*1000000);
                    usleep((60*1000000)/(2*tc_times_per_min));
                    tc_accumulate_times++;
                    if(tc_accumulate_times<tc_times)
                        *state=SWITCH_TC_STATE;
                    else
                        *state=INSERT_FINISH;
                }
            }
                break;
            case SWITCH_TC_STATE:
            {
                int state_;
                if(last_motor_test_state==PULL)
                    state_=INSERT;
                else
                    state_=PULL;
                int64_t now_angle_{};
                double diff_angle_[2]{};
                usleep(100);
                if(end_effector->readMotorAngle(motor_tc,&now_angle_))
                {
                    ROS_INFO("当前角度 :%ld ; 上一次的角度: %ld \r\n",now_angle_,last_angle);
                    if(last_motor_test_state==PULL)
                    {
                        diff_angle_[0]=8.0;
                        diff_angle_[1]=((now_angle_-last_angle)*0.01);
                        ROS_INFO("提的角度差值：%f",diff_angle_[1]);
                    }
                    else //此时顺时针转，对于motor_angle来说是增加
                    {
                        diff_angle_[0]=2.0;
                        diff_angle_[1]=((last_angle-now_angle_)*0.01);
                        ROS_INFO("插的角度差值：%f",diff_angle_[1]);
                    }
                    last_angle=now_angle_;
                    last_motor_test_state=state_;
                    *state=state_;
                }
                else
                    ROS_INFO("没收到电机回复，再发送一次");
            }
                break;
            case INSERT_FINISH:
            {
                if(end_effector->sendSpeedCommand(motor_tc,0))
                {
                    ROS_INFO("提插结束，即将开始捻转\r\n");
                    ROS_INFO("3\r\n");
                    sleep(1);
                    ROS_INFO("2\r\n");
                    sleep(1);
                    ROS_INFO("1\r\n");
                    sleep(1);
                    *state=TWIST_BACK;
                }
            }
            break;
            case TWIST:
            {
                double nz_dps_=(nz_angle)*nz_coefficient;
                if(end_effector->sendAngleCommand(motor_nz,(uint16_t)(nz_dps_/DPS2ANGLE_COMMAND),(int32_t)(nz_angle/DEGREE2ANGLE_COMMAND)))
                {
                    usleep((60*1000000)/(2*nz_times_per_min));
                    nz_accumulate_times++;
                    if(nz_accumulate_times<nz_times)
                        *state=SWITCH_NZ_STATE;
                    else
                        *state=TWIST_FINISH;
                }
            }
                break;
            case TWIST_BACK:
            {
                double nz_dps_=(nz_angle)*nz_coefficient;
                if(end_effector->sendAngleCommand(motor_nz,(uint16_t)(nz_dps_/DPS2ANGLE_COMMAND),(int32_t)(-nz_angle/DEGREE2ANGLE_COMMAND)))
                {
                    usleep((60*1000000)/(2*nz_times_per_min));
                    *state=SWITCH_NZ_STATE;
                }
            }
                break;
            case SWITCH_NZ_STATE:
            {
                int state_;
                if(last_motor_test_state==TWIST)
                    state_=TWIST_BACK;
                else
                    state_=TWIST;
                int64_t now_angle_{};
                double diff_angle_[2]{};
                usleep(100);
                if(end_effector->readMotorAngle(motor_nz,&now_angle_))
                {
                    ROS_INFO("当前角度 :%ld ; 上一次的角度: %ld \r\n",now_angle_,last_angle);
                    if(state_==TWIST)
                    {
                        diff_angle_[0]=8.0;
                        diff_angle_[1]=((last_angle-now_angle_)*0.01);
                        ROS_INFO("捻的角度差值：%f",diff_angle_[1]);
                    }
                    else
                    {
                        diff_angle_[0]=2.0;
                        diff_angle_[1]=((now_angle_-last_angle)*0.01);
                        ROS_INFO("转的角度差值：%f",diff_angle_[1]);
                    }
                    last_angle=now_angle_;
                    last_motor_test_state=state_;
                    *state=state_;
                }
                else
                    ROS_INFO("没收到电机回复，再发送一次");
            }
                break;
            case TWIST_FINISH:
            {
                if(end_effector->sendSpeedCommand(motor_nz,0))
                {
                    ROS_INFO("针灸结束\r\n");
                    *state=DO_NOTHING;
                }
            }
            break;
        }
   }


    void FSM::completeProcess(end_effector::endEffector *end_effector,end_sensor::endSensor *end_sensor,end_putter::endPutter *end_putter,
                         int *state,int motor_tc,int motor_nz,
                         uint16_t insert_speed,double insert_depth,double skin_thickness,
                         double tc_depth,int tc_times_per_min,int tc_times,
                         double nz_angle,int nz_times_per_min,int nz_times)
    {
        switch(*state)
        {
            case KEY_INPUT:
            {
                int64_t motor_tc_initial_angle_=420;
                int32_t motor_tc_initial_speed_=150;
                if(end_effector->sendSpeedCommand(motor_tc,0))
                {
                    std::cout<<"请输入要刺入的角度"<<std::endl;
                    std::cin>>putter_target_angle;
                    std::cout<<"要刺入的角度："<<putter_target_angle<<std::endl;
                    if(end_effector->sendAngleCommand(motor_tc,(uint16_t)(motor_tc_initial_speed_/DPS2ANGLE_COMMAND),(int32_t)(motor_tc_initial_angle_/DEGREE2ANGLE_COMMAND))
                    &&end_putter->send_string("1 1 45\n"))
                    {
                        sleep(3);
                        *state=INIT_DEVICE;
                    }
                }
            }
                break;
            case INIT_DEVICE:
            {
                if(end_effector->sendSpeedCommand(motor_tc,0))
                {
                    std::string input_;
                    std::cout<<"按任意键开始针灸"<<std::endl;
                    std::cin>>input_;
                    *state=START_MOVE;
                }
            }
                break;
            case START_MOVE:
            {
                if(end_effector->sendSpeedCommand(motor_tc,-(insert_speed/DPS2SPEED_COMMAND)))
                    *state=RISING_DETECT;
            }
                break;
            case RISING_DETECT:
            {
                int64_t now_motor_angle_{};
                size_t rising_deque_max_size_=5;
                int rising_threshold_=3;
                if(end_effector->readMotorAngle(motor_tc,&now_motor_angle_))
                {
                    if(end_sensor->detectPressureTrends(now_motor_angle_,rising_deque_max_size_,rising_threshold_,rising_angle,rising_value,&rising_average_value)==RISING)
                    {
                        ROS_INFO("检测到压力值上升!上升开始时角度值：%ld\r\n",rising_angle[0]);
                        detected_rising_time=ros::Time().now().toSec();
                        *state=DROPPING_DETECT;
                    }
                }
            }
                break;
            case DROPPING_DETECT:
            {
                int64_t now_motor_angle_{};
                size_t dropping_deque_max_size_=3;
                int dropping_threshold_=2;

                double duration_after_rise_=skin_thickness/( (insert_speed)*( 28.5/360.0) );
                if(ros::Time().now().toSec()-detected_rising_time>duration_after_rise_)
                {
                    ROS_INFO("理论上已经刺破皮肤，已停止\r\n");
                    detect_dropping=false;
                    *state=CURVED_DETECT;
                }
                else if(end_effector->readMotorAngle(motor_tc,&now_motor_angle_))
                {
                    if(end_sensor->detectPressureTrends(now_motor_angle_,dropping_deque_max_size_,dropping_threshold_,dropping_angle,dropping_value,&dropping_average_value)==DROPPING)
                    {
                        ROS_INFO("检测到压力值下降! 下降开始时角度值：%ld\r\n",dropping_angle[0]);
                        *state=CURVED_DETECT;
                    }
                }
            }
                break;
            case CURVED_DETECT:
            {
                if(end_effector->sendSpeedCommand(motor_tc,0))
                {
                    if(dropping_value[1]>curved_threshold)
                    {
                        ROS_INFO("发生弯针！！\r\n");
                        *state=DO_NOTHING;
                    }
                    else
                    {
                        detect_dropping=true;
                        *state=DETECT_FINISH;
                    }
                }
            }
                break;
            case DETECT_FINISH:
            {

                int64_t now_angle_;
                if(end_effector->readMotorAngle(motor_tc,&now_angle_))
                {
                    replenish_angle=(detect_dropping?(dropping_angle[0]-now_angle_)/100.0:(rising_angle[0]-now_angle_-(int64_t)(skin_thickness/( 28.5/360.0)))/100.0);
                    ROS_INFO("刺穿表皮，电机停止！去皮后要补偿的角度值：%ld\r\n",replenish_angle);
                    sleep(1);
                    *state=INSERT_DEEPER;
                }

            }
                break;
            case INSERT_DEEPER:
            {
                double insert_angle_=insert_depth*360.0/28.5-replenish_angle;
                ROS_INFO("理论要继续下压的角度：%f\r\n",insert_angle_);
                if(end_effector->sendAngleCommand(motor_tc,(uint16_t)(insert_speed/DPS2ANGLE_COMMAND),(int32_t)(-insert_angle_/DEGREE2ANGLE_COMMAND)))
                {
                    ROS_INFO("准备开始提插\r\n");
                    ROS_INFO("3\r\n");
                    sleep(1);
                    ROS_INFO("2\r\n");
                    sleep(1);
                    ROS_INFO("1\r\n");
                    sleep(1);
                    last_motor_test_state=INSERT;
                    *state=INSERT;
                }

            }
                break;
            case INSERT:
            {
                ROS_INFO("提\r\n");
                double tc_angle_=tc_depth*360.0/28.5;
                double tc_dps_=tc_angle_*tc_coefficient;
                if(end_effector->sendAngleCommand(motor_tc,(uint16_t)(tc_dps_/DPS2ANGLE_COMMAND),(int32_t)(tc_angle_/DEGREE2ANGLE_COMMAND)))
                {
                    usleep((60*1000000)/(2*tc_times_per_min));
                    *state=SWITCH_TC_STATE;
                }
            }
                break;
            case PULL:
            {
                ROS_INFO("插\r\n");
                double tc_angle_=tc_depth*360.0/28.5*0.85;
                double tc_dps_=tc_angle_*tc_coefficient;
                if(end_effector->sendAngleCommand(motor_tc,(uint16_t)(tc_dps_/DPS2ANGLE_COMMAND),(int32_t)(-tc_angle_/DEGREE2ANGLE_COMMAND)))
                {
                    ROS_INFO("等待时间：%d\r\n",60/(2*tc_times_per_min)*1000000);
                    usleep((60*1000000)/(2*tc_times_per_min));
                    tc_accumulate_times++;
                    if(tc_accumulate_times<tc_times)
                        *state=SWITCH_TC_STATE;
                    else
                        *state=INSERT_FINISH;
                }
            }
                break;
            case SWITCH_TC_STATE:
            {
                int state_;
                if(last_motor_test_state==PULL)
                    state_=INSERT;
                else
                    state_=PULL;
                int64_t now_angle_{};
                double diff_angle_[2]{};
                usleep(100);
                if(end_effector->readMotorAngle(motor_tc,&now_angle_))
                {
                    ROS_INFO("当前角度 :%ld ; 上一次的角度: %ld \r\n",now_angle_,last_angle);
                    if(last_motor_test_state==PULL)
                    {
                        diff_angle_[0]=8.0;
                        diff_angle_[1]=((now_angle_-last_angle)*0.01);
                        ROS_INFO("提的角度差值：%f",diff_angle_[1]);
                    }
                    else //此时顺时针转，对于motor_angle来说是增加
                    {
                        diff_angle_[0]=2.0;
                        diff_angle_[1]=((last_angle-now_angle_)*0.01);
                        ROS_INFO("插的角度差值：%f",diff_angle_[1]);
                    }
                    last_angle=now_angle_;
                    last_motor_test_state=state_;
                    *state=state_;
                }
                else
                    ROS_INFO("没收到电机回复，再发送一次");
            }
                break;
            case INSERT_FINISH:
            {
                if(end_effector->sendSpeedCommand(motor_tc,0))
                {
                    ROS_INFO("提插结束，即将开始捻转\r\n");
                    ROS_INFO("3\r\n");
                    sleep(1);
                    ROS_INFO("2\r\n");
                    sleep(1);
                    ROS_INFO("1\r\n");
                    sleep(1);
                    *state=TWIST_BACK;
                }
            }
                break;
            case TWIST:
            {
                double nz_dps_=(nz_angle)*nz_coefficient;
                if(end_effector->sendAngleCommand(motor_nz,(uint16_t)(nz_dps_/DPS2ANGLE_COMMAND),(int32_t)(nz_angle/DEGREE2ANGLE_COMMAND)))
                {
                    usleep((60*1000000)/(2*nz_times_per_min));
                    nz_accumulate_times++;
                    if(nz_accumulate_times<nz_times)
                        *state=SWITCH_NZ_STATE;
                    else
                        *state=TWIST_FINISH;
                }
            }
                break;
            case TWIST_BACK:
            {
                double nz_dps_=(nz_angle)*nz_coefficient;
                if(end_effector->sendAngleCommand(motor_nz,(uint16_t)(nz_dps_/DPS2ANGLE_COMMAND),(int32_t)(-nz_angle/DEGREE2ANGLE_COMMAND)))
                {
                    usleep((60*1000000)/(2*nz_times_per_min));
                    *state=SWITCH_NZ_STATE;
                }
            }
                break;
            case SWITCH_NZ_STATE:
            {
                int state_;
                if(last_motor_test_state==TWIST)
                    state_=TWIST_BACK;
                else
                    state_=TWIST;
                int64_t now_angle_{};
                double diff_angle_[2]{};
                usleep(100);
                if(end_effector->readMotorAngle(motor_nz,&now_angle_))
                {
                    ROS_INFO("当前角度 :%ld ; 上一次的角度: %ld \r\n",now_angle_,last_angle);
                    if(state_==TWIST)
                    {
                        diff_angle_[0]=8.0;
                        diff_angle_[1]=((last_angle-now_angle_)*0.01);
                        ROS_INFO("捻的角度差值：%f",diff_angle_[1]);
                    }
                    else
                    {
                        diff_angle_[0]=2.0;
                        diff_angle_[1]=((now_angle_-last_angle)*0.01);
                        ROS_INFO("转的角度差值：%f",diff_angle_[1]);
                    }
                    last_angle=now_angle_;
                    last_motor_test_state=state_;
                    *state=state_;
                }
                else
                    ROS_INFO("没收到电机回复，再发送一次");
            }
                break;
            case TWIST_FINISH:
            {
                if(end_effector->sendSpeedCommand(motor_nz,0))
                {
                    ROS_INFO("针灸结束\r\n");
                    *state=DO_NOTHING;
                }
            }
                break;
        }
    }
}//fsm