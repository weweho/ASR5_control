//
// Created by aung on 2024/1/7.
//

#ifndef SRC_FSM_H
#define SRC_FSM_H

#define DO_NOTHING  0
//END_DEVICE FSM
#define INIT_DEVICE 1
#define INSERT      2
#define TWIST       3
#define TEST        4
#define KEY_INPUT   5
#define DEVICE_TEST 6
#define SENSOR_TEST 7
#define PULL        8
#define SWITCH_TC_STATE  9
#define WAIT            10

//INSERT FSM
#define RISING_DETECT   11
#define DROPPING_DETECT 12
#define DETECT_FINISH   13
#define START_MOVE      14
#define CURVED_DETECT   15
#define INSERT_WAIT     16

//OTHER
#define INSERT_DEEP     17
#define INSERT_FINISH   18
#define TWIST_BACK      19
#define TWIST_FINISH    20
#define SWITCH_NZ_STATE 21

//PRESSURE DETECT
#define RISING          1
#define DROPPING        2
#define STABLE          0

//CONVERT
#define DPS2SPEED_COMMAND       0.01    // 0.01 dps/LSB
#define DPS2ANGLE_COMMAND       1       // 1 dps/LSB
#define DEGREE2ANGLE_COMMAND    0.01    // 0.01 degree/LSB

#include "end_effector.h"
#include "end_sensor.h"
#include "end_putter.h"
#include "file_operator.h"

namespace fsm
{
class FSM
{
public:
    FSM();
    void testAllDevice(end_effector::endEffector *end_effector,end_sensor::endSensor *end_sensor, end_putter::endPutter *end_putter, int *state,int motor_nz,int motor_tc);
    void testMotorAccuracy(end_effector::endEffector *end_effector,file_operator::fileOperator *file_operator,int *state,int motor_ip, int encoder_data,int dps ,double duration);
    void testSensorData(end_sensor::endSensor *end_sensor,file_operator::fileOperator *file_operator, int freq);
    void testInsertDirect(end_effector::endEffector *end_effector,end_sensor::endSensor *end_sensor,int *insert_state,int motor_tc,int tc_speed,double skin_thickness); //记得负是向下！
    void completeProcess(end_effector::endEffector *end_effector,end_sensor::endSensor *end_sensor,end_putter::endPutter *end_putter,
                         int *state,int motor_tc,int motor_nz,
                         uint16_t insert_speed,int32_t insert_angle,double skin_thickness,
                         int tc_angle,int tc_times_per_min,int tc_times,
                         int nz_angle,int nz_times_per_min,int nz_times); //记得负是向下！
    double sensor_data[2]{};

private:
    int last_motor_test_state;
    int last_state{};
    int64_t last_angle{};
    int64_t rising_angle[2]{},dropping_angle[2]{};
    double rising_value[2]{},dropping_value[2]{};
    double curved_threshold{};
    double rising_average_value{},dropping_average_value{};
    double detected_rising_time{};
    bool detect_dropping{false};
    void infoState(const int *state);

    int putter_target_angle;
    bool initial_putter_ready{false};
    int64_t replenish_angle{};
    int tc_accumulate_times{},nz_accumulate_times{};
    double tc_coefficient{},nz_coefficient{};

};

}


#endif //SRC_FSM_H
