//
// Created by aung on 2024/1/7.
//

#ifndef SRC_FSM_H
#define SRC_FSM_H

//END_DEVICE FSM
#define DO_NOTHING  0
#define INIT_DEVICE 1
#define INSERT      2
#define TWIST       3
#define TEST        4
#define KEY_INPUT   5
#define DEVICE_TEST 6
#define SENSOR_TEST 7
#define PULL        8
#define AUTO_INSERT  9

//INSERT FSM
#define RISING_DETECT   1
#define DROPPING_DETECT 2
#define DETECT_FINISH   3
#define START_MOVE      4

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
    void testMotorAccuracy(end_effector::endEffector *end_effector,file_operator::fileOperator *file_operator,int *state,int motor_ip, int encoder_data,int dps ,int duration);
    void testSensorData(end_sensor::endSensor *end_sensor,file_operator::fileOperator *file_operator, int freq);
    bool testInsertDirect(end_effector::endEffector *end_effector,end_sensor::endSensor *end_sensor,int *insert_state,int motor_tc);

private:
    int last_motor_test_state;
    int last_state{};
    int64_t last_angle{};
    double sensor_data[2]{};
    int64_t rising_angle{},dropping_angle{};
    void infoState(const int *state);
};

}


#endif //SRC_FSM_H
