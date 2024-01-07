//
// Created by aung on 2024/1/7.
//

#ifndef SRC_FSM_H
#define SRC_FSM_H

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
#include "end_putter.h"
#include "file_operator.h"

namespace fsm
{
class FSM
{
public:
    void controlEndDevice(end_effector::endEffector *end_effector,end_sensor::endSensor *end_sensor, end_putter::endPutter *end_putter, file_operator::fileOperator *file_operator, int *state,int motor_nz,int motor_tc);
    void testEndDevice(end_effector::endEffector *end_effector,end_sensor::endSensor *end_sensor, end_putter::endPutter *end_putter, int *state,int motor_nz,int motor_tc);

private:
    int last_state{};
    void infoState(const int *state);

};

}


#endif //SRC_FSM_H