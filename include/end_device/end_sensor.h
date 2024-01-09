//
// Created by aung on 2024/1/1.
//

#ifndef SRC_END_SENSOR_H
#define SRC_END_SENSOR_H
#include <ros/ros.h>
#include <serial/serial.h>
#include <iostream>
#include <deque>

struct SensorData {
    int64_t angle;
    double value;
};

namespace end_sensor
{
class endSensor
{
public:
    endSensor();
    bool initUSB0();
    bool USB0isOpen();
    bool getSensorData(double *value);
    int detectPressureTrends(int64_t now_motor_angle,size_t max_size,int threshold,int64_t *return_motor_angle); // 0 正常情况； 1 压力递增； 2 压力递减；

private:
    serial::Serial sp;
    serial::Timeout to;
    uint8_t end_sensor_send[8]{};
    uint8_t end_sensor_rec[9]{};
    std::deque<SensorData> sensor_data_buffer;
    double last_value{},second_last_value{};
    static void storeSensorData(std::deque<SensorData>& buffer, const SensorData& newData, size_t maxSize);

};

} // end_sensor

#endif //SRC_END_SENSOR_H
