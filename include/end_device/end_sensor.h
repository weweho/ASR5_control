//
// Created by aung on 2024/1/1.
//

#ifndef SRC_END_SENSOR_H
#define SRC_END_SENSOR_H
#include <ros/ros.h>
#include <serial/serial.h>

namespace end_sensor
{
class endSensor
{
public:
    endSensor();
    bool initUSB0();
    bool USB0isOpen();
    bool getSensorData(double *value);
    bool sendPutterCommand(bool is_push,int angle);

private:
    serial::Serial sp;
    serial::Timeout to;
    uint8_t end_sensor_send[8]{};
    uint8_t end_sensor_rec[9]{};
    uint8_t putter_send[3]{};

};

} // end_sensor

#endif //SRC_END_SENSOR_H
