//
// Created by aung on 2024/1/5.
//

#ifndef SRC_END_PUTTER_H
#define SRC_END_PUTTER_H
#include <ros/ros.h>
#include <serial/serial.h>

namespace end_putter
{
class endPutter
{
public:
    endPutter();
    bool initPutter();
    bool putterisOpen();
    bool readESPData();
    bool send_string(const char *str);

private:
    serial::Serial sp;
    serial::Timeout to;
};

};


#endif //SRC_END_PUTTER_H
