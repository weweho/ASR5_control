//
// Created by aung on 2024/1/5.
//

#include "end_putter.h"

namespace end_putter
{
    endPutter::endPutter()
    {
        to = serial::Timeout::simpleTimeout(100);
        sp.setPort("/dev/putter");
        sp.setBaudrate(115200);
        sp.setTimeout(to);
    }

    bool endPutter::initPutter()
    {
        try
        {
            sp.open();
        }
        catch(serial::IOException& e)
        {
            ROS_ERROR_STREAM("Unable to open port.");
            return false;
        }
        return true;
    }

    bool endPutter::putterisOpen()
    {
        if(sp.isOpen())
        {
            ROS_INFO_ONCE("/dev/putter is opened.");
            return true;
        }
        else
        {
            ROS_INFO_ONCE("/dev/putter open fail.");
            return false;
        }
    }

    bool endPutter::send_string(const char *str)
    {
        if(sp.isOpen())
        {
            sp.write(str);
            return true;
        }
        return false;
    }

    bool endPutter::readESPData()
    {
        std::string esp_rec{};
        if (sp.isOpen()&&sp.available())
        {
            std::string str_;
            sp.read(esp_rec,150);
            ROS_INFO("%s",esp_rec.c_str());
            return true;
        }
        return false;
    }

};