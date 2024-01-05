//
// Created by aung on 2024/1/5.
//

#include "file_operator.h"

namespace file_operator
{
    fileOperator::fileOperator()
    = default;

    void fileOperator::writeToTXT(double max_force,double min_force,double interval_time)
    {
        ofs.open("src/ASR5_control/file.txt",std::ios::app);
        ofs<<"max_force:"<<max_force<<std::endl;
        ofs<<"min_force:"<<min_force<<std::endl;
        ofs<<"interval_time:"<<interval_time<<std::endl;
        ofs.close();
    }

    void fileOperator::readToTXT(std::vector<std::string> *v_string)
    {
        ifs.open("src/ASR5_control/file.txt",std::ios::in);
        std::string s_;
        if (!ifs.is_open())
        {
            std::cout << "read file fail!" << std::endl;
            return;
        }
        while (getline(ifs,s_))
            v_string->push_back(s_);
        ifs.close();
    }

} // filleOperator