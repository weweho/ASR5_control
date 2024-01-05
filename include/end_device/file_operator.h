//
// Created by aung on 2024/1/5.
//

#ifndef SRC_FILE_OPERATOR_H
#define SRC_FILE_OPERATOR_H

#include <vector>
#include <string>
#include <fstream>
#include <iostream>

namespace file_operator
{
class fileOperator
{
public:
    fileOperator();
    void writeToTXT(double max_force,double min_force,double interval_time);
    void readToTXT(std::vector<std::string>* v1);

private:
    std::ofstream ofs;
    std::ifstream ifs;
};

} // fille_operator

#endif //SRC_FILE_OPERATOR_H
