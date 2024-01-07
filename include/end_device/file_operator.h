//
// Created by aung on 2024/1/5.
//

#ifndef SRC_FILE_OPERATOR_H
#define SRC_FILE_OPERATOR_H

#include <vector>
#include <string>
#include <fstream>
#include <iostream>
#include <streambuf>
#include <sstream>

using namespace std;
namespace file_operator
{
class fileOperator
{
public:
    fileOperator();
    void writeToTXT(double max_force,double min_force,double interval_time);
    void readTXT(vector<std::string>* v1);
    void writeToExcel(double max_force,double min_force,double interval_time);
    void readExcel(vector<std::string>* v1);
    static void getVectorData(vector<std::string> v1,int column_num);

private:
    std::ofstream txt_ofs,excel_ofs;
    std::ifstream txt_ifs,excel_ifs;
};

} // fille_operator

#endif //SRC_FILE_OPERATOR_H
