//
// Created by aung on 2024/1/5.
//
// using template function
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
    void readTXT(vector<std::string>* v1, const string& file_name);
    void readExcel(vector<std::string>* v1, const string& file_name);
    static void getVectorData(vector<std::string> v1,int column_num,bool ignore_first_row);
    template <typename T> void writeToTXT(T data[], int size, const string& file_name)
    {
        txt_ofs.open("src/ASR5_control/"+file_name,ios::app);
        for(int i=0; i<size; i++)
            txt_ofs<<"data "<<i<<":"<<data[i]<<endl;
        txt_ofs.close();
    }
    template <typename T> void writeToExcel(T data[], int size, const string& file_name)
    {
        excel_ofs.open("src/ASR5_control/"+file_name, ios::app);
        for(int i=0; i<size; i++)
        {
            if(i==size-1)
                excel_ofs << data[i]<< endl;
            else
                excel_ofs << data[i] << ",";
        }
        excel_ofs.close();
    }
private:
    std::ofstream txt_ofs,excel_ofs;
    std::ifstream txt_ifs,excel_ifs;
};

} // fille_operator

#endif //SRC_FILE_OPERATOR_H
