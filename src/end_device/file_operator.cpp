//
// Created by aung on 2024/1/5.
//

#include "file_operator.h"

using namespace std;
namespace file_operator
{
    fileOperator::fileOperator()
    = default;

    void fileOperator::writeToTXT(double data[], int size)
    {
        txt_ofs.open("src/ASR5_control/file.txt",ios::app);
        for(int i=0; i<size; i++)
            txt_ofs<<"data "<<i<<":"<<data[i]<<endl;
        txt_ofs.close();
    }

    void fileOperator::writeToExcel(double data[], int size)
    {
        excel_ofs.open("src/ASR5_control/file.csv", ios::app);
        for(int i=0; i<size; i++)
        {
            if(i==size-1)
                excel_ofs << data[i]<< endl;
            else
                excel_ofs << data[i] << ",";
        }
        excel_ofs.close();
    }

    void fileOperator::readTXT(vector<string> *v_string)
    {
        txt_ifs.open("src/ASR5_control/file.txt",ios::in);
        string s_;
        if (!txt_ifs.is_open())
        {
            cout << "read txt fail!" << endl;
            return;
        }
        while (getline(txt_ifs,s_))
            v_string->push_back(s_);
        txt_ifs.close();
    }

    void fileOperator::readExcel(vector<string>* v1)
    {
        excel_ifs.open("src/ASR5_control/file.csv", ios::in);
        string temp;
        if (!excel_ifs.is_open())
        {
            cout << "read excel fail!"<< endl;
            return;
        }
        while (getline(excel_ifs, temp))
            v1->push_back(temp);
    }

    void fileOperator::getVectorData(vector<std::string> v1,int column_num,bool ignore_first_row)
    {
        for (auto it = v1.begin()+(ignore_first_row?1:0); it != v1.end(); it++)        //遍历文件中的每一行数据
        {
            string str;
            istringstream istr(*it);
            //将字符串流 istr 中的字符读入到 str 字符串中，读取方式是以逗号为分隔符
            for(int i=0; i<column_num; i++)
            {
                getline(istr, str, ',');
                cout << str << "\t";            // str 对应第一列数据
            }
            cout<<"\r\n"<<endl;
        }
    }

} // filleOperator