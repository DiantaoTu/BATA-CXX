/*
 * @Author: Diantao Tu
 * @Date: 2022-04-30 15:04:48
 */
#ifndef _COMMON_H_
#define _COMMON_H_

#include <iostream>
#include <string>
#include <vector>
#include <Eigen/Core>

using namespace std;

template<typename T>
using eigen_vector = std::vector<T, Eigen::aligned_allocator<T>>;

vector<string> SplitString(const string& str, const char& delimma);

template<typename T>
inline T str2num(string str)
{
    T num;
    stringstream sin(str);
    if(sin >> num) {
        return num;
    }
    else{
        cout << "str2num error" << endl;
        exit(0);
    }
}

template<typename T>
inline string num2str(T num)
{
    ostringstream oss;
    if (oss << num) {
        string str(oss.str());
        return str;
    }
    else {
        cout << "num2str error" << endl;
        exit(0);
    }
}

#endif