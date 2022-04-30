/*
 * @Author: Diantao Tu
 * @Date: 2022-04-30 15:04:48
 */
#include <iostream>
#include <string>
#include <vector>

using namespace std;
vector<string> SplitString(const string& str, const char& delimma)
{
    vector<string> split;
    stringstream ss(str);
    string tmp;
    while (getline(ss, tmp, delimma))
    {
        split.push_back(tmp);
    }
    return split;
}

template<typename T>
inline T str2num(string str)
{
    T num;
    stringstream sin(str);
    if(sin >> num) {
        return num;
    }
    else{
        cout << "str2num error";
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