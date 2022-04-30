/*
 * @Author: Diantao Tu
 * @Date: 2022-04-30 16:42:31
 */
#include "common.h"
using namespace std;
vector<string> SplitString(const string& str, const char& delimma)
{
    vector<string> split;
    stringstream ss(str);
    string tmp;
    while (getline(ss, tmp, delimma))
    {
        if(!tmp.empty() && tmp[0] != delimma)
            split.push_back(tmp);
    }
    return split;
}