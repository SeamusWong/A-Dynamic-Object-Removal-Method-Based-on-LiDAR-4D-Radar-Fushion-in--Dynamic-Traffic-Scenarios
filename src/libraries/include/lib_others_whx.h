#ifndef LIB_OTHERS_WHX_
#define LIB_OTHERS_WHX_

#include "libs_system_all.h"

using namespace std;

template <typename T>
std::ostream &operator<<(std::ostream &os, const vector<T> &out)
{
    for (int count = 0; count < out.size(); count++)
    {
        os << out.at(count) << " ";
    }
    return os;
}

template <typename T>
std::ostream &operator<<(std::ostream &os, const vector<vector<T>> &out_mat)
{
    for (int count_row = 0; count_row < out_mat.size(); count_row++)
    {
        os << out_mat.at(count_row) << endl;
    }
    return os;
}

inline std_msgs::ColorRGBA SetColor(int r, int g, int b, int a)
{
    std_msgs::ColorRGBA color;
    color.r = r;
    color.g = g;
    color.b = b;
    color.a = a;
    return color;
}

template <typename T>
int VectorFind(const vector<T> &vec, T find);
template <typename T>
int VectorFind(const vector<T> &vec, T find, int start);

template <typename T>
int VectorFind(const vector<T> &vec, T find)
{
    return VectorFind(vec, find, 0);
}

template <typename T>
int VectorFind(const vector<T> &vec, T find, int start)
{
    
    for (int count = start; count < vec.size(); count++)
    {
        if (vec.at(count) == find)
        {
            return count;
        }
    }
    return -1;
}

#endif