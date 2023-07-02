#include "lib_hungarian_whx.h"

using namespace LIB_HUNGARIAN_WHX;

HungarianWHX &HungarianWHX::Set(const BoxListWHX &boxes_before_input, const BoxListWHX &boxes_now_input)
{
    this->boxes_before.clear();
    this->boxes_now.clear();
    this->optional_map.clear();
    this->if_before_vis.clear();
    this->result_now.clear();
    this->result_before.clear();

    CopyBoundingBox(this->boxes_before, boxes_before_input);
    CopyBoundingBox(this->boxes_now, boxes_now_input);

    this->if_before_vis.resize(this->boxes_before.size(), false);
    this->result_now.resize(this->boxes_now.size(), -1);
    this->result_before.resize(this->boxes_before.size(), -1);

    vector<bool> temp;
    for (int count_before = 0; count_before < boxes_before_input.size(); count_before++)
    {
        for (int count_now = 0; count_now < boxes_now_input.size(); count_now++)
        {
            temp.push_back(this->IfOptional(this->boxes_before.at(count_before), this->boxes_now.at(count_now)));
        }
        this->optional_map.push_back(temp);
        temp.clear();
    }
    // std::cout << this->optional_map;
    return *this;
}

bool HungarianWHX::IfOptional(const BoundingBox3DWHX &box_1, const BoundingBox3DWHX &box_2)
{
    if (box_1.type_name != box_2.type_name)
    {
        return false;
    }

    if (BoundingBoxCentralDistance(box_1, box_2) > 3)
    {
        return false;
    }

    // if (std::abs(box_1.velocity - box_2.velocity) > 0.5)
    // {
    //     return false;
    // }
    return true;
}

int HungarianWHX::MatchByNow(int count_now)
{
    for (int count_before = 0; count_before < this->boxes_before.size(); count_before++)
    {
        if (this->optional_map.at(count_before).at(count_now) == true && this->if_before_vis.at(count_before) == false)
        {
            this->if_before_vis[count_before] = true;
            if (this->result_before.at(count_before) == -1 || this->MatchByNow(this->result_before.at(count_before)) == 1)
            {
                this->result_now[count_now] = count_before;
                this->result_before[count_before] = count_now;
                return 1;
            }
        }
    }
    return 0;
}

int HungarianWHX::MaxMatch()
{
    int res = 0;
    this->result_before.clear();
    this->result_now.clear();
    this->result_now.resize(this->boxes_now.size(), -1);
    this->result_before.resize(this->boxes_before.size(), -1);

    for (int count_now = 0; count_now < this->boxes_now.size(); count_now++)
    {
        if (this->result_now[count_now] == -1)
        {
            this->if_before_vis.resize(this->boxes_before.size(), false);
            res += this->MatchByNow(count_now);
        }
    }
    return res;
}

void HungarianWHX::CallbackResultNow(vector<int> &output_result_now)
{
    output_result_now.clear();
    output_result_now.assign(this->result_now.begin(), this->result_now.end());
}

void HungarianWHX::CallbackResultBefore(vector<int> &output_result_before)
{
    output_result_before.clear();
    output_result_before.assign(this->result_before.begin(), this->result_before.end());
}