#ifndef LIB_HUNGARIAN_WHX_
#define LIB_HUNGARIAN_WHX_

#include "libs_system_all.h"
#include "lib_io_boundingbox_whx.h"
#include "lib_others_whx.h"

using namespace std;
using namespace LIB_IO_BOUNDINGBOX_WHX;

namespace LIB_HUNGARIAN_WHX
{
    class HungarianWHX
    {
    private:
        /* data */
        BoxListWHX boxes_before;
        BoxListWHX boxes_now;
        vector<vector<bool>> optional_map;
        vector<bool> if_before_vis;
        vector<int> result_now;
        vector<int> result_before;
        bool IfOptional(const BoundingBox3DWHX &box_1, const BoundingBox3DWHX &box_2);
        int MatchByNow(int count_now);

    public:
        HungarianWHX(/* args */);
        ~HungarianWHX();
        HungarianWHX &Set(const BoxListWHX &boxes_before_input, const BoxListWHX &boxes_now_input);
        int MaxMatch();
        void CallbackResultNow(vector<int> &output_result_now);
        void CallbackResultBefore(vector<int> &output_result_before);
    };

    HungarianWHX::HungarianWHX(/* args */)
    {
    }

    HungarianWHX::~HungarianWHX()
    {
    }
}

#endif