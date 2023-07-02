#ifndef _LIB_TRACKING_WHX_
#define _LIB_TRACKING_WHX_

#include "lib_io_boundingbox_whx.h"
#include "lib_hungarian_whx.h"
#include "lib_others_whx.h"

using namespace std;
using namespace LIB_IO_BOUNDINGBOX_WHX;
using namespace LIB_HUNGARIAN_WHX;

namespace LIB_TRACKING_WHX
{
    class TargetWHX
    {
    private:
        std::string m_type;
        vector<int> m_frames;
        BoxListWHX m_boxes;
        bool m_dynamic;

    public:
        TargetWHX(/* args */);
        explicit TargetWHX(std::string target_type);
        ~TargetWHX();
        TargetWHX &SetBox(int frame, const BoundingBox3DWHX &target_box);
        TargetWHX &SetDynamic(bool dynamic);
        bool IfHave(int frame);
        BoundingBox3DWHX CallbackBox(int frame);

        friend std::ostream &operator<<(std::ostream &os,const TargetWHX &out);
    };
    
    TargetWHX::TargetWHX(/* args */)
    {
    }

    TargetWHX::TargetWHX(std::string target_type)
    {
        this->m_type = target_type;
    }

    TargetWHX::~TargetWHX()
    {
    }

    std::ostream &operator<<(std::ostream &os, const vector<int> &out)
    {
        for (int count = 0; count < out.size(); count++)
        {
            os << out.at(count) << " ";
        }
        return os;
    }

    std::ostream &operator<<(std::ostream &os, const TargetWHX &out)
    {
        std::cout << setw(0) << setfill(' ')
                  << setw(0) << setfill(' ') << " Type: " << setw(10) << setfill(' ') << out.m_type
                  << setw(0) << setfill(' ') << " FrameList: " 
                  << out.m_frames
                  << endl
                  << out.m_boxes;
        return os;
    }

    class TargetListWHX
    {
    private:
        vector<TargetWHX> m_list;
        HungarianWHX m_hungarian;
        bool m_init = false;

    public: 
        TargetListWHX(/* args */);
        ~TargetListWHX(); 
        TargetListWHX &Init(int frame, const BoxListWHX &box_list);
        TargetListWHX &Input(int frame, const BoxListWHX &box_list);
        TargetListWHX &CallbackBoxList(int frame, BoxListWHX &out);
    };
    
    TargetListWHX::TargetListWHX(/* args */)
    {
    }
    
    TargetListWHX::~TargetListWHX()
    {
    }
    
    
}


#endif