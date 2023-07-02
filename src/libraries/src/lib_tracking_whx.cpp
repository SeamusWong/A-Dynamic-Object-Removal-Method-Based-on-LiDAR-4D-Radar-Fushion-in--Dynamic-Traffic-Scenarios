#include "lib_tracking_whx.h"
#include "lib_others_whx.h"

using namespace LIB_TRACKING_WHX;

TargetWHX &TargetWHX::SetBox(int frame, const BoundingBox3DWHX &target_box)
{
    this->m_frames.push_back(frame);
    this->m_boxes.push_back(target_box);
    return *this;
}

TargetWHX &TargetWHX::SetDynamic(bool dynamic)
{
    this->m_dynamic = dynamic;
    return *this;
}

bool TargetWHX::IfHave(int frame)
{
    if(VectorFind<int>(this->m_frames, frame) == -1)
    {
        return false;
    }
    return true;
}

BoundingBox3DWHX TargetWHX::CallbackBox(int frame)
{
    return this->m_boxes.at(VectorFind<int>(this->m_frames,frame));
}



/************************************************************************************/

TargetListWHX &TargetListWHX::CallbackBoxList(int frame,BoxListWHX &out)
{
    out.clear();
    for(int count = 0;count < this->m_list.size();count++)
    {
        TargetWHX temp_tar = this->m_list.at(count);
        BoundingBox3DWHX temp_box;

        // cout << temp_tar.IfHave(frame) << endl;

        if (temp_tar.IfHave(frame))
        {
            temp_box = temp_tar.CallbackBox(frame);
            temp_box.id = count;
            out.push_back(temp_box);
        }
    }
    return *this;
}

TargetListWHX &TargetListWHX::Input(int frame, const BoxListWHX &box_list)
{
    BoxListWHX box_list_before;
    vector<int> hung_by_now;
    this->CallbackBoxList(frame - 1 ,box_list_before);
    this->m_hungarian.Set(box_list_before,box_list).MaxMatch();
    this->m_hungarian.CallbackResultNow(hung_by_now);

    for (int count = 0; count < hung_by_now.size(); count++)
    {
        if(hung_by_now.at(count) != -1)
        {
            int target_id = box_list_before.at(hung_by_now.at(count)).id;
            BoundingBox3DWHX temp_box = box_list.at(count);
            temp_box.id = target_id;
            this->m_list[target_id].SetBox(frame, temp_box);
        }
        else
        {
            BoundingBox3DWHX temp_box = box_list.at(count);
            TargetWHX new_target(temp_box.type_name);
            temp_box.id = this->m_list.size();
            new_target.SetBox(frame, temp_box);
            this->m_list.push_back(new_target);
        }
    }
    return *this;
    
}

TargetListWHX &TargetListWHX::Init(int frame, const BoxListWHX &box_list)
{
    for(int count = 0;count < box_list.size();count++)
    {
        BoundingBox3DWHX temp_box = box_list.at(count);
        TargetWHX temp(temp_box.type_name);
        temp_box.id = count;
        temp.SetBox(frame, temp_box);
        this->m_list.push_back(temp);
    }
    return *this;
}
