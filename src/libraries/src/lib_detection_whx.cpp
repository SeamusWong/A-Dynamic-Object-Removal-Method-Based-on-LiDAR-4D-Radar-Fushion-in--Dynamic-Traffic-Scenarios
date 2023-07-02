#include "lib_detection_whx.h"

using namespace LIB_DETECTION_WHX;

void DetectionData3DWHX::ReadFile(const std::string &file_path)
{
    std::string data_str;
    ifstream detection_file;
    detection_file.open(file_path);
    while (getline(detection_file, data_str))
    {
        char *data_cstr = (char *)data_str.c_str();
        char *num_cstr = strtok(data_cstr, ",");
        vector<float> data_vector;
        while (num_cstr != NULL)
        {
            data_vector.push_back(std::atof(num_cstr));
            num_cstr = strtok(NULL, ",");
        }
        frame_number.push_back(data_vector.at(0));
        target_type.push_back(data_vector.at(1));
        switch ((int)data_vector.at(1))
        {
        case 0:
            this->target_type_str.push_back("NONE");
            break;
        case 1:
            this->target_type_str.push_back("Pedestrian");
            break;
        case 2:
            this->target_type_str.push_back("Car");
            break;
        case 3:
            this->target_type_str.push_back("Cyclist");
            break;
        default:
            this->target_type_str.push_back("NONE");
            break;
        }
        useless_3.push_back(data_vector.at(2));
        useless_4.push_back(data_vector.at(3));
        useless_5.push_back(data_vector.at(4));
        useless_6.push_back(data_vector.at(5));
        useless_7.push_back(data_vector.at(6));
        center_x.push_back(data_vector.at(10));
        center_y.push_back(data_vector.at(11));
        center_z.push_back(data_vector.at(12));
        box_height.push_back(data_vector.at(7));
        box_width.push_back(data_vector.at(8));
        box_length.push_back(data_vector.at(9));
        angle_yaw.push_back(data_vector.at(13));
        confidence.push_back(data_vector.at(14));
    }
}

BoundingBox3DWHX DetectionData3DWHX::CallbackBoundingBox(int number)
{
    BoundingBox3DWHX box_single;
    box_single.type_name = target_type_str.at(number);
    box_single.center_x = center_x.at(number);
    box_single.center_y = center_y.at(number);
    box_single.center_z = center_z.at(number);
    box_single.box_height = box_height.at(number);
    box_single.box_length = box_length.at(number);
    box_single.box_width = box_width.at(number);
    box_single.angle_yaw = angle_yaw.at(number);
    return box_single;
}

void DetectionData3DWHX::CallbackBoxList(int frame, BoxListWHX &boxes)
{
    boxes.clear();
    for (int count = 0; count < frame_number.size(); count++)
    {
        if (frame_number.at(count) == frame)
        {
            boxes.push_back(CallbackBoundingBox(count));
        }
    }
}

bool DetectionData3DWHX::IsEmpty()
{
    return (this->center_x.empty() && this->center_y.empty() && this->center_z.empty());
}