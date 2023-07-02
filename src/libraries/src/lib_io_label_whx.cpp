#include "lib_io_label_whx.h"

using namespace LIB_IO_LABEL_WHX;

void LabelDataWHX::ReadFile(const std::string &file_path)
{
    std::string data_str;
    ifstream label_file;
    label_file.open(file_path);
    while (getline(label_file, data_str))
    {
        char *data_cstr = (char *)data_str.c_str();
        char *num_cstr = strtok(data_cstr, " ");
        vector<float> data_vector;
        vector<std::string> str_vector;
        while (num_cstr != NULL)
        {
            if (num_cstr[0] != '-' && !(num_cstr[0] >= '0' && num_cstr[0] <= '9'))
            {
                str_vector.push_back(num_cstr);
            }
            else
            {
                data_vector.push_back(std::atof(num_cstr));
            }
            num_cstr = strtok(NULL, " ");
        }
        target_type.push_back(str_vector.at(0));
        useless_2.push_back(data_vector.at(0));
        useless_3.push_back(data_vector.at(1));
        useless_4.push_back(data_vector.at(2));
        useless_5.push_back(data_vector.at(3));
        useless_6.push_back(data_vector.at(4));
        useless_7.push_back(data_vector.at(5));
        useless_8.push_back(data_vector.at(6));
        box_height.push_back(data_vector.at(7));
        box_width.push_back(data_vector.at(8));
        box_length.push_back(data_vector.at(9));
        center_x.push_back(data_vector.at(10));
        center_y.push_back(data_vector.at(11));
        center_z.push_back(data_vector.at(12));
        angle_yaw.push_back(data_vector.at(13));
        confidence.push_back(data_vector.at(14));
    }
    if (target_type.size() == 0)
    {
        cout << "未检测到数据" << endl;
        exit(1);
    }
}

BoundingBox3DWHX LabelDataWHX::CallbackBoundingBox(int number)
{
    BoundingBox3DWHX box_single;
    box_single.type_name = this->target_type.at(number);
    box_single.center_x = this->center_x.at(number);
    box_single.center_y = this->center_y.at(number);
    box_single.center_z = this->center_z.at(number);
    box_single.box_height = this->box_height.at(number);
    box_single.box_length = this->box_length.at(number);
    box_single.box_width = this->box_width.at(number);
    box_single.angle_yaw = this->angle_yaw.at(number);
    return box_single;
}

void LabelDataWHX::CallbackBoxList(BoxListWHX &boxes)
{
    boxes.clear();
    for (int count = 0; count < target_type.size(); count++)
    {
        BoundingBox3DWHX box_single = this->CallbackBoundingBox(count);
        // if (box_single.type_name == "Car" || box_single.type_name == "truck" || box_single.type_name == "bicycle" || box_single.type_name == "motor" || box_single.type_name == "moped_scooter")
        {
            // if (sqrt(pow(box_single.center_x, 2) + pow(box_single.center_y, 2)) < 30)
            {
                boxes.push_back(box_single);
            }
        }
    }
}

bool LabelDataWHX::IsEmpty()
{
    return (this->center_x.empty() && this->center_y.empty() && this->center_z.empty());
}