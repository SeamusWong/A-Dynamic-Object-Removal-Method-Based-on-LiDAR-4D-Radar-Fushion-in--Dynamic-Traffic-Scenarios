#include "lib_io_calib_whx.h"

using namespace LIB_IO_CALIB_WHX;

void CalibDataWHX::ReadFile(const std::string &file_path)
{
    ifstream calib_fstream;
    calib_fstream.open(file_path);
    if (calib_fstream.is_open() == false)
    {
        std::cout << "打开Calib文件错误!" << endl;
        exit(EXIT_FAILURE);
    }

    while (calib_fstream.eof() == false)
    {
        std::string calib_line;
        getline(calib_fstream, calib_line);
        if (calib_line.find("P0") != std::string::npos)
        {
            this->ReadMatrixFromLine(calib_line, this->matrix_p0, " ");
        }
        else if (calib_line.find("P1") != std::string::npos)
        {
            this->ReadMatrixFromLine(calib_line, this->matrix_p1, " ");
        }
        else if (calib_line.find("P2") != std::string::npos)
        {
            this->ReadMatrixFromLine(calib_line, this->matrix_p2, " ");
        }
        else if (calib_line.find("P3") != std::string::npos)
        {
            this->ReadMatrixFromLine(calib_line, this->matrix_p3, " ");
        }
        else if (calib_line.find("R_rect") != std::string::npos || calib_line.find("R0_rect") != std::string::npos)
        {
            this->ReadMatrixFromLine(calib_line, this->matrix_rect, " ");
        }
        else if (calib_line.find("Tr_imu_to_velo") != std::string::npos)
        {
            this->ReadMatrixFromLine(calib_line, this->matrix_imu_to_velo, " ");
        }
        else if (calib_line.find("Tr_velo_to_cam") != std::string::npos)
        {
            this->ReadMatrixFromLine(calib_line, this->matrix_velo_to_cam, " ");
        }
        else;
    }
    calib_fstream.close();
}

void CalibDataWHX::ReadMatrixFromLine(const std::string line, Eigen::Matrix<float, 3, 4> &result_matrix, const std::string delimiter)
{
    char *data_cstr = (char *)line.c_str();
    char *num_cstr = strtok(data_cstr, delimiter.c_str());
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
    if (data_vector.size() != 12)
        return;
    for (int count_row = 0, count_num = 0; count_row < result_matrix.rows(); count_row++)
    {
        for (int count_col = 0; count_col < result_matrix.cols(); count_col++)
        {
            result_matrix(count_row, count_col) = data_vector.at(count_num++);
        }
    }
}

void CalibDataWHX::ReadMatrixFromLine(const std::string line, Eigen::Matrix<float, 3, 3> &result_matrix, const std::string delimiter)
{
    char *data_cstr = (char *)line.c_str();
    char *num_cstr = strtok(data_cstr, delimiter.c_str());
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
    if (data_vector.size() != 9)
        return;
    for (int count_row = 0, count_num = 0; count_row < result_matrix.rows(); count_row++)
    {
        for (int count_col = 0; count_col < result_matrix.cols(); count_col++)
        {
            result_matrix(count_row, count_col) = data_vector.at(count_num++);
        }
    }
}

Eigen::Matrix3Xf CalibDataWHX::CallbackMatrix(int matrix_code) const
{
    Eigen::Matrix3Xf result;
    switch (matrix_code)
    {
    case p0_code:
        result = this->matrix_p0;
        break;
    case p1_code:
        result = this->matrix_p1;
        break;
    case p2_code:
        result = this->matrix_p2;
        break;
    case p3_code:
        result = this->matrix_p3;
        break;
    case rect_code:
        result = this->matrix_rect;
        break;
    case vtc_code:
        result = this->matrix_velo_to_cam;
        break;
    case itv_code:
        result = this->matrix_imu_to_velo;
        break;
    default:
        break;
    }
    return result;
}

Eigen::Matrix4f CalibDataWHX::CallbackMatrix4f(int mat_code) const
{
    Eigen::Matrix4f result;
    if (mat_code == this->rect_code)
    {
        Eigen::Matrix3f real_result;
        Eigen::Vector3f add_vector;
        Eigen::RowVector4f add_line;
        Eigen::Matrix<float, 3, 4> temp;
        add_vector << 0, 0, 0;
        add_line << 0, 0, 0, 1;
        real_result = CallbackMatrix(mat_code);
        temp << real_result, add_vector;
        result << temp, add_line;
    }
    else
    {
        Eigen::Matrix<float, 3, 4> real_result;
        Eigen::RowVector4f add_line;
        add_line << 0, 0, 0, 1;
        real_result = CallbackMatrix(mat_code);
        result << real_result, add_line;
    }
    return result;
}

void CalibDataWHX::TransBoxList(BoxListWHX &boxes)
{
    Eigen::Matrix4f mat_vel_to_cam_inverse, mat_rect_inverse, mat_p0_inverse;
    mat_vel_to_cam_inverse = this->CallbackMatrix4f(CalibDataWHX::vtc_code).inverse();
    mat_rect_inverse = this->CallbackMatrix4f(CalibDataWHX::rect_code).inverse();
    mat_p0_inverse = this->CallbackMatrix4f(CalibDataWHX::p0_code).inverse();
    TramsformBoundingBox3D(boxes, mat_rect_inverse);
    TramsformBoundingBox3D(boxes, mat_vel_to_cam_inverse);
    BoundingBox3DhwlaTohlwa(boxes);
    ExpandBoundingBox3D(boxes, 0.1);
}

bool CalibDataWHX::IsEmpty()
{
    if (this->matrix_p0.isZero() && this->matrix_p1.isZero() && this->matrix_p2.isZero() && this->matrix_p3.isZero() == true)
    {
        return true;
    }
    else return false;
}
