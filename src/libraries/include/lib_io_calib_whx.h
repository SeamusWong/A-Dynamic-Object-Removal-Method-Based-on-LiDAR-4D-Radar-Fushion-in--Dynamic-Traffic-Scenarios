#ifndef LIB_IO_CALIB_WHX_
#define LIB_IO_CALIB_WHX_

#include "libs_system_all.h"

#include "lib_io_boundingbox_whx.h"

using namespace std;
using namespace LIB_IO_BOUNDINGBOX_WHX;

namespace LIB_IO_CALIB_WHX
{
    class CalibDataWHX
    {
    private:
        Eigen::Matrix<float, 3, 4> matrix_p0;
        Eigen::Matrix<float, 3, 4> matrix_p1;
        Eigen::Matrix<float, 3, 4> matrix_p2;
        Eigen::Matrix<float, 3, 4> matrix_p3;
        Eigen::Matrix<float, 3, 3> matrix_rect;
        Eigen::Matrix<float, 3, 4> matrix_velo_to_cam;
        Eigen::Matrix<float, 3, 4> matrix_imu_to_velo;
        void ReadMatrixFromLine(const std::string line, Eigen::Matrix<float, 3, 4> &result_matrix, const std::string delimiter = " ");
        void ReadMatrixFromLine(const std::string line, Eigen::Matrix<float, 3, 3> &result_matrix, const std::string delimiter = " ");

    public:
        static const int p0_code = 1, p1_code = 2, p2_code = 3, p3_code = 4, rect_code = 5, vtc_code = 6, itv_code = 7;
        CalibDataWHX();
        explicit CalibDataWHX(const std::string &file_path);
        ~CalibDataWHX();
        bool IsEmpty();
        void ReadFile(const std::string &file_path);
        Eigen::Matrix3Xf CallbackMatrix(int mat_code) const;
        Eigen::Matrix4f CallbackMatrix4f(int mat_code) const;
        void TransBoxList(BoxListWHX &boxes);
    };

    CalibDataWHX::CalibDataWHX()
    {
    }

    CalibDataWHX::CalibDataWHX(const std::string &file_path)
    {
        this->ReadFile(file_path);
    }

    CalibDataWHX::~CalibDataWHX()
    {
    }

}

#endif