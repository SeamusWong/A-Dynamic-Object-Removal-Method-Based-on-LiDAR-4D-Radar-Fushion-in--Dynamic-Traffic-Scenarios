#ifndef LIB_EKF_WHX_
#define LIB_EKF_WHX_

#include "libs_system_all.h"

using namespace std;

namespace LIB_EKF_WHX
{
    class ExtendedKalmanFilterWHX
    {
    private:
        bool is_init = false;
        Eigen::Vector4f initial_vec;
        Eigen::Matrix4f f_state_transistion_mat;
        Eigen::Matrix4f p_state_covariance_mat;
        Eigen::Matrix4f q_process_covariance_mat;
        Eigen::Matrix4f h_measurement_mat;
        Eigen::Matrix4f k_kalman_gain_mat;
        Eigen::Matrix4f r_measurement_covariance_mat;
        void Predict();

    public:
        static const int f_code = 0,p_code = 1,q_code = 2,h_code = 3,r_code = 4;
        ExtendedKalmanFilterWHX(/* args */);
        ~ExtendedKalmanFilterWHX();
        ExtendedKalmanFilterWHX &Set(int mat_code, Eigen::Matrix4f mat_input);
        ExtendedKalmanFilterWHX &DefaultSet(float update_time);
        ExtendedKalmanFilterWHX &Init(Eigen::Vector4f mat_input);
        ExtendedKalmanFilterWHX &UpdateMeasurement(const Eigen::Vector4f observation);
        Eigen::Vector4f GetResult();
    };

    ExtendedKalmanFilterWHX::ExtendedKalmanFilterWHX(/* args */)
    {
    }

    ExtendedKalmanFilterWHX::~ExtendedKalmanFilterWHX()
    {
    }
}


#endif