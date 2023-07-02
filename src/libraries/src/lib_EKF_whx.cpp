#include "lib_EKF_whx.h"

using namespace LIB_EKF_WHX;

ExtendedKalmanFilterWHX &ExtendedKalmanFilterWHX::Set(int mat_code, Eigen::Matrix4f mat_input)
{
    switch (mat_code)
    {
    case this->f_code:
        this->f_state_transistion_mat = mat_input;
        break;
    case this->p_code:
        this->p_state_covariance_mat = mat_input;
        break;
    case this->q_code:
        this->q_process_covariance_mat = mat_input;
        break;
    case this->h_code:
        this->h_measurement_mat = mat_input;
        break;
    case this->r_code:
        this->r_measurement_covariance_mat = mat_input;
        break;
    default:
        break;
    }
    return *this;
}

ExtendedKalmanFilterWHX &ExtendedKalmanFilterWHX::DefaultSet(float update_time)
{
    this->p_state_covariance_mat << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;
    this->q_process_covariance_mat = Eigen::Matrix4f::Identity();
    this->h_measurement_mat << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;
    this->f_state_transistion_mat << 1, 0, update_time, 0,
        0, 1, 0, update_time,
        0, 0, 1, 0,
        0, 0, 0, 1;
    this->r_measurement_covariance_mat << 0.01, 0, 0, 0,
        0, 0.01, 0, 0,
        0, 0, 0.01, 0,
        0, 0, 0, 0.01;
    return *this;
}

ExtendedKalmanFilterWHX &ExtendedKalmanFilterWHX::Init(Eigen::Vector4f mat_input)
{
    this->initial_vec = mat_input;
    this->is_init = true;
    return *this;
}

void ExtendedKalmanFilterWHX::Predict()
{
    this->initial_vec = this->f_state_transistion_mat * this->initial_vec;
    this->p_state_covariance_mat = this->f_state_transistion_mat * this->p_state_covariance_mat * (this->f_state_transistion_mat.transpose()) + this->q_process_covariance_mat;
}

ExtendedKalmanFilterWHX &ExtendedKalmanFilterWHX::UpdateMeasurement(const Eigen::Vector4f observation)
{
    this->Predict();
    Eigen::Vector4f difference = observation - (this->h_measurement_mat * this->initial_vec);
    Eigen::Matrix4f temp = (this->h_measurement_mat * this->p_state_covariance_mat * this->h_measurement_mat.transpose()) + this->r_measurement_covariance_mat;
    this->k_kalman_gain_mat = this->p_state_covariance_mat * this->h_measurement_mat.transpose() * temp.inverse();
    this->initial_vec = this->initial_vec + (this->k_kalman_gain_mat * difference);
    Eigen::Matrix4f identity = Eigen::Matrix4f::Identity();
    this->p_state_covariance_mat = (identity - this->k_kalman_gain_mat * this->h_measurement_mat) * this->p_state_covariance_mat;
    return *this;
}

Eigen::Vector4f ExtendedKalmanFilterWHX::GetResult()
{
    return this->initial_vec;
}