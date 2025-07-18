#include "slam/imu_process.hpp"

void IMUProcess::set_extrinsic(const MD(4, 4) & T)
{
    Lidar_T_wrt_IMU = T.block<3, 1>(0, 3);
    Lidar_R_wrt_IMU = T.block<3, 3>(0, 0);
}

void IMUProcess::set_extrinsic(const V3D &transl)
{
    Lidar_T_wrt_IMU = transl;
    Lidar_R_wrt_IMU.setIdentity();
}

void IMUProcess::set_extrinsic(const V3D &transl, const M3D &rot)
{
    Lidar_T_wrt_IMU = transl;
    Lidar_R_wrt_IMU = rot;
}

void IMUProcess::set_gyr_cov(const V3D &scaler)
{
    cov_gyr_scale = scaler;
}

void IMUProcess::set_acc_cov(const V3D &scaler)
{
    cov_acc_scale = scaler;
}

void IMUProcess::set_gyr_bias_cov(const V3D &b_g)
{
    cov_bias_gyr = b_g;
}

void IMUProcess::set_acc_bias_cov(const V3D &b_a)
{
    cov_bias_acc = b_a;
}

IMUProcess::IMUProcess() : b_first_frame_(true), imu_need_init_(true), start_timestamp_(-1)
{
    init_iter_num = 1;
    Q = process_noise_cov();
    cov_acc = V3D(0.1, 0.1, 0.1);
    cov_gyr = V3D(0.1, 0.1, 0.1);
    cov_bias_gyr = V3D(0.0001, 0.0001, 0.0001);
    cov_bias_acc = V3D(0.0001, 0.0001, 0.0001);
    mean_acc = V3D(0, 0, -1.0);
    mean_gyr = V3D(0, 0, 0);
    angvel_last = Zero3d;
    Lidar_T_wrt_IMU = Zero3d;
    Lidar_R_wrt_IMU = Eye3d;
    last_imu_.reset(new sensor_msgs::Imu());
}