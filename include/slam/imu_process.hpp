#pragma once

#include "data_type/base_data.hpp"
#include "data_type/eigen_type.hpp"
#include <Eigen/Dense>
#include <deque>
#include <fast_lio/Pose6D.h>
#include <fstream>
#include <ostream>
#include <pcl/point_cloud.h>
#include <sensor_msgs/Imu.h>
class IMUProcess
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    IMUProcess();
    ~IMUProcess();

    void Reset();
    void Reset(double start_timestamp, const sensor_msgs::ImuConstPtr &lastimu);
    void set_extrinsic(const V3D &transl, const M3D &rot);
    void set_extrinsic(const V3D &transl);
    void set_extrinsic(const MD(4, 4) & T);
    void set_gyr_cov(const V3D &scaler);
    void set_acc_cov(const V3D &scaler);
    void set_gyr_bias_cov(const V3D &b_g);
    void set_acc_bias_cov(const V3D &b_a);
    Eigen::Matrix<double, 12, 12> Q;

    std::ofstream fout_imu;
    V3D cov_acc;
    V3D cov_gyr;
    V3D cov_acc_scale;
    V3D cov_gyr_scale;
    V3D cov_bias_gyr;
    V3D cov_bias_acc;
    double first_lidar_time;
    int lidar_type;

private:
    PointCloudXYZI::Ptr cur_pcl_un_;
    sensor_msgs::ImuConstPtr last_imu_;
    std::deque<sensor_msgs::ImuConstPtr> v_imu_;
    std::vector<fast_lio::Pose6D> IMUpose;
    std::vector<M3D> v_rot_pcl_;
    M3D Lidar_R_wrt_IMU;
    V3D Lidar_T_wrt_IMU;
    V3D mean_acc;
    V3D mean_gyr;
    V3D angvel_last;
    V3D acc_s_last;
    double start_timestamp_;
    double last_lidar_end_time_;
    int init_iter_num = 1;
    bool b_first_frame_ = true;
    bool imu_need_init_ = true;
};