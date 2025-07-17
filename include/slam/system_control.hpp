#include "config_param.hpp"
#include "data_type/base_data.hpp"
#include "data_type/eigen_type.hpp"
#include "imu_process.hpp"
#include "preprocess.hpp"
#include <memory>
#include <nav_msgs/Path.h>
#include <pcl/filters/voxel_grid.h>
#include <ros/ros.h>
#include <string.h>

class SystemControl
{
public:
    SystemControl(ros::NodeHandlePtr nhp);

private:
    void InitParam();

private:
    double cube_len = 0, HALF_FOV_COS = 0, FOV_DEG = 0, total_distance = 0, lidar_end_time = 0, first_lidar_time = 0.0;
    bool point_selected_surf[100000] = {0};
    float res_last[100000] = {0.0};
    M3D Lidar_R_wrt_IMU;
    V3D Lidar_T_wrt_IMU;

    pcl::VoxelGrid<PointType> downSizeFilterSurf;
    pcl::VoxelGrid<PointType> downSizeFilterMap;

    ros::NodeHandlePtr node_ptr_;
    nav_msgs::Path path;

    std::shared_ptr<Preprocess> p_pre_ptr_;
    std::shared_ptr<IMUProcess> p_imu_ptr_;
};