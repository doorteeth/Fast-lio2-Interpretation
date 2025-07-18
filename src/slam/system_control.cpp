#include "slam/system_control.hpp"

SystemControl::SystemControl(ros::NodeHandlePtr nhp) : node_ptr_(nhp)
{
    p_pre_ptr_ = std::make_shared<Preprocess>();
    p_imu_ptr_ = std::make_shared<IMUProcess>();

    InitParam();
    p_pre_ptr_->lidar_type = ConfigParam::getInstance().lidar_type;
    path.header.stamp = ros::Time::now();
    path.header.frame_id = "camera_init";

    int effect_feat_num = 0, frame_num = 0;
    double deltaT, deltaR, aver_time_consu = 0, aver_time_icp = 0, aver_time_match = 0, aver_time_incre = 0, aver_time_solve = 0, aver_time_const_H_time = 0;
    bool flg_EKF_converged, EKF_stop_flg = 0;

    // 对原始视野角度 fov_deg 进行扩展（+10 度），但限制最大值为 179.9 度（接近 180 度，但不到 180 度，可能是为了避免数学计算中的奇点问题）
    FOV_DEG = (ConfigParam::getInstance().fov_deg + 10.0) > 179.9 ? 179.9 : (ConfigParam::getInstance().fov_deg + 10.0);
    // 在传感器数据处理中（如点云滤波、视场范围判断），常用半视野角的余弦值来快速判断某个方向是否在视野范围内（通过向量点积比较）
    HALF_FOV_COS = cos((FOV_DEG) * 0.5 * M_PI / 180.0);
    std::fill(std::begin(point_selected_surf), std::end(point_selected_surf), true);
    std::fill(std::begin(res_last), std::end(res_last), -1000.0f);

    downSizeFilterSurf.setLeafSize(ConfigParam::getInstance().filter_size_surf_min, ConfigParam::getInstance().filter_size_surf_min, ConfigParam::getInstance().filter_size_surf_min);
    downSizeFilterMap.setLeafSize(ConfigParam::getInstance().filter_size_map_min, ConfigParam::getInstance().filter_size_map_min, ConfigParam::getInstance().filter_size_map_min);

    Lidar_T_wrt_IMU << VEC_FROM_ARRAY(ConfigParam::getInstance().extrinT);
    Lidar_R_wrt_IMU << MAT_FROM_ARRAY(ConfigParam::getInstance().extrinR);
    p_imu_ptr_->set_extrinsic(Lidar_T_wrt_IMU, Lidar_R_wrt_IMU);
    p_imu_ptr_->set_gyr_cov(V3D(ConfigParam::getInstance().gyr_cov, ConfigParam::getInstance().gyr_cov, ConfigParam::getInstance().gyr_cov));
    p_imu_ptr_->set_acc_cov(V3D(ConfigParam::getInstance().acc_cov, ConfigParam::getInstance().acc_cov, ConfigParam::getInstance().acc_cov));
    p_imu_ptr_->set_gyr_bias_cov(V3D(ConfigParam::getInstance().b_gyr_cov, ConfigParam::getInstance().b_gyr_cov, ConfigParam::getInstance().b_gyr_cov));
    p_imu_ptr_->set_acc_bias_cov(V3D(ConfigParam::getInstance().b_acc_cov, ConfigParam::getInstance().b_acc_cov, ConfigParam::getInstance().b_acc_cov));
}

void SystemControl::InitParam()
{
    node_ptr_->param<bool>("publish/path_en", ConfigParam::getInstance().path_en, true);
    node_ptr_->param<bool>("publish/scan_publish_en", ConfigParam::getInstance().scan_pub_en, true);
    node_ptr_->param<bool>("publish/dense_publish_en", ConfigParam::getInstance().dense_pub_en, true);
    node_ptr_->param<bool>("publish/scan_bodyframe_pub_en", ConfigParam::getInstance().scan_body_pub_en, true);
    node_ptr_->param<int>("max_iteration", ConfigParam::getInstance().NUM_MAX_ITERATIONS, 4);
    node_ptr_->param<std::string>("map_file_path", ConfigParam::getInstance().map_file_path, "");
    node_ptr_->param<std::string>("common/lid_topic", ConfigParam::getInstance().lid_topic, "/livox/lidar");
    node_ptr_->param<std::string>("common/imu_topic", ConfigParam::getInstance().imu_topic, "/livox/imu");
    node_ptr_->param<bool>("common/time_sync_en", ConfigParam::getInstance().time_sync_en, false);
    node_ptr_->param<double>("common/time_offset_lidar_to_imu", ConfigParam::getInstance().time_diff_lidar_to_imu, 0.0);
    node_ptr_->param<double>("filter_size_corner", ConfigParam::getInstance().filter_size_corner_min, 0.5);
    node_ptr_->param<double>("filter_size_surf", ConfigParam::getInstance().filter_size_surf_min, 0.5);
    node_ptr_->param<double>("filter_size_map", ConfigParam::getInstance().filter_size_map_min, 0.5);
    node_ptr_->param<double>("cube_side_length", ConfigParam::getInstance().cube_len, 200);
    node_ptr_->param<float>("mapping/det_range", ConfigParam::getInstance().DET_RANGE, 300.f);
    node_ptr_->param<double>("mapping/fov_degree", ConfigParam::getInstance().fov_deg, 180);
    node_ptr_->param<double>("mapping/gyr_cov", ConfigParam::getInstance().gyr_cov, 0.1);
    node_ptr_->param<double>("mapping/acc_cov", ConfigParam::getInstance().acc_cov, 0.1);
    node_ptr_->param<double>("mapping/b_gyr_cov", ConfigParam::getInstance().b_gyr_cov, 0.0001);
    node_ptr_->param<double>("mapping/b_acc_cov", ConfigParam::getInstance().b_acc_cov, 0.0001);
    node_ptr_->param<double>("preprocess/blind", p_pre_ptr_->blind, 0.01);
    node_ptr_->param<int>("preprocess/lidar_type", ConfigParam::getInstance().lidar_type, AVIA);
    node_ptr_->param<int>("preprocess/scan_line", p_pre_ptr_->N_SCANS, 16);
    node_ptr_->param<int>("preprocess/timestamp_unit", p_pre_ptr_->time_unit, US);
    node_ptr_->param<int>("preprocess/scan_rate", p_pre_ptr_->SCAN_RATE, 10);
    node_ptr_->param<int>("point_filter_num", p_pre_ptr_->point_filter_num, 2);
    node_ptr_->param<bool>("feature_extract_enable", p_pre_ptr_->feature_enabled, false);
    node_ptr_->param<bool>("runtime_pos_log_enable", ConfigParam::getInstance().runtime_pos_log, 0);
    node_ptr_->param<bool>("mapping/extrinsic_est_en", ConfigParam::getInstance().extrinsic_est_en, true);
    node_ptr_->param<bool>("pcd_save/pcd_save_en", ConfigParam::getInstance().pcd_save_en, false);
    node_ptr_->param<int>("pcd_save/interval", ConfigParam::getInstance().pcd_save_interval, -1);
    node_ptr_->param<std::vector<double>>("mapping/extrinsic_T", ConfigParam::getInstance().extrinT, std::vector<double>());
    node_ptr_->param<std::vector<double>>("mapping/extrinsic_R", ConfigParam::getInstance().extrinR, std::vector<double>());
}
