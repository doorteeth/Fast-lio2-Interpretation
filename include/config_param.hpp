#pragma once
#include <string>
#include <vector>

class ConfigParam
{
public:
    static ConfigParam &getInstance()
    {
        static ConfigParam instance;
        return instance;
    }
    /*** Time Log Variables ***/
    double kdtree_incremental_time = 0.0, kdtree_search_time = 0.0, kdtree_delete_time = 0.0;
    double T1[720000], s_plot[720000], s_plot2[720000], s_plot3[720000], s_plot4[720000], s_plot5[720000], s_plot6[720000], s_plot7[720000], s_plot8[720000], s_plot9[720000], s_plot10[720000], s_plot11[720000];
    double match_time = 0, solve_time = 0, solve_const_H_time = 0;
    int kdtree_size_st = 0, kdtree_size_end = 0, add_point_size = 0, kdtree_delete_counter = 0;
    bool runtime_pos_log = false, pcd_save_en = false, time_sync_en = false, extrinsic_est_en = true, path_en = true;
    float res_last[100000] = {0.0};
    float DET_RANGE = 300.0f;
    const float MOV_THRESHOLD = 1.5f;
    double time_diff_lidar_to_imu = 0.0;
    double res_mean_last = 0.05, total_residual = 0.0;
    double last_timestamp_lidar = 0, last_timestamp_imu = -1.0;
    double gyr_cov = 0.1, acc_cov = 0.1, b_gyr_cov = 0.0001, b_acc_cov = 0.0001;
    double filter_size_corner_min = 0, filter_size_surf_min = 0, filter_size_map_min = 0, fov_deg = 0;
    double cube_len = 0;
    int effct_feat_num = 0, time_log_counter = 0, scan_count = 0, publish_count = 0;
    int iterCount = 0, feats_down_size = 0, NUM_MAX_ITERATIONS = 0, laserCloudValidNum = 0, pcd_save_interval = -1, pcd_index = 0;
    bool point_selected_surf[100000] = {0};
    bool lidar_pushed, flg_first_scan = true, flg_exit = false, flg_EKF_inited;
    bool scan_pub_en = false, dense_pub_en = false, scan_body_pub_en = false;
    int lidar_type;

    std::string root_dir = ROOT_DIR;
    std::string map_file_path, lid_topic, imu_topic;
    std::vector<double> extrinT = {0.0, 0.0, 0.0};
    std::vector<double> extrinR;

private:
    ConfigParam()
    {
    }

    ConfigParam(const ConfigParam &) = delete;
    ConfigParam &operator=(const ConfigParam &) = delete;
};