/******************************************************************************
 * @file        lidarMapping.cpp
 * @brief       Declaration of the lio module class and related functions.
 *
 * @author      Yunxiang He
 * @date        2024-11-27
 * @version     2.0.0
 *
 * @details     This file contains the definition and assignment of the lidar-inertial-odometry in the slam module,
 *              and initialize the global pose.
 ******************************************************************************/

#include "arps/lidarMapping.h"

namespace arps {
    namespace slam {
        namespace arps_lidar_inertial_odometry {
            // // 用于标记退出状态
            // std::atomic<bool> stop_flag(false);
            //
            // // 信号处理函数
            // void signalHandler(int signal) {
            //     if (signal == SIGINT) {
            //         std::cout << "\nSIGINT (Ctrl+C) caught. Exiting...\n";
            //         stop_flag = true;
            //     }
            // }

            LidarMapping::LidarMapping(const rclcpp::NodeOptions &node_options)
                : Node("laserMapping", node_options) {
                RCLCPP_INFO(get_logger(), "\033[1;32m----> Laser Mapping Started.\033[0m");

                p_pre = std::make_shared<Preprocess>();
                p_imu = std::make_shared<ImuProcess>();

                declare_parameter<std::string>("config_file",
                                               "/home/tsm/gb_car_ws/src/mapping/arps_lidar_inertial_odometry/config/mid360.yaml");
                get_parameter("config_file", config_file_path_);
                getParameters(config_file_path_);

                p_pre->lidar_type = lidar_type;

                path.header.stamp = get_ros_time(lidar_end_time);
                path.header.frame_id = "camera_init";

                callbackGroupLidar = create_callback_group(
                    rclcpp::CallbackGroupType::MutuallyExclusive);
                callbackGroupImu = create_callback_group(
                    rclcpp::CallbackGroupType::MutuallyExclusive);

                auto lidarOpt = rclcpp::SubscriptionOptions();
                lidarOpt.callback_group = callbackGroupLidar;
                auto imuOpt = rclcpp::SubscriptionOptions();
                imuOpt.callback_group = callbackGroupImu;

                if (p_pre->lidar_type == AVIA) {
                    RCLCPP_INFO(get_logger(), "lidar type is AVIA");
                    livox_sub_ = create_subscription<livox_ros_driver2::msg::CustomMsg>(
                        lid_topic, 100, std::bind(&LidarMapping::livox_pcl_cbk, this, std::placeholders::_1),
                        lidarOpt);
                } else {
                    RCLCPP_INFO(get_logger(), "lidar type is normal");
                    lidar_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
                        lid_topic, 100, std::bind(&LidarMapping::standard_pcl_cbk, this, std::placeholders::_1),
                        lidarOpt);
                }
                imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
                    imu_topic, 2000, std::bind(&LidarMapping::imu_cbk, this, std::placeholders::_1), imuOpt);

                pubSyncImu = create_publisher<sensor_msgs::msg::Imu>("/imu/data", 1000);

                pubLaserCloudFullRes = create_publisher<sensor_msgs::msg::PointCloud2>
                        ("/cloud_registered", 100000);
                pubLaserCloudFullRes_body = create_publisher<sensor_msgs::msg::PointCloud2>
                        ("/cloud_registered_body", 100000);
                pubLaserCloudEffect = create_publisher<sensor_msgs::msg::PointCloud2>
                        ("/cloud_effected", 100000);
                pubLaserCloudMap = create_publisher<sensor_msgs::msg::PointCloud2>
                        ("/Laser_map", 100000);
                pubOdomAftMapped = create_publisher<nav_msgs::msg::Odometry>
                        ("/aft_mapped_to_init", 100000);
                pubPath = create_publisher<nav_msgs::msg::Path>
                        ("/lio_path", 100000);

                initialization();

                // process_timer_ = create_wall_timer(
                //     std::chrono::milliseconds(static_cast<int>(100.)),
                //     std::bind(&LidarMapping::run, this));
            }

            void LidarMapping::getParameters(std::string &config_path) {
                YAML::Node config = YAML::LoadFile(config_path);
                if (!config) {
                    RCLCPP_WARN(this->get_logger(), "Failed to load config file %s", config_path.c_str());
                    return;
                }

                max_iteration = config["common"]["max_iteration"].as<int>();
                point_filter_num = config["common"]["point_filter_num"].as<int>();
                laser_point_cov = config["common"]["laser_point_cov"].as<double>();
                lid_topic = config["common"]["lid_topic"].as<std::string>();
                imu_topic = config["common"]["imu_topic"].as<std::string>();
                filter_size_surf = config["mapping"]["filter_size_surf"].as<double>();
                filter_size_map = config["mapping"]["filter_size_map"].as<double>();
                cube_side_length = config["common"]["cube_side_length"].as<double>();
                imu_int_frame = config["mapping"]["imu_int_frame"].as<int>();
                imu_en = config["mapping"]["imu_en"].as<bool>();
                DET_RANGE = config["mapping"]["det_range"].as<float>();
                gyr_cov = config["mapping"]["gyr_cov"].as<double>();
                acc_cov = config["mapping"]["acc_cov"].as<double>();
                b_gyr_cov = config["mapping"]["b_gyr_cov"].as<double>();
                b_acc_cov = config["mapping"]["b_acc_cov"].as<double>();
                blind = config["preprocess"]["blind"].as<double>();
                lidar_type = config["preprocess"]["lidar_type"].as<int>();
                scan_line = config["preprocess"]["scan_line"].as<int>();
                timestamp_unit = config["preprocess"]["timestamp_unit"].as<int>();
                feature_extract_en = config["preprocess"]["feature_extract_en"].as<bool>();
                cut_frame_num = config["initialization"]["cut_frame_num"].as<int>();
                orig_odom_freq = config["initialization"]["orig_odom_freq"].as<int>();
                online_refine_time = config["initialization"]["online_refine_time"].as<double>();
                mean_acc_norm = config["initialization"]["mean_acc_norm"].as<double>();
                max_init_count = config["initialization"]["max_init_count"].as<int>();
                path_en = config["publish"]["path_en"].as<bool>();
                scan_pub_en = config["publish"]["scan_publish_en"].as<bool>();
                dense_pub_en = config["publish"]["dense_publish_en"].as<bool>();
                scan_body_pub_en = config["publish"]["scan_bodyframe_pub_en"].as<bool>();
                pcd_save_en = config["pcd_save"]["pcd_save_en"].as<bool>();
                interval = config["pcd_save"]["interval"].as<int>();
                extrinT = config["mapping"]["extrinsic_T"].as<std::vector<double>>();
                extrinR = config["mapping"]["extrinsic_R"].as<std::vector<double>>();
                extrinRPY = config["mapping"]["extrinsic_RPY"].as<std::vector<double>>();

                RCLCPP_INFO(get_logger(), "Load Parameters complete");
            }

            void LidarMapping::initialization() {

                lidar_buffer.clear();
                imu_buffer.clear();

                featsFromMap.reset(new PointCloudXYZI());
                feats_undistort.reset(new PointCloudXYZI());
                feats_down_body.reset(new PointCloudXYZI());
                feats_down_world.reset(new PointCloudXYZI());
                normvec.reset(new PointCloudXYZI(100000, 1));
                laserCloudOri.reset(new PointCloudXYZI(100000, 1));
                corr_normvect.reset(new PointCloudXYZI(100000, 1));
                pcl_wait_save.reset(new PointCloudXYZI());

                last_rot = M3D::Identity();
                XAxisPoint_body = V3F(LIDAR_SP_LEN, 0.0, 0.0);
                XAxisPoint_world = V3F(LIDAR_SP_LEN, 0.0, 0.0);
                position_last = V3D::Zero();
                last_odom = V3D::Zero();

                Lidar_T_wrt_IMU = V3D::Zero();
                Lidar_R_wrt_IMU = M3D::Identity();
                ExtrinRPY = M3D::Identity();

                memset(point_selected_surf, true, sizeof(point_selected_surf));
                memset(res_last, -1000.0f, sizeof(res_last));
                downSizeFilterSurf.setLeafSize(filter_size_surf, filter_size_surf, filter_size_surf);
                downSizeFilterMap.setLeafSize(filter_size_map, filter_size_map, filter_size_map);
                memset(point_selected_surf, true, sizeof(point_selected_surf));
                memset(res_last, -1000.0f, sizeof(res_last));

                Lidar_T_wrt_IMU << VEC_FROM_ARRAY(extrinT);
                Lidar_R_wrt_IMU << MAT_FROM_ARRAY(extrinR);
                ExtrinRPY = eulerToRotationMatrix(extrinRPY[0] * M_PI / 180.0,
                                                  extrinRPY[1] * M_PI / 180.0,
                                                  extrinRPY[2] * M_PI / 180.0);

                p_imu->set_extrinsic(Lidar_T_wrt_IMU, Lidar_R_wrt_IMU);
                p_imu->set_gyr_cov_scale(V3D(gyr_cov, gyr_cov, gyr_cov));
                p_imu->set_acc_cov_scale(V3D(acc_cov, acc_cov, acc_cov));
                p_imu->set_gyr_bias_cov(V3D(b_gyr_cov, b_gyr_cov, b_gyr_cov));
                p_imu->set_acc_bias_cov(V3D(b_acc_cov, b_acc_cov, b_acc_cov));

                G.setZero();
                H_T_H.setZero();
                I_STATE.setIdentity();

                CleanFmt = Eigen::IOFormat(4, 1, ", ", " ", "[", "]");
                // std::cout << ExtrinRPY.format(CleanFmt) << std::endl;

                br = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

                RCLCPP_INFO_STREAM(get_logger(), "Initialization complete.");
            }

            float calc_dist(PointType p1, PointType p2) {
                float d = (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z);
                return d;
            }

            void LidarMapping::calcBodyVar(Eigen::Vector3d &pb, const float range_inc,
                                           const float degree_inc, Eigen::Matrix3d &var) {
                float range = sqrt(pb[0] * pb[0] + pb[1] * pb[1] + pb[2] * pb[2]);
                float range_var = range_inc * range_inc;
                Eigen::Matrix2d direction_var;
                direction_var << pow(sin(DEG2RAD(degree_inc)), 2), 0, 0,
                        pow(sin(DEG2RAD(degree_inc)), 2);
                Eigen::Vector3d direction(pb);
                direction.normalize();
                Eigen::Matrix3d direction_hat;
                direction_hat << 0, -direction(2), direction(1), direction(2), 0,
                        -direction(0), -direction(1), direction(0), 0;
                Eigen::Vector3d base_vector1(1, 1,
                                             -(direction(0) + direction(1)) / direction(2));
                base_vector1.normalize();
                Eigen::Vector3d base_vector2 = base_vector1.cross(direction);
                base_vector2.normalize();
                Eigen::Matrix<double, 3, 2> N;
                N << base_vector1(0), base_vector2(0), base_vector1(1), base_vector2(1),
                        base_vector1(2), base_vector2(2);
                Eigen::Matrix<double, 3, 2> A = range * direction_hat * N;
                var = direction * range_var * direction.transpose() +
                      A * direction_var * A.transpose();
            }

            Eigen::Matrix3d LidarMapping::eulerToRotationMatrix(double roll, double pitch, double yaw) {
                // 定义旋转矩阵
                Eigen::Matrix3d rotationMatrix;

                // 计算每个分量的正弦和余弦值
                double cos_roll = cos(roll);
                double sin_roll = sin(roll);
                double cos_pitch = cos(pitch);
                double sin_pitch = sin(pitch);
                double cos_yaw = cos(yaw);
                double sin_yaw = sin(yaw);

                // 填充旋转矩阵
                rotationMatrix(0, 0) = cos_yaw * cos_pitch;
                rotationMatrix(0, 1) = cos_yaw * sin_pitch * sin_roll - sin_yaw * cos_roll;
                rotationMatrix(0, 2) = cos_yaw * sin_pitch * cos_roll + sin_yaw * sin_roll;
                rotationMatrix(1, 0) = sin_yaw * cos_pitch;
                rotationMatrix(1, 1) = sin_yaw * sin_pitch * sin_roll + cos_yaw * cos_roll;
                rotationMatrix(1, 2) = sin_yaw * sin_pitch * cos_roll - cos_yaw * sin_roll;
                rotationMatrix(2, 0) = -sin_pitch;
                rotationMatrix(2, 1) = cos_pitch * sin_roll;
                rotationMatrix(2, 2) = cos_pitch * cos_roll;

                return rotationMatrix;
            }

            void LidarMapping::pointBodyToWorld(PointType const *const pi, PointType *const po) {
                V3D p_body(pi->x, pi->y, pi->z);
                V3D p_global(state.rot_end * (Lidar_R_wrt_IMU * p_body + Lidar_T_wrt_IMU) + state.pos_end);

                po->x = p_global(0);
                po->y = p_global(1);
                po->z = p_global(2);
                po->normal_x = pi->normal_x;
                po->normal_y = pi->normal_y;
                po->normal_z = pi->normal_z;
                po->intensity = pi->intensity;
            }

            template<typename T>
            void LidarMapping::pointBodyToWorld(const Eigen::Matrix<T, 3, 1> &pi, Eigen::Matrix<T, 3, 1> &po) {
                V3D p_body(pi[0], pi[1], pi[2]);
                V3D p_global(state.rot_end * (Lidar_R_wrt_IMU * p_body + Lidar_T_wrt_IMU) + state.pos_end);
                po[0] = p_global(0);
                po[1] = p_global(1);
                po[2] = p_global(2);
            }

            void LidarMapping::RGBpointBodyToWorld(PointType const *const pi, PointTypeRGB *const po) {
                V3D p_body(pi->x, pi->y, pi->z);
                V3D p_global(state.rot_end * (Lidar_R_wrt_IMU * p_body + Lidar_T_wrt_IMU) + state.pos_end);
                po->x = p_global(0);
                po->y = p_global(1);
                po->z = p_global(2);
                po->r = pi->normal_x;
                po->g = pi->normal_y;
                po->b = pi->normal_z;

                float intensity = pi->intensity;
                intensity = intensity - floor(intensity);

                int reflection_map = intensity * 10000;
            }

            void LidarMapping::points_cache_collect() {
                PointVector points_history;
                ikdtree.acquire_removed_points(points_history);
                points_cache_size = points_history.size();
                for (int i = 0; i < points_history.size(); i++) _featsArray->push_back(points_history[i]);
            }

            void LidarMapping::lasermap_fov_segment() {
                cub_needrm.clear();

                pointBodyToWorld(XAxisPoint_body, XAxisPoint_world);
                V3D pos_LiD = state.pos_end;

                if (!Localmap_Initialized) {
                    for (int i = 0; i < 3; i++) {
                        LocalMap_Points.vertex_min[i] = pos_LiD(i) - cube_len / 2.0;
                        LocalMap_Points.vertex_max[i] = pos_LiD(i) + cube_len / 2.0;
                    }
                    Localmap_Initialized = true;
                    return;
                }

                float dist_to_map_edge[3][2];
                bool need_move = false;
                for (int i = 0; i < 3; i++) {
                    dist_to_map_edge[i][0] = fabs(pos_LiD(i) - LocalMap_Points.vertex_min[i]);
                    dist_to_map_edge[i][1] = fabs(pos_LiD(i) - LocalMap_Points.vertex_max[i]);
                    if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE ||
                        dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE)
                        need_move = true;
                }
                if (!need_move) return;
                BoxPointType New_LocalMap_Points, tmp_boxpoints;
                New_LocalMap_Points = LocalMap_Points;
                float mov_dist = max((cube_len - 2.0 * MOV_THRESHOLD * DET_RANGE) * 0.5 * 0.9,
                                     double(DET_RANGE * (MOV_THRESHOLD - 1)));
                for (int i = 0; i < 3; i++) {
                    tmp_boxpoints = LocalMap_Points;
                    if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE) {
                        New_LocalMap_Points.vertex_max[i] -= mov_dist;
                        New_LocalMap_Points.vertex_min[i] -= mov_dist;
                        tmp_boxpoints.vertex_min[i] = LocalMap_Points.vertex_max[i] - mov_dist;
                        cub_needrm.push_back(tmp_boxpoints);
                    } else if (dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE) {
                        New_LocalMap_Points.vertex_max[i] += mov_dist;
                        New_LocalMap_Points.vertex_min[i] += mov_dist;
                        tmp_boxpoints.vertex_max[i] = LocalMap_Points.vertex_min[i] + mov_dist;
                        cub_needrm.push_back(tmp_boxpoints);
                    }
                }
                LocalMap_Points = New_LocalMap_Points;
                points_cache_collect();
            }


            void LidarMapping::livox_pcl_cbk(const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr msg) {
                std::lock_guard<std::mutex> lock1(lidar_mtx);
                // RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "get lidar msg");
                if (get_time_in_sec(msg->header.stamp) < last_timestamp_lidar) {
                    PCL_WARN("lidar loop back, clear buffer");
                    lidar_buffer.clear();
                    time_buffer.clear();
                }
                last_timestamp_lidar = get_time_in_sec(msg->header.stamp);

                PointCloudXYZI::Ptr tmp_ptr(new PointCloudXYZI());
                PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
                p_pre->process(msg, tmp_ptr);

                //    ptr->resize(tmp_ptr->points.size());
                std::size_t i = 0;
                for (const auto &p: tmp_ptr->points) {
                    auto point = p;
                    V3D p_raw(p.x, p.y, p.z);
                    V3D p_h(ExtrinRPY * p_raw);
                    point.x = p_h.x();
                    point.y = p_h.y();
                    point.z = p_h.z();

                    //        if ((point.x < 0 && point.x > -3.0) && (abs(point.y) <= 0.6)) continue;
                    //        if ((point.x >= 0 && point.x <= 4.0) && (abs(point.y) <= 4.0)  && (point.z <= -0.1))
                    //            continue;

                    ptr->points.push_back(point);

                    i++;
                }

                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "point size : %lu",
                                     ptr->points.size());

                lidar_buffer.push_back(ptr);
                time_buffer.push_back(last_timestamp_lidar);
                // sig_buffer.notify_all();
            }

            void LidarMapping::standard_pcl_cbk(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg) {
                std::lock_guard<std::mutex> lock1(lidar_mtx);
                if (get_time_in_sec(msg->header.stamp) < last_timestamp_lidar) {
                    PCL_ERROR("lidar loop back, clear Lidar buffer.");
                    lidar_buffer.clear();
                    time_buffer.clear();
                }
                last_timestamp_lidar = get_time_in_sec(msg->header.stamp);
                PointCloudXYZI::Ptr tmp_ptr(new PointCloudXYZI());
                PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
                p_pre->process(msg, tmp_ptr);

                ptr->resize(tmp_ptr->points.size());
                std::size_t i = 0;
                for (const auto &p: tmp_ptr->points) {
                    auto point = p;
                    V3D p_raw(p.x, p.y, p.z);
                    V3D p_h(ExtrinRPY * p_raw);
                    point.x = p_h.x();
                    point.y = p_h.y();
                    point.z = p_h.z();

                    //        if ((point.x < 0 && point.x > -3.0) && (abs(point.y) <= 0.6)) continue;
                    //        if ((point.x >= 0 && point.x <= 4.0) && (abs(point.y) <= 4.0)  && (point.z <= -0.1))
                    //            continue;

                    ptr->points[i] = point;

                    i++;
                }

                // RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "normal point size : %lu",
                                     // ptr->points.size());

                lidar_buffer.push_back(ptr);
                time_buffer.push_back(last_timestamp_lidar);
                // sig_buffer.notify_all();
            }

            void LidarMapping::imu_cbk(const sensor_msgs::msg::Imu::ConstSharedPtr msg_in) {
                std::lock_guard<std::mutex> lock1(imu_mtx);
                // RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "imu msg");
                sensor_msgs::msg::Imu::SharedPtr msg(new sensor_msgs::msg::Imu(*msg_in));
                double timestamp = get_time_in_sec(msg->header.stamp);

                V3D ang_raw(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
                V3D ang_h(ExtrinRPY * ang_raw);
                msg->angular_velocity.x = ang_h.x();
                msg->angular_velocity.y = ang_h.y();
                msg->angular_velocity.z = ang_h.z();

                V3D acc_raw(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
                V3D acc_h(ExtrinRPY * acc_raw);
                msg->linear_acceleration.x = acc_h.x();
                msg->linear_acceleration.y = acc_h.y();
                msg->linear_acceleration.z = acc_h.z();

                if (last_timestamp_imu > 0.0 && timestamp < last_timestamp_imu) {
                    RCLCPP_ERROR(get_logger(), "imu loop back \n");
                    // sig_buffer.notify_all();
                    return;
                }

                last_timestamp_imu = timestamp;
                imu_buffer.push_back(msg);

                pubSyncImu->publish(*msg);

                // sig_buffer.notify_all();
            }

            bool LidarMapping::sync_packages(MeasureGroup &meas) {
                // if (!m_imu_en) {
                //     if (!lidar_buffer.empty()) {
                //         meas.lidar = lidar_buffer.front();
                //         meas.lidar_beg_time = time_buffer.front();
                //         lidar_buffer.pop_front();
                //         time_buffer.pop_front();
                //         return true;
                //     }
                //
                //     return false;
                // }

                if (lidar_buffer.empty() || imu_buffer.empty()) {
                    return false;
                    RCLCPP_ERROR(get_logger(), "lidar or imu buffer is empty");
                }

                // RCLCPP_INFO(get_logger(), "sync_packages, lidar buf size : %lu, imu buf size : %lu \n", lidar_buffer.size(), imu_buffer.size());

                /** push a lidar scan **/
                if (!lidar_pushed) {
                    meas.lidar = lidar_buffer.front();

                    if (meas.lidar->points.size() <= 1) {
                        RCLCPP_WARN(get_logger(), "Too few input point cloud!\n");
                        lidar_buffer.pop_front();
                        time_buffer.pop_front();
                        return false;
                    }

                    meas.lidar_beg_time = time_buffer.front(); //unit:s
                    lidar_end_time = meas.lidar_beg_time + meas.lidar->points.back().curvature / double(1000); //unit:s

                    lidar_pushed = true;
                }

                if (last_timestamp_imu < lidar_end_time) {
                    RCLCPP_ERROR(get_logger(), "last_timestamp_imu < lidar_end_time , and back is %.4f \n", meas.lidar->points.back().curvature / double(1000));
                    return false;
                }

                /*** push imu data, and pop from imu buffer ***/
                double imu_time = get_time_in_sec(imu_buffer.front()->header.stamp);
                meas.imu.clear();
                while ((!imu_buffer.empty()) && (imu_time < lidar_end_time)) {
                    imu_time = get_time_in_sec(imu_buffer.front()->header.stamp);
                    if (imu_time > lidar_end_time) break;
                    meas.imu.push_back(imu_buffer.front());
                    imu_buffer.pop_front();
                }
                lidar_buffer.pop_front();
                time_buffer.pop_front();
                lidar_pushed = false; // sync one whole lidar scan.
                // sig_buffer.notify_all();
                return true;
            }

            void LidarMapping::map_incremental() {
                PointVector PointToAdd;
                PointVector PointNoNeedDownsample;
                PointToAdd.reserve(feats_down_size);
                PointNoNeedDownsample.reserve(feats_down_size);
                for (int i = 0; i < feats_down_size; i++) {
                    /* transform to world frame */
                    pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
                    /* decide if need add to map */
                    if (!Nearest_Points[i].empty() && flg_EKF_inited) {
                        const PointVector &points_near = Nearest_Points[i];
                        bool need_add = true;
                        BoxPointType Box_of_Point;
                        PointType downsample_result, mid_point;
                        mid_point.x = floor(feats_down_world->points[i].x / filter_size_map) * filter_size_map +
                                      0.5 * filter_size_map;
                        mid_point.y = floor(feats_down_world->points[i].y / filter_size_map) * filter_size_map +
                                      0.5 * filter_size_map;
                        mid_point.z = floor(feats_down_world->points[i].z / filter_size_map) * filter_size_map +
                                      0.5 * filter_size_map;
                        float dist = calc_dist(feats_down_world->points[i], mid_point);
                        if (fabs(points_near[0].x - mid_point.x) > 0.5 * filter_size_map &&
                            fabs(points_near[0].y - mid_point.y) > 0.5 * filter_size_map &&
                            fabs(points_near[0].z - mid_point.z) > 0.5 * filter_size_map) {
                            PointNoNeedDownsample.push_back(feats_down_world->points[i]);
                            continue;
                        }
                        for (int readd_i = 0; readd_i < NUM_MATCH_POINTS; readd_i++) {
                            if (points_near.size() < NUM_MATCH_POINTS) break;
                            if (calc_dist(points_near[readd_i], mid_point) < dist) {
                                need_add = false;
                                break;
                            }
                        }
                        if (need_add) PointToAdd.push_back(feats_down_world->points[i]);
                    } else {
                        PointToAdd.push_back(feats_down_world->points[i]);
                    }
                }

                add_point_size = ikdtree.Add_Points(PointToAdd, true);
                ikdtree.Add_Points(PointNoNeedDownsample, false);
                add_point_size = PointToAdd.size() + PointNoNeedDownsample.size();
            }

            void LidarMapping::publish_frame_world(
                const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &pubLaserCloudFullRes) {
                if (scan_pub_en) {
                    PointCloudXYZI::Ptr laserCloudFullRes(dense_pub_en ? feats_undistort : feats_down_body);
                    int size = laserCloudFullRes->points.size();

                    PointCloudXYZRGB::Ptr laserCloudWorldRGB(new PointCloudXYZRGB(size, 1));
                    PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(size, 1));

                    for (int i = 0; i < size; i++) {
                        pointBodyToWorld(&laserCloudFullRes->points[i],
                                         &laserCloudWorld->points[i]);
                    }

                    sensor_msgs::msg::PointCloud2 laserCloudmsg;

                    pcl::toROSMsg(*laserCloudWorld, laserCloudmsg);

                    laserCloudmsg.header.stamp = get_ros_time(lidar_end_time);
                    laserCloudmsg.header.frame_id = "camera_init";
                    pubLaserCloudFullRes->publish(laserCloudmsg);
                    // publish_count -= PUBFRAME_PERIOD;
                }


                /**************** save map ****************/
                /* 1. make sure you have enough memories
                   2. noted that pcd save will influence the real-time performences **/
                // if (pcd_save_en) {
                //     // boost::filesystem::create_directories(root_dir + "/PCD");
                //     int size = feats_undistort->points.size();
                //     PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(size, 1));
                //     for (int i = 0; i < size; i++) {
                //         pointBodyToWorld(&feats_undistort->points[i], &laserCloudWorld->points[i]);
                //     }
                //
                //     *pcl_wait_save += *laserCloudWorld;
                //     static int scan_wait_num = 0;
                //     scan_wait_num++;
                //     if (pcl_wait_save->size() > 0 && pcd_save_interval > 0 && scan_wait_num >= pcd_save_interval) {
                //         pcd_index++;
                //         all_points_dir = string(root_dir + "/PCD/PCD") + to_string(pcd_index) + string(".pcd");
                //         cout << "current scan saved to " << all_points_dir << endl;
                //         pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
                //         pcl_wait_save->clear();
                //         scan_wait_num = 0;
                //     }
                // }
            }

            void LidarMapping::publish_frame_body(
                const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &pubLaserCloudFullRes_body) {
                PointCloudXYZI::Ptr laserCloudFullRes(feats_undistort);
                sensor_msgs::msg::PointCloud2 laserCloudmsg;
                pcl::toROSMsg(*feats_undistort, laserCloudmsg);
                laserCloudmsg.header.stamp = get_ros_time(lidar_end_time);
                laserCloudmsg.header.frame_id = "camera_init";
                pubLaserCloudFullRes_body->publish(laserCloudmsg);
            }

            void LidarMapping::publish_effect_world(
                const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &pubLaserCloudEffect) {
                PointCloudXYZI::Ptr laserCloudWorld(\
                    new PointCloudXYZI(effect_feat_num, 1));
                for (int i = 0; i < effect_feat_num; i++) {
                    pointBodyToWorld(&laserCloudOri->points[i], &laserCloudWorld->points[i]);
                }
                sensor_msgs::msg::PointCloud2 laserCloudFullRes3;
                pcl::toROSMsg(*laserCloudWorld, laserCloudFullRes3);
                laserCloudFullRes3.header.stamp = get_ros_time(lidar_end_time);
                laserCloudFullRes3.header.frame_id = "camera_init";
                pubLaserCloudEffect->publish(laserCloudFullRes3);
            }

            void LidarMapping::publish_map(
                const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &pubLaserCloudMap) {
                sensor_msgs::msg::PointCloud2 laserCloudMap;
                pcl::toROSMsg(*featsFromMap, laserCloudMap);
                laserCloudMap.header.stamp = get_ros_time(lidar_end_time);
                laserCloudMap.header.frame_id = "camera_init";
                pubLaserCloudMap->publish(laserCloudMap);
            }

            template<typename T>
            void LidarMapping::set_posestamp(T &out) {
                out.position.x = state.pos_end(0);
                out.position.y = state.pos_end(1);
                out.position.z = state.pos_end(2);

                out.orientation.x = geoQuat.x;
                out.orientation.y = geoQuat.y;
                out.orientation.z = geoQuat.z;
                out.orientation.w = geoQuat.w;
            }

            void LidarMapping::publish_odometry(
                const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr &pubOdomAftMapped) {
                odomAftMapped.header.frame_id = "camera_init";
                odomAftMapped.child_frame_id = "aft_mapped";
                odomAftMapped.header.stamp = get_ros_time(lidar_end_time);
                set_posestamp(odomAftMapped.pose.pose);

                pubOdomAftMapped->publish(odomAftMapped);

                geometry_msgs::msg::TransformStamped transformStamped;
                // 设置时间戳和坐标系
                transformStamped.header.stamp = odomAftMapped.header.stamp;
                transformStamped.header.frame_id = "camera_init";
                transformStamped.child_frame_id = "aft_mapped";

                // 设置平移
                transformStamped.transform.translation.x = odomAftMapped.pose.pose.position.x;
                transformStamped.transform.translation.y = odomAftMapped.pose.pose.position.y;
                transformStamped.transform.translation.z = odomAftMapped.pose.pose.position.z;

                // 设置旋转
                transformStamped.transform.rotation.w = odomAftMapped.pose.pose.orientation.w;
                transformStamped.transform.rotation.x = odomAftMapped.pose.pose.orientation.x;
                transformStamped.transform.rotation.y = odomAftMapped.pose.pose.orientation.y;
                transformStamped.transform.rotation.z = odomAftMapped.pose.pose.orientation.z;
                br->sendTransform(transformStamped);
            }

            // void publish_mavros(const auto &mavros_pose_publisher) {
            //     msg_body_pose.header.stamp = get_ros_time(lidar_end_time);
            //
            //     msg_body_pose.header.frame_id = "camera_odom_frame";
            //     set_posestamp(msg_body_pose.pose);
            //     mavros_pose_publisher.publish(msg_body_pose);
            // }

            void LidarMapping::publish_path(const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath) {
                set_posestamp(msg_body_pose.pose);
                msg_body_pose.header.stamp = get_ros_time(lidar_end_time);
                msg_body_pose.header.frame_id = "camera_init";
                static int jjj = 0;
                jjj++;
                if (jjj % 5 == 0) // if path is too large, the RVIZ will crash
                {
                    path.poses.push_back(msg_body_pose);
                    pubPath->publish(path);
                }
            }

            void LidarMapping::run() {
                RCLCPP_INFO(get_logger(), "Main slam loop running");

                // std::signal(SIGINT, signalHandler);
                rclcpp::Rate rate(5000);
                bool status = rclcpp::ok();

                while (rclcpp::ok()) {
                    // if (stop_flag) break;
                    if (sync_packages(Measures)) {
                        // if (flg_reset) {
                        //     RCLCPP_WARN( get_logger(), "reset when rosbag play back.");
                        //     p_imu->Reset();
                        //     flg_reset = false;
                        //     continue;
                        // }

                        if (feats_undistort->empty() || (feats_undistort == NULL)) {
                            first_lidar_time = Measures.lidar_beg_time;
                            p_imu->first_lidar_time = first_lidar_time;
                            RCLCPP_WARN_ONCE(get_logger(),
                                             "First lidar frame, LIO not ready, no points stored.");
                        }

                        flg_EKF_inited = (Measures.lidar_beg_time - first_lidar_time) < INIT_TIME ? false : true;
                        auto t1 = std::chrono::high_resolution_clock::now();
                        p_imu->Process(Measures, state, feats_undistort);
                        state_propagat = state; {
                            static auto pub_UndistortPcl =
                                    create_publisher<sensor_msgs::msg::PointCloud2>("/livox_undistort", 100);
                            sensor_msgs::msg::PointCloud2 pcl_out_msg;
                            pcl::toROSMsg(*feats_undistort, pcl_out_msg);
                            pcl_out_msg.header.stamp = get_ros_time(Measures.lidar_beg_time);
                            pcl_out_msg.header.frame_id = "map";
                            pub_UndistortPcl->publish(pcl_out_msg);
                        }

                        //            ROS_INFO("IMU bias : acc bias : %.4f %.4f %.4f; gyr bias : %.4f %.4f %.4f  \n",
                        //                     p_imu->cov_bias_acc[0], p_imu->cov_bias_acc[1], p_imu->cov_bias_acc[2],
                        //                     p_imu->cov_bias_gyr[0], p_imu->cov_bias_gyr[1], p_imu->cov_bias_gyr[2]);

                        /*** Segment the map in lidar FOV ***/
                        lasermap_fov_segment();

                        /*** downsample the feature points in a scan ***/
                        downSizeFilterSurf.setInputCloud(feats_undistort);
                        downSizeFilterSurf.filter(*feats_down_body);
                        feats_down_size = feats_down_body->points.size();
                        //             std::cout << "feats size" << feats_undistort->size() << ", down size: " << feats_down_body->size() << std::endl;
                        /*** initialize the map kdtree ***/
                        if (ikdtree.Root_Node == nullptr) {
                            if (feats_down_size > 5) {
                                ikdtree.set_downsample_param(filter_size_map);
                                feats_down_world->resize(feats_down_size);
                                for (int i = 0; i < feats_down_size; i++) {
                                    pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
                                }
                                ikdtree.Build(feats_down_world->points);
                            }
                            continue;
                        }
                        int featsFromMapNum = ikdtree.validnum();
                        kdtree_size_st = ikdtree.size();


                        /*** ICP and iterated Kalman filter update ***/
                        normvec->resize(feats_down_size);
                        feats_down_world->resize(feats_down_size);
                        euler_cur = RotMtoEuler(state.rot_end);


                        pointSearchInd_surf.resize(feats_down_size);
                        Nearest_Points.resize(feats_down_size);
                        int rematch_num = 0;
                        bool nearest_search_en = true;


                        /*** iterated state estimation ***/
                        std::vector<M3D> body_var;
                        std::vector<M3D> crossmat_list;
                        body_var.reserve(feats_down_size);
                        crossmat_list.reserve(feats_down_size);

                        for (iterCount = 0; iterCount < max_iteration && flg_EKF_inited; iterCount++) {
                            laserCloudOri->clear();
                            corr_normvect->clear();
                            total_residual = 0.0;

                            /** closest surface search and residual computation **/
#ifdef MP_EN
                            omp_set_num_threads(MP_PROC_NUM);
#pragma omp parallel for
#endif
                            for (int i = 0; i < feats_down_size; i++) {
                                PointType &point_body = feats_down_body->points[i];
                                PointType &point_world = feats_down_world->points[i];
                                V3D p_body(point_body.x, point_body.y, point_body.z);
                                /// transform to world frame
                                pointBodyToWorld(&point_body, &point_world);
                                vector<float> pointSearchSqDis(NUM_MATCH_POINTS);
                                auto &points_near = Nearest_Points[i];
                                uint8_t search_flag = 0;

                                if (nearest_search_en) {
                                    /** Find the closest surfaces in the map **/
                                    ikdtree.Nearest_Search(point_world, NUM_MATCH_POINTS, points_near, pointSearchSqDis,
                                                           8);
                                    if (points_near.size() < NUM_MATCH_POINTS)
                                        point_selected_surf[i] = false;
                                    else
                                        point_selected_surf[i] = !(pointSearchSqDis[NUM_MATCH_POINTS - 1] > 5);
                                }

                                res_last[i] = -1000.0f;

                                if (!point_selected_surf[i] || points_near.size() < NUM_MATCH_POINTS) {
                                    point_selected_surf[i] = false;
                                    continue;
                                }

                                point_selected_surf[i] = false;
                                VD(4) pabcd;
                                pabcd.setZero();
                                if (esti_plane(pabcd, points_near, 0.1)) //(planeValid)
                                {
                                    float pd2 = pabcd(0) * point_world.x + pabcd(1) * point_world.y + pabcd(2) *
                                                point_world
                                                .z +
                                                pabcd(3);
                                    float s = 1 - 0.9 * fabs(pd2) / sqrt(p_body.norm());

                                    if (s > 0.9) {
                                        point_selected_surf[i] = true;
                                        normvec->points[i].x = pabcd(0);
                                        normvec->points[i].y = pabcd(1);
                                        normvec->points[i].z = pabcd(2);
                                        normvec->points[i].intensity = pd2;
                                        res_last[i] = abs(pd2);
                                    }
                                }
                            }
                            effect_feat_num = 0;
                            for (int i = 0; i < feats_down_size; i++) {
                                if (point_selected_surf[i]) {
                                    laserCloudOri->points[effect_feat_num] = feats_down_body->points[i];
                                    corr_normvect->points[effect_feat_num] = normvec->points[i];
                                    effect_feat_num++;
                                }
                            }

                            res_mean_last = total_residual / effect_feat_num;

                            /*** Computation of Measurement Jacobian matrix H and measurents vector ***/

                            Eigen::MatrixXd Hsub(effect_feat_num, 6);
                            Eigen::MatrixXd Hsub_T_R_inv(6, effect_feat_num);
                            //                VectorXd R_inv(effect_feat_num);
                            Eigen::VectorXd meas_vec(effect_feat_num);

                            Hsub.setZero();
                            Hsub_T_R_inv.setZero();
                            meas_vec.setZero();

                            for (int i = 0; i < effect_feat_num; i++) {
                                const PointType &laser_p = laserCloudOri->points[i];
                                V3D point_this_L(laser_p.x, laser_p.y, laser_p.z);

                                V3D point_this = Lidar_R_wrt_IMU * point_this_L + Lidar_T_wrt_IMU;
                                M3D var;
                                calcBodyVar(point_this, 0.02, 0.05, var);
                                var = state.rot_end * var * state.rot_end.transpose();
                                M3D point_crossmat;
                                point_crossmat << SKEW_SYM_MATRX(point_this);

                                /*** get the normal vector of closest surface/corner ***/
                                const PointType &norm_p = corr_normvect->points[i];
                                V3D norm_vec(norm_p.x, norm_p.y, norm_p.z);

                                //                    R_inv(i) = 1 / LASER_POINT_COV;
                                //                    laserCloudOri->points[i].intensity = sqrt(R_inv(i));

                                /*** calculate the Measuremnt Jacobian matrix H ***/
                                V3D A(point_crossmat * state.rot_end.transpose() * norm_vec);
                                Hsub.row(i) << VEC_FROM_ARRAY(A), norm_p.x, norm_p.y, norm_p.z;
                                //                    Hsub_T_R_inv.col(i) << A[0] * R_inv(i), A[1] * R_inv(i), A[2] * R_inv(i), norm_p.x * R_inv(i),
                                //                            norm_p.y * R_inv(i),
                                //                            norm_p.z * R_inv(i);
                                // cout<<"Hsub.row(i): "<<Hsub.row(i)<<" "<<norm_vec.transpose()<<endl;
                                // cout<<"Hsub_T_R_inv.row(i): "<<Hsub_T_R_inv.col(i).transpose()<<endl;
                                /*** Measuremnt: distance to the closest surface/corner ***/
                                meas_vec(i) = -norm_p.intensity;
                                // if (iterCount == 0) fout_dbg<<setprecision(6)<<i<<" "<<meas_vec[i]<<"
                                // "<<Hsub.row(i)<<endl; if (fabs(meas_vec(i)) > 0.07)
                                // {
                                //     printf("meas_vec(i): %.6lf index: %d \n", meas_vec(i), i);
                                // }
                            }

                            Eigen::MatrixXd K(DIM_STATE, effect_feat_num);

                            EKF_stop_flg = false;
                            flg_EKF_converged = false;

                            if (!flg_EKF_inited) {
                                cout << "||||||||||Initialize LiDAR||||||||||" << endl;
                                /*** only run in initialization period ***/
                                Eigen::MatrixXd H_init(MD(9, DIM_STATE)::Zero());
                                Eigen::MatrixXd z_init(VD(9)::Zero());
                                H_init.block<3, 3>(0, 0) = M3D::Identity();
                                H_init.block<3, 3>(3, 3) = M3D::Identity();
                                H_init.block<3, 3>(6, 15) = M3D::Identity();
                                z_init.block<3, 1>(0, 0) = -Log(state.rot_end);
                                z_init.block<3, 1>(0, 0) = -state.pos_end;

                                auto H_init_T = H_init.transpose();
                                auto &&K_init = state.cov * H_init_T * (H_init * state.cov * H_init_T +
                                                                        0.0001 * MD(9, 9)::Identity()).inverse();
                                solution = K_init * z_init;

                                // solution.block<9,1>(0,0).setZero();
                                // state += solution;
                                // state.cov = (MatrixXd::Identity(DIM_STATE, DIM_STATE) - K_init * H_init) * state.cov;

                                state.resetpose();
                                EKF_stop_flg = true;
                            } else {
                                /*** Iterative Kalman Filter Update ***/
                                Hsub_T_R_inv = Hsub.transpose();
                                auto &&HTz = Hsub_T_R_inv * meas_vec;
                                H_T_H.block<6, 6>(0, 0) = Hsub_T_R_inv * Hsub;
                                //                    MD(DIM_STATE, DIM_STATE) &&K_1 = (H_T_H + state.cov.inverse()).inverse();
                                MD(DIM_STATE, DIM_STATE) &&K_1 = (H_T_H + (state.cov / laser_point_cov).inverse()).
                                        inverse();
                                G.block<DIM_STATE, 6>(0, 0) = K_1.block<DIM_STATE, 6>(0, 0) * H_T_H.block<6, 6>(0, 0);
                                auto vec = state_propagat - state;

                                solution =
                                        K_1.block<DIM_STATE, 6>(0, 0) * HTz + vec -
                                        G.block<DIM_STATE, 6>(0, 0) * vec.block<6, 1>(0, 0);


                                int minRow, minCol;
                                // VD(6) V = H_T_H.block<6, 6>(0, 0).eigenvalues().real();

                                Eigen::MatrixXd block = H_T_H.block<6, 6>(0, 0);
                                Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> solver(block);
                                VD(6) V = solver.eigenvalues().real();

                                if (V.minCoeff(&minRow, &minCol) < 1.0f) {
                                    RCLCPP_WARN(get_logger(),
                                                "IMU states : Gravity: %.4f %.4f %.4f; acc covarience: %.8f %.8f %.8f; gry covarience: %.8f %.8f %.8f \n",
                                                state.gravity[0], state.gravity[1], state.gravity[2],
                                                p_imu->cov_acc[0], p_imu->cov_acc[1], p_imu->cov_acc[2],
                                                p_imu->cov_gyr[0], p_imu->cov_gyr[1], p_imu->cov_gyr[2]);

                                    RCLCPP_WARN(get_logger(), "!!!!!! Degeneration occur, eigen values: ",
                                                V.transpose().format(CleanFmt));
                                }

                                //state update
                                state += solution;

                                rot_add = solution.block<3, 1>(0, 0);
                                T_add = solution.block<3, 1>(3, 0);


                                if ((rot_add.norm() * 57.3 < 0.01) && (T_add.norm() * 100 < 0.015))
                                    flg_EKF_converged = true;

                                deltaR = rot_add.norm() * 57.3;
                                deltaT = T_add.norm() * 100;
                            }

                            euler_cur = RotMtoEuler(state.rot_end);

                            ////debug print
                            // std::cout << "State BA  : " << state.bias_a.format(CleanFmt) << "  BG : "
                            //         << state.bias_g.format(CleanFmt) << "  Grav : " << state.gravity.format(CleanFmt)
                            //         << std::endl;

                            /*** Rematch Judgement ***/
                            nearest_search_en = false;
                            if (flg_EKF_converged || ((rematch_num == 0) && (iterCount == (max_iteration - 2)))) {
                                nearest_search_en = true;
                                rematch_num++;
                            }

                            /*** Convergence Judgements and Covariance Update ***/
                            if (!EKF_stop_flg && (rematch_num >= 2 || (iterCount == max_iteration - 1))) {
                                if (flg_EKF_inited) {
                                    /*** Covariance Update ***/
                                    state.cov = (I_STATE - G) * state.cov;
                                    total_distance += (state.pos_end - position_last).norm();
                                    position_last = state.pos_end;


                                    quat.setRPY(euler_cur(0), euler_cur(1), euler_cur(2));
                                    geoQuat = tf2::toMsg(quat);

                                    VD(DIM_STATE) K_sum = K.rowwise().sum();
                                    VD(DIM_STATE) P_diag = state.cov.diagonal();
                                }
                                EKF_stop_flg = true;
                            }

                            if (EKF_stop_flg) break;
                        }
                        auto t2 = std::chrono::high_resolution_clock::now();
                        double single = std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1).count() / 1e6;

                        RCLCPP_INFO(get_logger(), "process time : %.3f [msec] ", single);

                        /******* Publish odometry *******/
                        euler_cur = RotMtoEuler(state.rot_end);
                        quat.setRPY(euler_cur(0), euler_cur(1), euler_cur(2));
                        geoQuat = tf2::toMsg(quat);
                        publish_odometry(pubOdomAftMapped);

                        /*** add the feature points to map kdtree ***/
                        map_incremental();

                        kdtree_size_end = ikdtree.size();

                        /******* Publish points *******/
                        if (scan_pub_en || pcd_save_en) publish_frame_world(pubLaserCloudFullRes);
                        if (scan_pub_en && scan_body_pub_en) publish_frame_body(pubLaserCloudFullRes_body);
                        last_odom = state.pos_end;
                        last_rot = state.rot_end;
                        publish_effect_world(pubLaserCloudEffect);
                        if (path_en) publish_path(pubPath);
                        //publish_mavros(mavros_pose_publisher);
                    }

                    rate.sleep();
                }
            }
        }
    }
}
