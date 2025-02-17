/******************************************************************************
 * @file        imuProcessing.h
 * @brief       Declaration of the imu process module class and related functions.
 *
 * @author      Yunxiang He
 * @date        2024-11-26
 * @version     2.0.0
 * @copyright   WESTLAKE ROBOTICS TECHNOLOGY (HANGZHOU) CO.,LTD
 *
 * @details     This file contains the definition and assignment of the pre-process of imu data in the slam module,
 *              and initialize the imu.
 *
 * @license     MIT License
 ******************************************************************************/
#pragma once
#ifndef IMUPROCESSING_H
#define IMUPROCESSING_H

#include <arps/utility.h>
#include <rclcpp/rclcpp.hpp>

namespace arps {
    namespace slam {
        namespace arps_lidar_inertial_odometry {

            /// *************IMU Process and undistortion
            class ImuProcess {
            public:
                EIGEN_MAKE_ALIGNED_OPERATOR_NEW

                ImuProcess();

                ~ImuProcess();

                void Reset();

                void Reset(double start_timestamp, const sensor_msgs::msg::Imu::ConstSharedPtr &lastimu);

                void set_extrinsic(const V3D &transl, const M3D &rot);

                void set_extrinsic(const V3D &transl);

                void set_extrinsic(const Eigen::Matrix<double, 4, 4> &T);

                void set_mean_acc_norm(const double &mean_acc_norm);

                void set_gyr_cov_scale(const V3D &scaler);

                void set_acc_cov_scale(const V3D &scaler);

                void set_gyr_bias_cov(const V3D &b_g);

                void set_acc_bias_cov(const V3D &b_a);

                void set_max_init_count(const int &max_init_count);

                void disable_imu();

                static bool time_list(PointType &x, PointType &y);

                void Process(const MeasureGroup &meas, StatesGroup &stat, PointCloudXYZI::Ptr cur_pcl_un_);

                //    ros::NodeHandle nh;
                std::ofstream fout_imu;
                V3D cov_acc;
                V3D cov_gyr;
                V3D cov_acc_scale;
                V3D cov_gyr_scale;
                V3D cov_bias_gyr;
                V3D cov_bias_acc;
                double first_lidar_time;

            private:
                void IMU_init(const MeasureGroup &meas, StatesGroup &state, int &N);

                void UndistortPcl(const MeasureGroup &meas, StatesGroup &state_inout, PointCloudXYZI &pcl_in_out);

                void Forward_propagation_without_imu(const MeasureGroup &meas, StatesGroup &state_inout,
                                                     PointCloudXYZI &pcl_out);


                PointCloudXYZI::Ptr cur_pcl_un_;
                sensor_msgs::msg::Imu::ConstSharedPtr last_imu_;
                // StatesGroup last_state;
                std::deque<sensor_msgs::msg::Imu::ConstSharedPtr> v_imu_;
                std::vector<Pose6D> IMUpose;
                std::vector<M3D> v_rot_pcl_;
                M3D Lid_rot_to_IMU;
                V3D Lid_offset_to_IMU;
                V3D mean_acc;
                V3D mean_gyr;
                V3D angvel_last;
                V3D acc_s_last;
                double start_timestamp_;
                double time_last_scan;
                int init_iter_num = 1;
                bool b_first_frame_ = true;
                bool imu_need_init_ = true;
                bool imu_en = true;
                double IMU_mean_acc_norm;
                int MAX_INIT_COUNT;
            };
        }
    }
}

#endif //IMUPROCESSING_H
