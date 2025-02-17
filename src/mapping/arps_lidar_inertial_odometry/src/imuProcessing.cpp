/******************************************************************************
 * @file        imuProcessing.h
 * @brief       Declaration of the imu process module class and related functions.
 *
 * @author      Yunxiang He
 * @date        2024-11-26
 * @version     2.0.0
 *
 * @details     This file contains the definition and assignment of the pre-process of imu data in the slam module,
 *              and initialize the imu.
 ******************************************************************************/

#include <arps/imuProcessing.h>
#include <arps/lidarMapping.h>

namespace arps {
    namespace slam {
        namespace arps_lidar_inertial_odometry {
            bool ImuProcess::time_list(PointType &x, PointType &y) { return (x.curvature < y.curvature); };

            ImuProcess::ImuProcess() : b_first_frame_(true), imu_need_init_(true) {
                init_iter_num = 1;
                cov_acc = V3D(0.1, 0.1, 0.1);
                cov_gyr = V3D(0.1, 0.1, 0.1);
                cov_acc_scale = V3D(1, 1, 1);
                cov_gyr_scale = V3D(1, 1, 1);
                cov_bias_gyr = V3D(0.0001, 0.0001, 0.0001);
                cov_bias_acc = V3D(0.0001, 0.0001, 0.0001);
                mean_acc = V3D(0, 0, -1.0);
                mean_gyr = V3D(0, 0, 0);
                angvel_last = Zero3d;
                acc_s_last = Zero3d;
                Lid_offset_to_IMU = Zero3d;
                Lid_rot_to_IMU = Eye3d;
                last_imu_.reset(new sensor_msgs::msg::Imu());
            }

            ImuProcess::~ImuProcess() {
            }

            void ImuProcess::Reset() {
                LOG(INFO) <<  "Reset ImuProcess\n";
                mean_acc = V3D(0, 0, -1.0);
                mean_gyr = V3D(0, 0, 0);
                angvel_last = Zero3d;
                imu_need_init_ = true;
                start_timestamp_ = -1;
                init_iter_num = 1;
                v_imu_.clear();
                IMUpose.clear();
                last_imu_.reset(new sensor_msgs::msg::Imu());
                cur_pcl_un_.reset(new PointCloudXYZI());
            }

            void ImuProcess::disable_imu() {
                LOG(INFO) << "IMU disabled !!!!!\n";
                imu_en = false;
                imu_need_init_ = false;
            }

            void ImuProcess::set_extrinsic(const Eigen::Matrix<double, 4, 4> &T) {
                Lid_offset_to_IMU = T.block<3, 1>(0, 3);
                Lid_rot_to_IMU = T.block<3, 3>(0, 0);
            }

            void ImuProcess::set_extrinsic(const V3D &transl) {
                Lid_offset_to_IMU = transl;
                Lid_rot_to_IMU.setIdentity();
            }

            void ImuProcess::set_extrinsic(const V3D &transl, const M3D &rot) {
                Lid_offset_to_IMU = transl;
                Lid_rot_to_IMU = rot;
            }

            void ImuProcess::set_mean_acc_norm(const double &mean_acc_norm) {
                IMU_mean_acc_norm = mean_acc_norm;
            }

            void ImuProcess::set_gyr_cov_scale(const V3D &scaler) { cov_gyr_scale = scaler; }

            void ImuProcess::set_acc_cov_scale(const V3D &scaler) { cov_acc_scale = scaler; }

            void ImuProcess::set_gyr_bias_cov(const V3D &b_g) { cov_bias_gyr = b_g; }

            void ImuProcess::set_acc_bias_cov(const V3D &b_a) { cov_bias_acc = b_a; }

            void ImuProcess::set_max_init_count(const int &max_init_count) {
                MAX_INIT_COUNT = max_init_count;
            }


            void ImuProcess::IMU_init(const MeasureGroup &meas, StatesGroup &state_inout, int &N) {
                /** 1. initializing the gravity, gyro bias, acc and gyro covariance
                 ** 2. normalize the acceleration measurenments to unit gravity **/
                //    ROS_INFO("IMU Initializing: %.1f %%", double( N ) / MAX_INI_COUNT * 100);
                V3D cur_acc, cur_gyr;

                if (b_first_frame_) {
                    Reset();
                    N = 1;
                    b_first_frame_ = false;
                    const auto &imu_acc = meas.imu.front()->linear_acceleration;
                    const auto &gyr_acc = meas.imu.front()->angular_velocity;
                    mean_acc << imu_acc.x, imu_acc.y, imu_acc.z;
                    mean_gyr << gyr_acc.x, gyr_acc.y, gyr_acc.z;
                    first_lidar_time = meas.lidar_beg_time;
                    // std::cout<<"init acc norm: "<<mean_acc.norm()<<std::endl;
                }

                for (const auto &imu: meas.imu) {
                    const auto &imu_acc = imu->linear_acceleration;
                    const auto &gyr_acc = imu->angular_velocity;
                    cur_acc << imu_acc.x, imu_acc.y, imu_acc.z;
                    cur_gyr << gyr_acc.x, gyr_acc.y, gyr_acc.z;

                    mean_acc += (cur_acc - mean_acc) / N;
                    mean_gyr += (cur_gyr - mean_gyr) / N;

                    cov_acc = cov_acc * (N - 1.0) / N + (cur_acc - mean_acc).cwiseProduct(cur_acc - mean_acc) * (
                                  N - 1.0) / (N * N);
                    cov_gyr = cov_gyr * (N - 1.0) / N + (cur_gyr - mean_gyr).cwiseProduct(cur_gyr - mean_gyr) * (
                                  N - 1.0) / (N * N);

                    // std::cout<<"acc norm: "<<cur_acc.norm()<<" "<<mean_acc.norm()<<std::endl;

                    N++;
                }

                state_inout.gravity = -mean_acc / mean_acc.norm() * G_m_s2;

                state_inout.rot_end = Eye3d; // Exp(mean_acc.cross(V3D(0, 0, -1 / scale_gravity)));
                //    state_inout.bias_g = Zero3d; // mean_gyr;
                state_inout.bias_g = mean_gyr; // mean_gyr;

                last_imu_ = meas.imu.back();
            }


            void ImuProcess::Forward_propagation_without_imu(const MeasureGroup &meas, StatesGroup &state_inout,
                                                             PointCloudXYZI &pcl_out) {
                pcl_out = *(meas.lidar);
                /*** sort point clouds by offset time ***/
                const double &pcl_beg_time = meas.lidar_beg_time;
                sort(pcl_out.points.begin(), pcl_out.points.end(), time_list);
                const double &pcl_end_offset_time = pcl_out.points.back().curvature / double(1000);

                Eigen::Matrix<double,DIM_STATE,DIM_STATE> F_x, cov_w;
                double dt = 0.0;

                if (b_first_frame_) {
                    dt = 0.1;
                    b_first_frame_ = false;
                } else {
                    dt = pcl_beg_time - time_last_scan;
                    time_last_scan = pcl_beg_time;
                }

                /* covariance propagation */
                F_x.setIdentity();
                cov_w.setZero();
                /** In CV model, bias_g represents angular velocity **/
                /** In CV model，bias_a represents linear acceleration **/
                M3D Exp_f = Exp(state_inout.bias_g, dt);
                F_x.block<3, 3>(0, 0) = Exp(state_inout.bias_g, -dt);
                F_x.block<3, 3>(0, 15) = Eye3d * dt;
                F_x.block<3, 3>(3, 12) = Eye3d * dt;


                cov_w.block<3, 3>(15, 15).diagonal() = cov_gyr_scale * dt * dt;
                cov_w.block<3, 3>(12, 12).diagonal() = cov_acc_scale * dt * dt;

                /** Forward propagation of covariance**/
                state_inout.cov = F_x * state_inout.cov * F_x.transpose() + cov_w;

                /** Forward propagation of attitude **/
                state_inout.rot_end = state_inout.rot_end * Exp_f;

                /** Position Propagation **/
                state_inout.pos_end += state_inout.vel_end * dt;


                auto it_pcl = pcl_out.points.end() - 1;
                double dt_j = 0.0;
                for (; it_pcl != pcl_out.points.begin(); it_pcl--) {
                    dt_j = pcl_end_offset_time - it_pcl->curvature / double(1000);
                    M3D R_jk(Exp(state_inout.bias_g, -dt_j));
                    V3D P_j(it_pcl->x, it_pcl->y, it_pcl->z);
                    // Using rotation and translation to un-distort points
                    V3D p_jk;
                    p_jk = -state_inout.rot_end.transpose() * state_inout.vel_end * dt_j;

                    V3D P_compensate = R_jk * P_j + p_jk;

                    /// save Undistorted points and their rotation
                    it_pcl->x = P_compensate(0);
                    it_pcl->y = P_compensate(1);
                    it_pcl->z = P_compensate(2);
                }
            }

            //void ImuProcess::UndistortPcl(const MeasureGroup &meas, StatesGroup &state_inout, PointCloudXYZI &pcl_out) {
            //    /*** add the imu of the last frame-tail to the current frame-head ***/
            //    pcl_out = *(meas.lidar);
            //    auto v_imu = meas.imu;
            //    v_imu.push_front(last_imu_);
            //    double imu_end_time = v_imu.back()->header.stamp.toSec();
            //    double pcl_beg_time, pcl_end_time;
            //
            //
            //    pcl_beg_time = meas.lidar_beg_time;
            //    /*** sort point clouds by offset time ***/
            //    sort(pcl_out.points.begin(), pcl_out.points.end(), time_list);
            //    pcl_end_time = pcl_beg_time + pcl_out.points.back().curvature / double(1000);
            //
            //
            //    /*** Initialize IMU pose ***/
            //    IMUpose.clear();
            //    IMUpose.push_back(
            //            set_pose6d(0.0, acc_s_last, angvel_last, state_inout.vel_end, state_inout.pos_end, state_inout.rot_end));
            //
            //    /*** forward propagation at each imu point ***/
            //    V3D acc_imu, angvel_avr, acc_avr, vel_imu(state_inout.vel_end), pos_imu(state_inout.pos_end);
            //    M3D R_imu(state_inout.rot_end);
            //    MD(DIM_STATE, DIM_STATE) F_x, cov_w;
            //
            //    double dt = 0;
            //    for (auto it_imu = v_imu.begin(); it_imu < (v_imu.end() - 1); it_imu++) {
            //        auto &&head = *(it_imu);
            //        auto &&tail = *(it_imu + 1);
            //
            //        if (tail->header.stamp.toSec() < pcl_beg_time) continue;
            //
            //        angvel_avr << 0.5 * (head->angular_velocity.x + tail->angular_velocity.x),
            //                0.5 * (head->angular_velocity.y + tail->angular_velocity.y),
            //                0.5 * (head->angular_velocity.z + tail->angular_velocity.z);
            //
            //
            //        acc_avr << 0.5 * (head->linear_acceleration.x + tail->linear_acceleration.x),
            //                0.5 * (head->linear_acceleration.y + tail->linear_acceleration.y),
            //                0.5 * (head->linear_acceleration.z + tail->linear_acceleration.z);
            //
            //        V3D angvel_now(head->angular_velocity.x, head->angular_velocity.y, head->angular_velocity.z);
            //        V3D acc_now(head->linear_acceleration.x, head->linear_acceleration.y, head->linear_acceleration.z);
            //        fout_imu << setw(10) << head->header.stamp.toSec() << "  " << angvel_now.transpose() << " "
            //                 << acc_now.transpose() << std::endl;
            //
            //        angvel_avr -= state_inout.bias_g;
            //        acc_avr = acc_avr / mean_acc.norm() * G_m_s2 - state_inout.bias_a;
            //
            //        if (head->header.stamp.toSec() < pcl_beg_time)
            //            dt = tail->header.stamp.toSec() - pcl_beg_time;
            //        else
            //            dt = tail->header.stamp.toSec() - head->header.stamp.toSec();
            //
            //        /* covariance propagation */
            //        M3D acc_avr_skew;
            //        M3D Exp_f = Exp(angvel_avr, dt);
            //        acc_avr_skew << SKEW_SYM_MATRX(acc_avr);
            //
            //        F_x.setIdentity();
            //        cov_w.setZero();
            //
            //        F_x.block<3, 3>(0, 0) = Exp(angvel_avr, -dt);
            //        F_x.block<3, 3>(0, 9) = -Eye3d * dt;
            //        // F_x.block<3,3>(3,0)  = R_imu * off_vel_skew * dt;
            //        F_x.block<3, 3>(3, 6) = Eye3d * dt;
            //        F_x.block<3, 3>(6, 0) = -R_imu * acc_avr_skew * dt;
            //        F_x.block<3, 3>(6, 12) = -R_imu * dt;
            //        F_x.block<3, 3>(6, 15) = Eye3d * dt;
            //
            //        cov_w.block<3, 3>(0, 0).diagonal() = cov_gyr * dt * dt;
            //        cov_w.block<3, 3>(6, 6) = R_imu * cov_acc.asDiagonal() * R_imu.transpose() * dt * dt;
            //        cov_w.block<3, 3>(9, 9).diagonal() = cov_bias_gyr * dt * dt; // bias gyro covariance
            //        cov_w.block<3, 3>(12, 12).diagonal() = cov_bias_acc * dt * dt; // bias acc covariance
            //
            //        state_inout.cov = F_x * state_inout.cov * F_x.transpose() + cov_w;
            //
            //        /* propagation of IMU attitude (global frame)*/
            //        R_imu = R_imu * Exp_f;
            //
            //        /* Specific acceleration (global frame) of IMU */
            //        acc_imu = R_imu * acc_avr + state_inout.gravity;
            //
            //        /* propagation of IMU position (global frame)*/
            //        pos_imu = pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt;
            //
            //        /* velocity of IMU (global frame)*/
            //        vel_imu = vel_imu + acc_imu * dt;
            //
            //        /* save the poses at each IMU measurements (global frame)*/
            //        angvel_last = angvel_avr;
            //        acc_s_last = acc_imu;
            //        double &&offs_t = tail->header.stamp.toSec() - pcl_beg_time;
            //        IMUpose.push_back(set_pose6d(offs_t, acc_imu, angvel_avr, vel_imu, pos_imu, R_imu));
            //    }
            //
            //    /*** calculated the pos and attitude prediction at the frame-end ***/
            //    double note = pcl_end_time > imu_end_time ? 1.0 : -1.0;
            //    dt = note * (pcl_end_time - imu_end_time);
            //    state_inout.vel_end = vel_imu + note * acc_imu * dt;
            //    state_inout.rot_end = R_imu * Exp(V3D(note * angvel_avr), dt);
            //    state_inout.pos_end = pos_imu + note * vel_imu * dt + note * 0.5 * acc_imu * dt * dt;
            //
            //
            //    auto pos_liD_e = state_inout.pos_end + state_inout.rot_end * Lid_offset_to_IMU;
            //
            //    /*** un-distort each lidar point (backward propagation) ***/
            //    auto it_pcl = pcl_out.points.end() - 1; //a single point in k-th frame
            //    for (auto it_kp = IMUpose.end() - 1; it_kp != IMUpose.begin(); it_kp--) {
            //        auto head = it_kp - 1;
            //        R_imu << MAT_FROM_ARRAY(head->rot);
            //        acc_imu << VEC_FROM_ARRAY(head->acc);
            //        // std::cout<<"head imu acc: "<<acc_imu.transpose()<<std::endl;
            //        vel_imu << VEC_FROM_ARRAY(head->vel);
            //        pos_imu << VEC_FROM_ARRAY(head->pos);
            //        angvel_avr << VEC_FROM_ARRAY(head->gyr);
            //        for (; it_pcl->curvature / double(1000) > head->offset_time; it_pcl--) {
            //            dt = it_pcl->curvature / double(1000) - head->offset_time; //dt = t_j - t_i > 0
            //            /* Transform to the 'scan-end' IMU frame（I_k frame)*/
            ////            M3D R_i(R_imu * Exp(angvel_avr, dt));
            ////            V3D P_i = pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt;
            ////            V3D p_in(it_pcl->x, it_pcl->y, it_pcl->z);
            ////            V3D P_compensate = Lid_rot_to_IMU.transpose() * state_inout.rot_end.transpose() *
            ////                               (R_i * (Lid_rot_to_IMU * p_in + Lid_offset_to_IMU) +
            ////                                P_i - state_inout.pos_end) -
            ////                               Lid_rot_to_IMU.transpose() * Lid_offset_to_IMU;
            //
            //            M3D R_i(R_imu * Exp(angvel_avr, dt));
            //            V3D T_ei(pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt +
            //                     R_i * Lid_offset_to_IMU - pos_liD_e);
            //
            //            V3D P_i(it_pcl->x, it_pcl->y, it_pcl->z);
            //            V3D P_compensate = state_inout.rot_end.transpose() * (R_i * P_i + T_ei);
            //            /// save Undistorted points
            //            it_pcl->x = P_compensate(0);
            //            it_pcl->y = P_compensate(1);
            //            it_pcl->z = P_compensate(2);
            //            if (it_pcl == pcl_out.points.begin()) break;
            //        }
            //
            //    }
            //}

            void ImuProcess::UndistortPcl(const MeasureGroup &meas,
                                          StatesGroup &state_inout,
                                          PointCloudXYZI &surf_pcl_out) {
                /*** add the imu of the last frame-tail to the of current frame-head ***/
                auto v_imu = meas.imu;
                v_imu.push_front(last_imu_);
                const double &imu_beg_time = get_time_in_sec(v_imu.front()->header.stamp);
                const double &imu_end_time = get_time_in_sec(v_imu.back()->header.stamp);
                const double &pcl_beg_time = meas.lidar_beg_time;
                /*** sort point clouds by offset time ***/
                surf_pcl_out = *(meas.lidar);
                sort(surf_pcl_out.points.begin(), surf_pcl_out.points.end(), time_list);
                const double &pcl_end_time =
                        pcl_beg_time + surf_pcl_out.points.back().curvature / double(1000);
                /*** Initialize IMU pose ***/
                IMUpose.clear();
                IMUpose.push_back(set_pose6d(0.0, acc_s_last, angvel_last,
                                             state_inout.vel_end, state_inout.pos_end,
                                             state_inout.rot_end));
                /*** forward propagation at each imu point ***/
                V3D acc_imu, angvel_avr, acc_avr, vel_imu(state_inout.vel_end),
                        pos_imu(state_inout.pos_end);
                M3D R_imu(state_inout.rot_end);
                Eigen::Matrix<double,DIM_STATE,DIM_STATE> F_x, cov_w;
                double dt = 0;
                for (auto it_imu = v_imu.begin(); it_imu < (v_imu.end() - 1); it_imu++) {
                    auto &&head = *(it_imu);
                    auto &&tail = *(it_imu + 1);

                    if (get_time_in_sec(tail->header.stamp) < pcl_beg_time) {
                        continue;
                    }

                    angvel_avr << 0.5 * (head->angular_velocity.x + tail->angular_velocity.x),
                            0.5 * (head->angular_velocity.y + tail->angular_velocity.y),
                            0.5 * (head->angular_velocity.z + tail->angular_velocity.z);
                    acc_avr << 0.5 *
                            (head->linear_acceleration.x + tail->linear_acceleration.x),
                            0.5 * (head->linear_acceleration.y + tail->linear_acceleration.y),
                            0.5 * (head->linear_acceleration.z + tail->linear_acceleration.z);

                    angvel_avr -= state_inout.bias_g;
                    acc_avr = acc_avr * G_m_s2 / mean_acc.norm() - state_inout.bias_a;

                    if (get_time_in_sec(head->header.stamp) < pcl_beg_time) {
                        dt = get_time_in_sec(tail->header.stamp) - pcl_beg_time;
                    } else {
                        dt = get_time_in_sec(tail->header.stamp) - get_time_in_sec(head->header.stamp);
                    }

                    /*** covariance propagation of error state ***/
                    M3D acc_avr_skew;
                    M3D Exp_f = Exp(angvel_avr, dt);
                    acc_avr_skew << SKEW_SYM_MATRX(acc_avr);
                    // F_x
                    F_x.setIdentity();
                    F_x.block<3, 3>(0, 0) = Exp(angvel_avr, -dt);
                    F_x.block<3, 3>(0, 9) = -Eye3d * dt;
                    F_x.block<3, 3>(3, 6) = Eye3d * dt;
                    F_x.block<3, 3>(6, 0) = -R_imu * acc_avr_skew * dt;
                    F_x.block<3, 3>(6, 12) = -R_imu * dt;
                    F_x.block<3, 3>(6, 15) = Eye3d * dt;
                    // Q
                    cov_w.setZero();
                    cov_w.block<3, 3>(0, 0).diagonal() = cov_gyr * dt * dt;
                    cov_w.block<3, 3>(6, 6) =
                            R_imu * cov_acc.asDiagonal() * R_imu.transpose() * dt * dt;
                    cov_w.block<3, 3>(9, 9).diagonal() =
                            cov_bias_gyr * dt * dt; // bias gyro covariance
                    cov_w.block<3, 3>(12, 12).diagonal() =
                            cov_bias_acc * dt * dt; // bias acc covariance
                    // P = FPF^T + Q
                    state_inout.cov = F_x * state_inout.cov * F_x.transpose() + cov_w;

                    /*** propogation of nonmial state ***/
                    R_imu = R_imu * Exp_f;
                    acc_imu = R_imu * acc_avr + state_inout.gravity;
                    pos_imu = pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt;
                    vel_imu = vel_imu + acc_imu * dt;
                    /* save the poses at each IMU measurements */
                    angvel_last = angvel_avr;
                    acc_s_last = acc_imu;
                    double &&offs_t = get_time_in_sec(tail->header.stamp) - pcl_beg_time;
                    IMUpose.push_back(
                        set_pose6d(offs_t, acc_imu, angvel_avr, vel_imu, pos_imu, R_imu));
                }
                /*** calculated the pos and attitude prediction at the frame-end ***/
                double note = pcl_end_time > imu_end_time ? 1.0 : -1.0;
                dt = note * (pcl_end_time - imu_end_time);
                state_inout.vel_end = vel_imu + note * acc_imu * dt;
                state_inout.rot_end = R_imu * Exp(V3D(note * angvel_avr), dt);
                state_inout.pos_end =
                        pos_imu + note * vel_imu * dt + note * 0.5 * acc_imu * dt * dt;

                auto pos_liD_e =
                        state_inout.pos_end + state_inout.rot_end * Lid_offset_to_IMU;
                /*** undistort each surf lidar point (backward propagation) ***/
                auto it_surf_pcl = surf_pcl_out.points.end() - 1;
                for (auto it_kp = IMUpose.end() - 1; it_kp != IMUpose.begin(); it_kp--) {
                    auto head = it_kp - 1;
                    auto tail = it_kp;
                    R_imu << MAT_FROM_ARRAY(head->rot);
                    acc_imu << VEC_FROM_ARRAY(head->acc);
                    // std::cout<<"head imu acc: "<<acc_imu.transpose()<<std::endl;
                    vel_imu << VEC_FROM_ARRAY(head->vel);
                    pos_imu << VEC_FROM_ARRAY(head->pos);
                    angvel_avr << VEC_FROM_ARRAY(head->gyr);

                    for (; it_surf_pcl->curvature / double(1000) > head->offset_time; it_surf_pcl--) {
                        dt = it_surf_pcl->curvature / double(1000) - head->offset_time;
                        /* Transform to the 'end' frame, using only the rotation
                         * Note: Compensation direction is INVERSE of Frame's moving direction
                         * So if we want to compensate a point at timestamp-i to the frame-e
                         * P_compensate = R_imu_e ^ T * (R_i * P_i + T_ei) where T_ei is
                         * represented in global frame */
                        M3D R_i(R_imu * Exp(angvel_avr, dt));
                        V3D T_ei(pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt +
                                 R_i * Lid_offset_to_IMU - pos_liD_e);

                        V3D P_i(it_surf_pcl->x, it_surf_pcl->y, it_surf_pcl->z);
                        V3D P_compensate = state_inout.rot_end.transpose() * (R_i * P_i + T_ei);

                        /// save Undistorted points and their rotation
                        it_surf_pcl->x = P_compensate(0);
                        it_surf_pcl->y = P_compensate(1);
                        it_surf_pcl->z = P_compensate(2);

                        if (it_surf_pcl == surf_pcl_out.points.begin())
                            break;
                    }
                }
            }


            void ImuProcess::Process(const MeasureGroup &meas, StatesGroup &stat, PointCloudXYZI::Ptr cur_pcl_un_) {
                if (imu_en) {
                    if (meas.imu.empty()) return;
                    assert(meas.lidar != nullptr);

                    if (imu_need_init_) {
                        /// The very first lidar frame
                        IMU_init(meas, stat, init_iter_num);
                        imu_need_init_ = true;
                        last_imu_ = meas.imu.back();
                        if (init_iter_num > MAX_INIT_COUNT) {
                            cov_acc *= pow(G_m_s2 / mean_acc.norm(), 2);
                            imu_need_init_ = false;

                            cov_acc = cov_acc.cwiseProduct(cov_acc_scale);
                            cov_gyr = cov_gyr.cwiseProduct(cov_gyr_scale);
                            IMU_mean_acc_norm = mean_acc.norm();
                        }
                        return;
                    }
                    UndistortPcl(meas, stat, *cur_pcl_un_);
                    last_imu_ = meas.imu.back();
                } else {
                    Forward_propagation_without_imu(meas, stat, *cur_pcl_un_);
                }
            }
        }
    }
}
