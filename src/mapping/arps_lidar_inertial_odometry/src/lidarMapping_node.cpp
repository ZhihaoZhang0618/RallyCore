//
// Created by hyx on 24-11-27.
//

#include <arps/lidarMapping.h>
#include <glog/logging.h>

int main(int argc, char **argv) {
    google::InitGoogleLogging(argv[0]);
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    options.arguments({"lidarMapping"});
    auto lio = std::make_shared<arps::slam::arps_lidar_inertial_odometry::LidarMapping>(options);
    // lio->run();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(lio);

    std::thread lioMappingthread(&arps::slam::arps_lidar_inertial_odometry::LidarMapping::run, lio);
    executor.spin();
    rclcpp::shutdown();

    lioMappingthread.join();

    return 0;
}
