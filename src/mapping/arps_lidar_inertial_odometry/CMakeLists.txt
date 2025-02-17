cmake_minimum_required(VERSION 3.8)
project(arps_lidar_inertial_odometry)

SET(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++17")
set(CMAKE_CXX_FLAGS_RELEASE "-O4 -Wall -g")

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fexceptions")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -pthread -std=c++0x -std=c++17 -fexceptions")


if (POLICY CMP0048)
    cmake_policy(SET CMP0048 NEW)
endif (POLICY CMP0048)

message("Current CPU architecture: ${CMAKE_SYSTEM_PROCESSOR}")
if (CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)")
    include(ProcessorCount)
    ProcessorCount(N)
    message("Processer number:  ${N}")
    if (N GREATER 6)
        add_definitions(-DMP_EN)
        add_definitions(-DMP_PROC_NUM=6)
        message("core for MP: 3")
    elseif (N GREATER 3)
        add_definitions(-DMP_EN)
        add_definitions(-DMP_PROC_NUM=2)
        message("core for MP: 2")
    else ()
        add_definitions(-DMP_PROC_NUM=1)
    endif ()
else ()
    add_definitions(-DMP_PROC_NUM=1)
endif ()

find_package(OpenMP QUIET)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}   ${OpenMP_C_FLAGS}")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_ros REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(glog REQUIRED)

include_directories(
        ${EIGEN3_INCLUDE_DIR}
        ${PCL_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        include)

ament_auto_add_library(ikdtree SHARED
        include/ikd-Tree/ikd_Tree.cpp
)
target_link_libraries(ikdtree
        ${PCL_LIBRARIES}
)
install(TARGETS ikdtree
        EXPORT export_${PROJECT_NAME}
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
)

set(dependencies
        rclcpp
        std_msgs
        geometry_msgs
        nav_msgs
        sensor_msgs
        image_transport
        cv_bridge
        tf2
        tf2_ros
        tf2_eigen
        tf2_geometry_msgs
        visualization_msgs
        pcl_conversions
        livox_ros_driver2
)

ament_auto_add_library(arpsSlamLib SHARED
        src/preProcess.cpp
        src/imuProcessing.cpp
        src/lidarMapping.cpp
)
ament_target_dependencies(arpsSlamLib
        ${dependencies}
)
target_link_libraries(arpsSlamLib
        ${PCL_LIBRARIES}
        ${OpenCV_LIBRARIES}
        ikdtree
        yaml-cpp
        glog::glog
)
install(TARGETS arpsSlamLib
        EXPORT export_${PROJECT_NAME}
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
)

add_executable(lio_node src/lidarMapping_node.cpp)
ament_target_dependencies(lio_node
        ${dependencies}
)
target_link_libraries(lio_node
        arpsSlamLib
        ${PCL_LIBRARIES}
        ${OpenCV_LIBRARIES}
        ikdtree
        glog::glog
)

add_executable(publish_keyframe src/publish_keyframe.cpp)
ament_target_dependencies(publish_keyframe
        ${dependencies}
)
target_link_libraries(publish_keyframe
        ${PCL_LIBRARIES}
        ${OpenCV_LIBRARIES}
)

install(TARGETS lio_node publish_keyframe
        DESTINATION lib/${PROJECT_NAME}
)

install(
        DIRECTORY config launch rviz_cfg
        DESTINATION share/${PROJECT_NAME}
)

ament_package()
