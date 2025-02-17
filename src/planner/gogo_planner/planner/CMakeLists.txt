cmake_minimum_required(VERSION 3.4...3.18)
project(planner)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

# 添加 pybind11 子目录
add_subdirectory(pybind11)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

# 如果 setup.py 没有传递 EXAMPLE_VERSION_INFO，则默认使用 "dev"
if(NOT DEFINED EXAMPLE_VERSION_INFO)
    set(EXAMPLE_VERSION_INFO "dev")
endif()

pybind11_add_module(planner 
  src/main.cpp
  src/grid_map.cpp
  src/penal_traj_opt.cpp)

target_compile_definitions(planner PRIVATE VERSION_INFO=${EXAMPLE_VERSION_INFO})

target_link_libraries(
  planner
  PRIVATE pybind11::module
  ${OpenCV_LIBS}
  ${pybind11_LIBRARIES}
)
