# Veddar VESC Interface

![ROS2 CI Workflow](https://github.com/f1tenth/vesc/workflows/ROS2%20CI%20Workflow/badge.svg)

Packages to interface with Veddar VESC motor controllers. See https://vesc-project.com/ for details

This is a ROS2 implementation of the ROS1 driver using the new serial driver located in [transport drivers](https://github.com/ros-drivers/transport_drivers).

## How to test

1. Clone this repository and [transport drivers](https://github.com/ros-drivers/transport_drivers) into `src`.
2. `rosdep update && rosdep install --from-paths src -i -y`
3. Plug in the VESC with a USB cable.
4. Modify `vesc/vesc_driver/params/vesc_config.yaml` to reflect any changes.
5. Build the packages `colcon build`
6. `ros2 launch vesc_driver vesc_driver_node.launch.py`
7. If prompted "permission denied" on the serial port: `sudo chmod 777 /dev/ttyACM0`

## Modification
1. Modify the odometry computation, as the original method causes a speed delay when decelerating from high velocities to a stop.
2. update duty, current control mode
```
Speed+steer
float32 steering_angle
float32 steering_angle_velocity 0
float32 speed
float32 acceleration 0
float32 jerk 0

speed+acc(forward)+steer （nav only）
float32 steering_angle
float32 steering_angle_velocity 0
float32 speed
float32 acceleration 
float32 jerk 1

current+steer
float32 steering_angle
float32 steering_angle_velocity 0
float32 speed 0
float32 acceleration (current)
float32 jerk 2

duty+steer
float32 steering_angle
float32 steering_angle_velocity 0
float32 speed 0
float32 acceleration (duty)
float32 jerk 3
```
