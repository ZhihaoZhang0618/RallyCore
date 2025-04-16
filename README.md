# RallyCore

## Overview
RallyCore is a ROS2- and Nav2-based software stack designed for rally car, which is a rally car for autonomous vehicle development at all kinds of complex terrains. 

All the code was written during my free time after work while I was a Research Assistant at [ZJU FAST Lab](https://github.com/ZJU-FAST-Lab). I am deeply grateful to the wonderful people at ZJU FAST Lab for their support and guidance. Even though this is just a small project, their assistance was invaluable.
## Installation

I provide an installation guide [here](install.md), for ROS2 Humble on Ubuntu 22.04.

May have some issues with the installation guide, please let me know if you have any questions.
## Modification & Customization
#### Some Hardware Details
NVIDIA Orin or NUC(recommended)

Lidar: mid360(main)

Camera: didn't use right now

IMU: fdilink Deta10(only for yaw estimation)

Remote Controller: RadioMaster Pocket ELRS version (much better than XBOX series controller) 

For more details about the hardware, please contact to the introduction in FastLab NAS.
#### VESC Interface
The modified VESC interface is based on the VESC interface provided by Veddar VESC Interface. The modification includes:
- Modify the odometry computation, as the original method causes a speed delay when decelerating from high velocities to a stop.

#### ackermann_mux
- Add some scripts to process msg from ELRS driver and publish to /teleop

#### Nav2's param
- refer to the [QUTMS_Driverless](https://github.com/QUT-Motorsport/QUTMS_Driverless) to set up Nav2

## Main Launch Files

```
ros2 launch fastlio2 lio_launch.py
ros2 launch livox_ros_driver2 msg_MID360_launch.py



ros2 launch f1tenth_system base.launch.py
#for hardware driver bringup

ros2 launch f1tenth_system slam.launch.py
#for slam 

ros2 launch f1tenth_system localization_slam.launch.py
#for localization

ros2 launch f1tenth_system nav.launch.py
# nav2 wo avoidance launch file

or

ros2 launch f1tenth_system nav_avoid.launch.py
# nav2 with avoidance launch file
```


## Acknowledgement
This project would not be possible without the use of multiple great open-sourced code bases as listed below:
- [ForzaETH Race Stack](https://github.com/ForzaETH/race_stack)
- [QUTMS_Driverless](https://github.com/QUT-Motorsport/QUTMS_Driverless)
- [f1tenth_system](https://github.com/f1tenth/f1tenth_system)
- [ros2_crsf_receiver](https://github.com/AndreyTulyakov/ros2_crsf_receiver.git)
- [ackermann_mux](https://github.com/z1047941150/ackermann_mux.git)
- [Veddar VESC Interface](https://github.com/f1tenth/vesc)
- [FASTLIO2_ROS2](https://github.com/liangheming/FASTLIO2_ROS2.git)

##### All the code was written at FAST Lab, Zhejiang University.




## Future Work
- Add terrain traversability or elevation estimation module
- Add flexible path planning module
- Add current&acceleration calibration and control module
- Use a better simulation environment, like ISAAC Lab, Autodrive.
- Use RL to learn end-to-end policies.













