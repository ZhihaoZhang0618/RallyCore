sudo apt install net-tools openssh-server

sudo service ssh start

sudo service ssh status

ifconfig

#### nomachie for x86
wget https://download.nomachine.com/download/8.13/Linux/nomachine_8.13.1_1_amd64.deb

sudo dpkg -i nomachine_8.13.1_1_amd64.deb

#### zsh
sudo apt install zsh git curl terminator vim -y

sh -c "$(curl -fsSL https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh)"

git clone https://github.com/zsh-users/zsh-syntax-highlighting.git $ZSH_CUSTOM/plugins/zsh-syntax-highlighting

git clone https://github.com/zsh-users/zsh-autosuggestions $ZSH_CUSTOM/plugins/zsh-autosuggestions

#### need to add sth in .zshrc

plugins=(git
zsh-autosuggestions
zsh-syntax-highlighting
)

setopt no_nomatch

source /opt/ros/humble/setup.zsh

source ~/RallyCore/install/setup.zsh
source ~/livox_ws/install/setup.zsh

eval "$(register-python-argcomplete3 ros2)"

eval "$(register-python-argcomplete3 colcon)"

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

export CYCLONEDDS_URI=~/cyclonedds.xml # optional

alias base='ros2 launch f1tenth_system base.launch.py'

alias nav='ros2 launch f1tenth_system nav.launch.py'

alias slam='ros2 launch f1tenth_system slam.launch.py'

alias localization='ros2 launch f1tenth_system localization_slam.launch.py'

#### when you in China, use this to install ROS2
wget http://fishros.com/install -O fishros && bash fishros


git clone https://github.com/gbmhunter/CppLinuxSerial.git

cd CppLinuxSerial

mkdir build && cd build

cmake .. && make

sudo make install

git clone https://github.com/RoverRobotics-forks/serial-ros2.git

cd serial-ros2

mkdir build && cd build

cmake .. && make

sudo make install

sudo apt install ros-humble-asio-cmake-module libasio-dev rapidjson-dev gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl libfuse2 libxkbcommon-x11-0 libxcb-cursor-dev -y

setopt no_nomatch && sudo apt install ros-humble-udp-msgs* ros-humble-ackermann-* ros-humble-plotjuggler* ros-humble-diagnostic-* ros-humble-robot-localization* ros-humble-rmw-cyclonedds-* ros-humble-slam-toolbox* ros-humble-rqt* ros-humble-nav2* ros-humble-tf* ros-humble-map* ros-humble-pcl-* -y

#### follow livox driver readme to compile
https://github.com/Livox-SDK/livox_ros_driver2

#### install main 
git clone --recursive git@github.com:z1047941150/RallyCore.git

cd PhoenixCore && git submodule update --init --recursive && colcon build

sudo cp rules/*.rules /etc/udev/rules.d

sudo apt remove brltty

#### modify wired ip
192.168.1.5	255.255.255.0	192.168.1.1
