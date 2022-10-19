# ROS2 Foxy Installation
## Set locale
Make sure you have a local which support `UTF-8`.
```bash
    locale  # check for UTF-8
    
    sudo apt update && sudo apt install locales
    sudo locale-gen en_US en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8
    
    locale  # ver`ify settings
```
## Setup Sources
```bash
apt-cache policy | grep universe
```
This should output a line like the one below
```bash
500 http://us.archive.ubuntu.com/ubuntu focal/universe amd64 Packages
    release v=20.04,o=Ubuntu,a=focal,n=focal,l=Ubuntu,c=universe,b=amd64
```
If you do not see an output line like the one above, then:
```bash
# enable the Universe repository with these instructions.
sudo apt install software-properties-common
sudo add-apt-repository universe
# add ROS2 apt repository to your system.
sudo apt update && sudo apt install curl gnupg2 lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg
# add repository to your sources list
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```
## Install ROS2 packages
Update your apt repository caches after setting up the repositories.
Then, install ROS2 (recommended desktop version)
```bash
sudo apt update
sudo apt install ros-foxy-desktop
```
## Environment Setup
### Sourcing the setup script
Set up your environment by sourcing the following file.
```bash
# Replace ".bash" with your shell if you're not using bash
# Possible values are: setup.bash, setup.sh, setup.zsh
source /opt/ros/foxy/setup.bash
```
## Try some examples
In one terminal, source the setup file and then run Python `talker`:
```bash
source /opt/ros/foxy/setup.bash
ros2 run demo_nodes_py talker
```
In another terminal source the setup file and then run a Python `listener`:
```bash
source /opt/ros/foxy/setup.bash
ros2 run demo_nodes_py listener
```
You should see the `talker` saying that it is `Publishing` messages and the `listener` saying `I heard` those messages.
![](https://github.com/sh0hb0zbek/sms_lab/blob/main/pics/week_03_1.png)
## Uninstall
If you need to uninstall ROS2 or switch a source-based install once you have already installed from binaries, run the following command:
```bash
sudo apt remove ~nros-foxy-* && sudo apt autoremove
```
You may also want to remove the repository:
```bash
sudo rm /etc/apt/sources.list.d/ros2.list
sudo apt update
sudo apt autoremove
# Consider upgrading for packages previously shadowed.
sudo apt upgrade
```

# Using `turtlesim` and `rqt`
## Install `turtlesim`
Install the `turtlesim` package for your ROS2 distro:
```bash
sudo apt update
sudo apt install ros-foxy-turtlesim
# check that the package installed:
ros2 pkg excecutable turtlesim
```
The above command should return a list of `turtlesim`'s executables:
![](https://github.com/sh0hb0zbek/sms_lab/blob/main/pics/week_03_2.png)
## Start `turtlesim`
To start `turtlesim`, enter the following command in your terminal:
```bash
ros2 run turtlesim turtlesim_node
```
The simulator window should appear, with a random turtle in the center.
![](https://github.com/sh0hb0zbek/sms_lab/blob/main/pics/week_03_3.png)
In the terminal un the command you will see messages from the node:
![](https://github.com/sh0hb0zbek/sms_lab/blob/main/pics/week_03_4.png)
## Use `turtlesim`
Open a new terminal and source ROS2 again.
Now you will run a new code to control the turtle in the first node:
```bash
ros2 run turtlesim turtle_teleop_key
```
You can see the nodes and their associated services, topics, and actions using the `list` command:
```bash
ros2 node list
ros2 topic list
ros2 service list
ros2 action list
```
![](https://github.com/sh0hb0zbek/sms_lab/blob/main/pics/week_03_5.png)
## Install `rqt`
Open a new terminal to install `rqt` and its plugins:
```bash
sudo apt update
sudo apt install ~nros-foxy-rqt*
```
To run `rqt`
```bash
rqt
```
## Remapping
In a new terminal, source the ROS2, and run:
```bash
ros2 run turtlesim turtle_teleop_key --ros-args --remap turtle1/cmd_vel:=turtle2/cmd_vel
```
Now you can move turtle2 when this terminal is active, and turtle1 when other terminal running the `turtle_teleop_key` is active.
![](https://github.com/sh0hb0zbek/sms_lab/blob/main/pics/week_03_6.png)
## Close `turtlesim`
To stop the simulation, you can enter `Ctrl + C` in the `turtlesim_node` terminal, and `q` in the `teleop` terminal