# sm_lab# Turtlesim installation

### First of all, make sure your system is up-to-date
```
sudo apt update
```
### Install the turtlesim library
```
sh0hb0zbek@ubuntu:~$ sudo apt install ros-rolling-turtlesim
Reading package lists... Done
Building dependency tree       
Reading state information... Done
....
```
### Check that the package is installed
```
sh0hb0zbek@ubuntu:~$ ros2 pkg executables turtlesim
turtlesim draw_square
turtlesim mimic
turtlesim turtle_teleop_key
turtlesim turtlesim_node
```
### To start turtlesim, enter the following command in your terminal
```
sh0hb0zbek@ubuntu:~$ ros2 run turtlesim turtlesim_node
[INFO] [1663638092.636082943] [turtlesim]: Starting turtlesim with node name /turtlesim
[INFO] [1663638092.645975073] [turtlesim]: Spawning turtle [turtle1] at x=[5.544445], y=[5.544445], theta=[0.000000]
```
![](https://github.com/ULUGBEK12194914/turtlesim_project/blob/main/screenshots/Screen%20Shot%202022-09-21%20at%2014.03.15.png)
### To control the turtle type the following command:
```
sh0hb0zbek@ubuntu:~$ ros2 run turtlesim turtle_teleop_key
Reading from keyboard
---------------------------
Use arrow keys to move the turtle.
Use G|B|V|C|D|E|R|T keys to rotate to absolute orientations. 'F' to cancel a rotation.
'Q' to quit.
```
### RQT: installation
```bash
#  MAKE SURE YOUR SYSTEM IS UP-TO-DATE
sh0hb0zbek@ubuntu:~$ sudo apt update

# INSTALL THE RQT LIBRARY AND ITS PLUGINS
sh0hb0zbek@ubuntu:~$ sudo apt install ~nros-rolling-rqt*
Reading package lists... Done
Building dependency tree       
Reading state information... Done
The following additional packages will be installed:
  pybind11-dev ros-rolling-class-loader
  ...
# To run rqt by just typing command "rqt"
sh0hb0zbek@ubuntu:~$ rqt
```
![](https://github.com/ULUGBEK12194914/turtlesim_project/blob/main/screenshots/Screen%20Shot%202022-09-21%20at%2018.40.15.png)
# RQT: running rqt_console
```
sh0hb0zbek@ubuntu:~$ ros2 run rqt_console rqt_console
```
![](https://github.com/ULUGBEK12194914/turtlesim_project/blob/main/screenshots/Screen%20Shot%202022-09-21%20at%2021.45.00.png)
### Turtlesim simulation
```bash
# RUN THE TURTLESIM_NODE IN A NEW TAB
sh0hb0zbek@ubuntu:~$ ros2 run turtlesim turtlesim_node
[INFO] [1664067181.891114659] [turtlesim]: Starting turtlesim with node name /turtlesim
[INFO] [1664067181.940055496] [turtlesim]: Spawning turtle [turtle1] at x=[5.544445], y=[5.544445], theta=[0.000000]
# RUN THE TURTLE_TELEOP_KEY TO MOVE THE TURTLE IN A NEW TAB
sh0hb0zbek@ubuntu:~$ ros2 run turtlesim turtle_teleop_key
Reading from keyboard
---------------------------
Use arrow keys to move the turtle.
Use G|B|V|C|D|E|R|T keys to rotate to absolute orientations. 'F' to cancel a rotation.
'Q' to quit.
#RUN THE rqt in your terminal
sh0hb0zbek@ubuntu:~$ rqt
#CHANGE THE /SPAWN SERVICE PARAMETERS TO RUN TURTLE2
#TO RUN THE TURTLE TO RUN THIS COMMAND
sh0hb0zbek@ubuntu:~$ ros2 run turtlesim turtle_teleop_key --ros-args --remap turtle1/cmd_vel:=turtle2/cmd_vel
Reading from keyboard
---------------------------
Use arrow keys to move the turtle.
Use G|B|V|C|D|E|R|T keys to rotate to absolute orientations. 'F' to cancel a rotation.
'Q' to quit.
```
![](https://github.com/ULUGBEK12194914/turtlesim_project/blob/main/screenshots/Screen%20Shot%202022-09-25%20at%2010.11.58.png)
# TURTLE simulation
![](https://github.com/ULUGBEK12194914/turtlesim_project/blob/main/screenshots/Peek%202022-09-24%2018-24.gif)
### ROS2 Colcon
```bash
# INSTALLING Colcon
sh0hb0zbek@ubuntu:~$ sudo apt install python3-colcon-common-extensions
[sudo] password for sh0hb0zbek: 
Reading package lists... Done
Building dependency tree       
Reading state information... Done
python3-colcon-common-extensions is already the newest version (0.3.0-1).
0 upgraded, 0 newly installed, 0 to remove and 4 not upgraded.

-----------------------------------
 # ROS2 Colcon: Create a workspace
 ----------------------------------
sh0hb0zbek@ubuntu:~$ echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> ~/.bashrc
sh0hb0zbek@ubuntu:~$ echo "export _colcon_cd_root=/opt/ros/rolling/" >> ~/.bashrc
sh0hb0zbek@ubuntu:~$ source /opt/ros/rolling/setup.bash
sh0hb0zbek@ubuntu:~$ mkdir -p ~/ros2_ws/src
sh0hb0zbek@ubuntu:~$ cd ~/ros2_ws
sh0hb0zbek@ubuntu:~/ros2_ws$ git clone https://github.com/ros/ros_tutorials.git -b rolling-devel
Cloning into 'ros_tutorials'...
remote: Enumerating objects: 2841, done.
remote: Counting objects: 100% (161/161), done.
remote: Compressing objects: 100% (88/88), done.
remote: Total 2841 (delta 93), reused 131 (delta 71), pack-reused 2680
Receiving objects: 100% (2841/2841), 617.66 KiB | 7.72 MiB/s, done.
Resolving deltas: 100% (1710/1710), done.
# RESOLVE DEPENDENCIES
sh0hb0zbek@ubuntu:~/ros2_ws$ rosdep install -i --from-path src --rosdistro rolling -y
All required rosdeps installed successfully

# BUILD THE WORKSPACE WITH Colcon
sh0hb0zbek@ubuntu:~/ros2_ws$ colcon build
[1.070s] WARNING:colcon.colcon_core.package_selection:Some selected packages are already built in one or more underlay workspaces:
	'turtlesim' is in: /opt/ros/rolling, /opt/ros/foxy
If a package in a merged underlay workspace is overridden and i
...
This may be promoted to an error in a future release of colcon-override-check.
Starting >>> turtlesim
[Processing: turtlesim]                             
[Processing: turtlesim] 
....
# SOURCE THE OVERLAY
sh0hb0zbek@ubuntu:~$ source /opt/ros/rolling/setup.bash
sh0hb0zbek@ubuntu:~$ cd ~/ros2_ws
sh0hb0zbek@ubuntu:~/ros2_ws$ . install/local_setup.bash
# MODIFY THE OVERLAY
sh0hb0zbek@ubuntu:~/ros2_ws$ cd src
sh0hb0zbek@ubuntu:~/ros2_ws/src$ ls
sh0hb0zbek@ubuntu:~/ros2_ws/src$ git clone https://github.com/ros/ros_tutorials
Cloning into 'ros_tutorials'...
remote: Enumerating objects: 2841, done.
remote: Counting objects: 100% (161/161), done.
remote: Compressing objects: 100% (88/88), done.
remote: Total 2841 (delta 93), reused 131 (delta 71), pack-reused 2680
Receiving objects: 100% (2841/2841), 617.66 KiB | 855.00 KiB/s, done.
Resolving deltas: 100% (1710/1710), done.
sh0hb0zbek@ubuntu:~/ros2_ws/src$ ls
ros_tutorials
sh0hb0zbek@ubuntu:~/ros2_ws/src$ cd ros_tutorials
sh0hb0zbek@ubuntu:~/ros2_ws/src/ros_tutorials$ ls
roscpp_tutorials  rospy_tutorials  ros_tutorials  turtlesim
sh0hb0zbek@ubuntu:~/ros2_ws/src/ros_tutorials$ cd turtlesim/src
sh0hb0zbek@ubuntu:~/ros2_ws/src/ros_tutorials/turtlesim/src$ ls
turtle.cpp  turtle_frame.cpp  turtlesim  turtlesim.cpp
sh0hb0zbek@ubuntu:~/ros2_ws/src/ros_tutorials/turtlesim/src$ nano turtle_frame.cpp
```
# Create a package
```bash
sh0hb0zbek@ubuntu:~/ros2_ws/src$ ros2 pkg create --build-type ament_python --node-name my_node my_package
going to create a new package
package name: my_package
destination directory: /home/ulugbekmirzabakhromov/ros2_ws/src
package format: 3
version: 0.0.0
...
sh0hb0zbek@ubuntu:~/ros2_ws/src$ cd ..

# BUILDING PACKAGES
sh0hb0zbek@sh0hb0zbek:~/ros2_ws$ colcon build            
Summary: 3 packages finished [4.08s]
sh0hb0zbek@sh0hb0zbek:~/ros2_ws$ colcon build --packages-select my_package
Starting >>> my_package
Finished <<< my_package [1.14s]          
Summary: 1 package finished [1.37s]

#SOURCE THE SETUP FILE
sh0hb0zbek@sh0hb0zbek:~/ros2_ws$ . install/setup.bash
sh0hb0zbek@sh0hb0zbek:~/ros2_ws$ ros2 run my_package my_node
Hi from my_package.
```

# Writing a publisher and subscriber(Python)
```bash
#MAKE SURE YOU ARE IN THE src FOLDER AND ENTER THE COMMAND FOR CREATING THE PY_PACKAGE
sh0hb0zbek@sh0hb0zbek:~$ cd ~/ros2_ws
sh0hb0zbek@sh0hb0zbek:~/ros2_ws/src$ ros2 pkg create --build-type ament_python py_pubsub
going to create a new package
package name: py_pubsub
destination directory: /home/sh0hb0zbek/ros2_ws/src
package format: 3
version: 0.0.0
...
sh0hb0zbek@sh0hb0zbek:~/ros2_ws/src$ ls
my_package  my_pkg  py_pubsub  ros_tutorials
sh0hb0zbek@sh0hb0zbek:~/ros2_ws/src$ wget https://raw.githubusercontent.com/ros2/examples/foxy/rclpy/topics/minimal_publisher/examples_rclpy_minimal_publisher/publisher_member_function.py
--2022-09-27 21:42:13--  https://raw.githubusercontent.com/ros2/examples/foxy/rclpy/topics/minimal_publisher/examples_rclpy_minimal_publisher/publisher_member_function.py
...
2022-09-27 21:42:14 (11.5 MB/s) - ‘publisher_member_function.py’ saved [1576/1576]

#DOWNLOAD THE publisher_member_function.py TO ros2_ws/src/py_pubsub/py_pubsub DIRECTORY

sh0hb0zbek@sh0hb0zbek:~/ros2_ws/src/py_pubsub/py_pubsub$ wget https://raw.githubusercontent.com/ros2/examples/foxy/rclpy/topics/minimal_publisher/examples_rclpy_minimal_publisher/publisher_member_function.py
--2022-09-27 22:19:36--  https://raw.githubusercontent.com/ros2/examples/foxy/rclpy/topics/minimal_publisher/examples_rclpy_minimal_publisher/publisher_member_function.py
Resolving raw.githubusercontent.com (raw.githubusercontent.com)... 185.199.111.133, 185.199.108.133, 185.199.109.133, ...
...
2022-09-27 22:19:37 (31.7 MB/s) - ‘publisher_member_function.py’ saved [1576/1576]

#ALSO DOWNLOAD THE subscriber_member_function.py 
sh0hb0zbek@sh0hb0zbek:~/ros2_ws/src/py_pubsub/py_pubsub$ wget https://raw.githubusercontent.com/ros2/examples/foxy/rclpy/topics/minimal_subscriber/examples_rclpy_minimal_subscriber/subscriber_member_function.py
...
2022-09-27 21:54:36 (18.1 MB/s) - ‘subscriber_member_function.py’ saved [1469/1469]
```
# Build and run
```bash
sh0hb0zbek@sh0hb0zbek:~/ros2_ws$ rosdep install -i --from-path src --rosdistro foxy -y
#All required rosdeps installed successfully
sh0hb0zbek@sh0hb0zbek:~/ros2_ws$ colcon build --packages-select py_pubsub
Starting >>> py_pubsub
Finished <<< py_pubsub [2.89s]          

Summary: 1 package finished [4.10s]
sh0hb0zbek@sh0hb0zbek:~/ros2_ws$ rosdep install -i --from-path src --rosdistro foxy -y
#All required rosdeps installed successfully
sh0hb0zbek@sh0hb0zbek:~/ros2_ws$ colcon build --packages-select py_pubsub
Starting >>> py_pubsub
Finished <<< py_pubsub [1.41s]          

Summary: 1 package finished [1.66s]

---------------------------------------
# TESTING THE NODES
---------------------------------------
sh0hb0zbek@sh0hb0zbek:~/ros2_ws$ . install/setup.bash
sh0hb0zbek@sh0hb0zbek:~/ros2_ws$ ros2 run py_pubsub talker
[INFO] [1664342591.577268871] [minimal_publisher]: Publishing: "Hello World: 0"
[INFO] [1664342592.061454408] [minimal_publisher]: Publishing: "Hello World: 1"
[INFO] [1664342592.562140724] [minimal_publisher]: Publishing: "Hello World: 2"
[INFO] [1664342593.062828040] [minimal_publisher]: Publishing: "Hello World: 3"
sh0hb0zbek@sh0hb0zbek:~/ros2_ws$ . install/setup.bash
sh0hb0zbek@sh0hb0zbek:~/ros2_ws$ ros2 run py_pubsub talker
[INFO] [1664342591.577268871] [minimal_publisher]: Publishing: "Hello World: 0"
[INFO] [1664342592.061454408] [minimal_publisher]: Publishing: "Hello World: 1"
[INFO] [1664342592.562140724] [minimal_publisher]: Publishing: "Hello World: 2"
[INFO] [1664342593.062828040] [minimal_publisher]: Publishing: "Hello World: 3"
```
# Writing a simple service and client(Python)
```bash

#Firstly, create the package into the ros2_ws/src directory
------------------------------------------------------------
sh0hb0zbek@sh0hb0zbek:~$ cd ros2_ws
sh0hb0zbek@sh0hb0zbek:~/ros2_ws$ cd src
sh0hb0zbek@sh0hb0zbek:~/ros2_ws/src$ ros2 pkg create --build-type ament_python py_srvcli --dependencies rclpy example_interfaces
going to create a new package
package name: py_srvcli
....

# Write the service node inside the ros2_ws/src/py_srvcli/py_srvcli directory:
------------------------------------------------------------------------------
sh0hb0zbek@sh0hb0zbek:~/ros2_ws/src/py_srvcli$ nano service_member_function.py
sh0hb0zbek@sh0hb0zbek:~/ros2_ws/src/py_srvcli$ nano client_member_function.py

# Add an entry points into the setup.py to be able to run the servic&client nodes.
----------------------------------------------------------------------------------
sh0hb0zbek@sh0hb0zbek:~/ros2_ws/src/py_srvcli$ nano setup.py
```
![](https://github.com/ULUGBEK12194914/turtlesim_project/blob/main/screenshots/Screen%20Shot%202022-10-04%20at%2011.20.46.png)
```bash
# BUILD AND RUN
----------------
sh0hb0zbek@sh0hb0zbek:~/ros2_ws$ rosdep install -i --from-path src --rosdistro foxy -y
#All required rosdeps installed successfully
sh0hb0zbek@sh0hb0zbek:~/ros2_ws$ colcon build --packages-select py_srvcli
Starting >>> py_srvcli
Finished <<< py_srvcli [2.13s]          
Summary: 1 package finished [2.48s]

# Open a new terminal, navigate to ros2_ws, and source the setup files
----------------------------------------------------------------------
sh0hb0zbek@sh0hb0zbek:~/ros2_ws$ . install/setup.bash
sh0hb0zbek@sh0hb0zbek:~/ros2_ws$ ros2 run py_srvcli service
[INFO] [1664849371.615817085] [minimal_service]: Incoming request
a: 2 b: 3
# Do the same thing for the client node
sh0hb0zbek@sh0hb0zbek:~/ros2_ws$ . install/setup.bash
sh0hb0zbek@sh0hb0zbek:~/ros2_ws$ ros2 run py_srvcli client 2 3
[INFO] [1664849371.665846561] [minimal_client_async]: Result of add_two_ints: for 2 + 3 = 5
```
# Writing an action server and client(Python)
```bash
#Writing an action server
-------------------------
sh0hb0zbek@sh0hb0zbek:~$ nano fibonacci_action_server.py

#You can get the action_server code from this link:
---------------------------------------------------
```

```bash
#Run the action server
sh0hb0zbek@sh0hb0zbek:~$ python3 fibonacci_action_server.py 
[INFO] [1664948740.009765307] [fibonacci_action_server]: Executing goal...
[WARN] [1664948740.010891932] [fibonacci_action_server]: Goal state not set, assuming aborted. Goal ID: [ 43 132  59 102 136 104  68 222 136  35 140  96 233 172 179 145]

#In another terminal, use the command line interface to send a goal:
--------------------------------------------------------------------
sh0hb0zbek@sh0hb0zbek:~$ ros2 action send_goal fibonacci action_tutorials_interfaces/action/Fibonacci "{order: 5}"
Waiting for an action server to become available...
Sending goal:
     order: 5

Goal accepted with ID: 15197cd7683f45baa7c2facdddd31a74

Result:
    sequence:
- 0
- 1
- 1
- 2
- 3
- 5

Goal finished with status: SUCCEEDED

# Writing a action_client server
-------------------------
sh0hb0zbek@sh0hb0zbek:~$ nano fibonacci_action_client.py

#You can get the action_client code from this link:
---------------------------------------------------
```

```bash
#Run the action client
-----------------------
sh0hb0zbek@sh0hb0zbek:~$ python3 fibonacci_action_client.py
[INFO] [1664949683.787442150] [fibonacci_action_client]: Goal accepted :)
[INFO] [1664949683.790887484] [fibonacci_action_client]: Received feedback: array('i', [0, 1, 1])
[INFO] [1664949684.794285170] [fibonacci_action_client]: Received feedback: array('i', [0, 1, 1, 2])
[INFO] [1664949685.796666631] [fibonacci_action_client]: Received feedback: array('i', [0, 1, 1, 2, 3])
[INFO] [1664949686.799187578] [fibonacci_action_client]: Received feedback: array('i', [0, 1, 1, 2, 3, 5])
....

#In another terminal, run the action_server:
--------------------------------------------------------------------
sh0hb0zbek@sh0hb0zbek:~$ python3 fibonacci_action_server.py 
[INFO] [1664949683.788839423] [fibonacci_action_server]: Executing goal...
[INFO] [1664949683.789502547] [fibonacci_action_server]: Feedback: array('i', [0, 1, 1])
[INFO] [1664949684.792395799] [fibonacci_action_server]: Feedback: array('i', [0, 1, 1, 2])
[INFO] [1664949685.794762732] [fibonacci_action_server]: Feedback: array('i', [0, 1, 1, 2, 3])
[INFO] [1664949686.797654565] [fibonacci_action_server]: Feedback: array('i', [0, 1, 1, 2, 3, 5])
....
```

