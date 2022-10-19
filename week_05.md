# Writing a simple publisher subscriber
## Create a package
```bash
cd ~/test_ws/src/
ros2 pkg create --build-type ament_python py_pubsub
```
## Write the `publisher` node
```bash
cd py_pubsub/py_pubsub
wget https://raw.githubusercontent.com/ros2/examples/foxy/rclpy/topics/minimal_publisher/examples_rclpy_minimal_publisher/publisher_member_function.py
```
![](https://github.com/sh0hb0zbek/sms_lab/blob/main/pics/week_05_1.png)
[publisher_member_function.py](https://raw.githubusercontent.com/ros2/examples/foxy/rclpy/topics/minimal_publisher/examples_rclpy_minimal_publisher/publisher_member_function.py)
```python     
import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Add dependencies
Open `py_pubsub/package.xml` with text editor and make sure to fill in the `<description>`, `<maintainer>` and `<license>` tags:
```xml
<description>Examples of minimal publisher/subscriber using rclpy</description>
<maintainer email="shkhalimjonov@gmail.com">shokhbozbek</maintainer>
<license>Apache License 2.0</license>
```
Add the following dependencies corresponding to node's import statements:
```xml
<exec_depend>rclpy</exec_depend>
<exec_depend>std_msgs</exec_depend>
```
This declares the package needs `rclpy` and `std_msgs` when its code is executed.
Final form of `package.xml` file:
```xml 
    <?xml version="1.0"?>
    <?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
    <package format="3">
      <name>py_pubsub</name>
      <version>0.0.0</version>
      <description>Examples of minimal publisher/subscriber using rclpy</description>
      <maintainer email="shkhalimjonov@gmail.com">shokhbozbek</maintainer>
      <license>Apache License 2.0</license>
    
      <test_depend>ament_copyright</test_depend>
      <test_depend>ament_flake8</test_depend>
      <test_depend>ament_pep257</test_depend>
      <test_depend>python3-pytest</test_depend>
    
      <exec_depend>rclpy</exec_depend>
      <exec_depend>std_msgs</exec_depend>
    
      <export>
        <build_type>ament_python</build_type>
      </export>
    </package>
```
### Add an entry point
Open the `py_pubsub/setup.py` file. Again, match the `maintainer`, `maintainer_email`, `description` and `license` fields to the `package.xml` file:
```python
maintainer='shokhbozbek',
maintainer_email='shkhalimjonov@gmail.com',
description='Examples of minimal publisher/subscriber using rclpy',
license='Apache License 2.0',
```
Add the following line within `console_scripts` brackets of the `entry_points` field:
```python
entry_points={
        'console_scripts': [
                'talker = py_pubsub.publisher_member_function:main',
        ],
},
```
## Write the `subscriber` node
Return to `~/test_ws/src/py_pubsub/py_pubsub/` to create the next node. Enter the following code in the terminal:
```bash
cd ~/test_ws/src/py_pubsub/py_pubsub/
wget https://raw.githubusercontent.com/ros2/examples/foxy/rclpy/topics/minimal_subscriber/examples_rclpy_minimal_subscriber/subscriber_member_function.py
```
![](https://github.com/sh0hb0zbek/sms_lab/blob/main/pics/week_05_2.png)\
Now the directory should have these files:
![](https://github.com/sh0hb0zbek/sms_lab/blob/main/pics/week_05_3.png)
\
\
(subscriber_member_function.py)[https://raw.githubusercontent.com/ros2/examples/foxy/rclpy/topics/minimal_subscriber/examples_rclpy_minimal_subscriber/subscriber_member_function.py]
```python     
import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```
### Add an entry point
Reopen `py_pubsub/setup.py` and add the entry point for the subscriber node below the publisher's entry point. The `entry_points` field should now look like this:
```python
entry_points={
        'console_scripts': [
                'talker = py_pubsub.publisher_member_function:main',
                'listener = py_pubsub.subscriber_member_function:main',
        ],
},
```
\
\
Final form of `setup.py` file:
```python
from setuptools import setup

package_name = 'py_pubsub'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='shokhbozbek',
    maintainer_email='shkhalimjonov@gmail.com',
    description='Examples of minimal publisher/subscriber using rclpy',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = py_pubsub.publisher_member_function:main',
            'listener = py_pubsub.subscriber_member_function:main',
        ],
    },
)
```
## Build and run
Check for missing dependencies before building:
```bash
rosdep install -i --from-path src --rosdistro foxy -y
```
Make sure you are in the root of the workspace, build the package and source the setup files:
```bash
colcon build --packages-select py_pubsub
. install/setup.bash
```
![](https://github.com/sh0hb0zbek/sms_lab/blob/main/pics/week_05_4.png)\
Now run the `talker` node:
```bash
ros2 run py_pubsub talker
```
Open a new terminal and source the setup files from inside `test_ws` again, and then start the `listener` node:
```bash
ros2 run py_pubsub listener
```
![](https://github.com/sh0hb0zbek/sms_lab/blob/main/pics/week_05_5.png)

# Writing a simple service and client
## Create a package
Navigate into the `test_ws/src/` and create a new package
```bash
cd ~/test_ws/src/
ros2 pkg create --build-type ament_python py_srvcli --dependencies rclpy example_interfaces
```
![](https://github.com/sh0hb0zbek/sms_lab/blob/main/pics/week_05_6.png)
### Update `package.xml`
```xml
<description>Python client server tutorial</description>
<maintainer email="shkhalimjonov@gmail.com">shokhbozbek</maintainer>
<license>Apache License 2.0</license>
```
### Update `setup.py`
```python
maintainer='shokhbozbek',
maintainer_email='shkhalimjonov@gmail.com',
description='Python client server tutorial',
license='Apache License 2.0',
```
## Write the `service` node
Inside the `~/test_ws/src/py_srvcli/py_srvcli/` directory, create a new file called `service_member_function.py` and paste the following code within:
```python
from example_interfaces.srv import AddTwoInts

import rclpy
from rclpy.node import Node


class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))

        return response


def main(args=None):
    rclpy.init(args=args)

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
```
### Add an entry point
Add the following line between the `'console_scripts':` brackets:
```python
'service = py_srvcli.service_member_function:main',
```
## Write the `client` node
Inside the `~/test_ws/src/py_srvcli/py_srvcli/` directory, create a new file called `client_member_function.py` and paste the following code within:
```python
import sys

from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node


class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main(args=None):
    rclpy.init(args=args)

    minimal_client = MinimalClientAsync()
    response = minimal_client.send_request(int(sys.argv[1]), int(sys.argv[2]))
    minimal_client.get_logger().info(
        'Result of add_two_ints: for %d + %d = %d' %
        (int(sys.argv[1]), int(sys.argv[2]), response.sum))

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```
### Add an entry point
Add the following line between the `'console_scripts':` brackets:
```python
'client = py_srvcli.client_member_function:main',
```
Final `entry_points` should like this:
```python
entry_points={
    'console_scripts': [
        'service = py_srvcli.service_member_function:main',
        'client  = py_srvcli.client_member_function:main',
    ],
},
```
## Build and run
Check for missing dependencies before building:
```bash
rosdep install -i --from-path src --rosdistro foxy -y
```
Make sure you are in the root of the workspace, build the package and source the setup files:
```bash
colcon build --packages-select py_srvcli
. install/setup.bash
```
![](https://github.com/sh0hb0zbek/sms_lab/blob/main/pics/week_05_7.png)\
Now run the `service` node:
```bash
ros2 run py_srvcli service
```
Open a new terminal and source the setup files from inside `test_ws` again, and then start the `client` node, followed by any two integers separated by a space:
```bash
ros2 run py_srvcli client 5 6
```
![](https://github.com/sh0hb0zbek/sms_lab/blob/main/pics/week_05_8.png)\
\
Enter `Ctrl+C` in the server terminal to stop the node from spinning.

# Create custom `msg` and `srv` files
## Create a new package
Navigate into the `test_ws/src/` and create a new package
```bash
cd ~/test_ws/src/
ros2 pkg create --build-type ament_cmake tutorial_interfaces
```
Make `msg` and `srv` directories inside package directory:
```bash
cd tutorial_interfaces/
mkdir msg
mkdir srv
```
![](https://github.com/sh0hb0zbek/sms_lab/blob/main/pics/week_05_9.png)\
## Create custom definitions
### `msg` definition
In the `tutorial_interfaces/msg` directory you just created, make a new file called `Num.msg` with one line of code declaring its data structure:
```msg
int64 num
```
Also in the `tutorial_interfaces/msg` directory you just created, make a new file called `Sphere.msg` with the following content:
```msg
geometry_msgs/Point center
float64 radius
```
### `srv` definition
Back in the `tutorial_interfaces/srv` directory you just created, make a new file called `AddThreeInts.srv` with the following request and response structure:
```srv
int64 a
int64 b
int64 c
---
int64 sum
```
![](https://github.com/sh0hb0zbek/sms_lab/blob/main/pics/week_05_10.png)\
## `CMakeaLists.txt`
```cmake
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Num.msg"
  "msg/Sphere.msg"
  "srv/AddThreeInts.srv"
  DEPENDENCIES geometry_msgs # Add packages that above messages depend on, in this case geometry_msgs for Sphere.msg
)
```
## `package.xml`
```xml
<depend>geometry_msgs</depend>
<build_depend>rosidl_default_generators</build_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```
## Build the `tutorial_interfaces` package
```bash
cd ~/test_ws/
colcon build --packages-select tutorial_interfaces
```
![](https://github.com/sh0hb0zbek/sms_lab/blob/main/pics/week_05_11.png)
## Confirm `msg` and `srv` creation
```bash
cd ~/test_ws/
. install/setup.bash
ros2 interface show tutorial_interfaces/msg/Num
ros2 interface show tutorial_interfaces/msg/Sphere
ros2 interface show tutorial_interfaces/srv/AddThreeInts
```
![](https://github.com/sh0hb0zbek/sms_lab/blob/main/pics/week_05_12.png)

# Implementing custom interfaces
## Create a package
Navigate into the `test_ws/src/`, create a new package and make a folder within it for `msg` files:
```bash
cd ~/test_ws/src/
ros2 pkg create --build-type ament_cmake more_interfaces
mkdir more_interfaces/msg
```
![](https://github.com/sh0hb0zbek/sms_lab/blob/main/pics/week_05_13.png)
## Create a `msg` file
Inside `more_interfaces/msg`, create a new file `AddressBook.msg`.
Paste the following code to create a message meant to carry information about an individual:
```msg
bool FEMALE=true
bool MALE=false

string first_name
string last_name
bool gender
uint8 age
string address
```
### Build a `msg` file
Open `more_intertfaces/package.xml`, and add the following lines:
```xml
<buildtool_depend>rosidl_default_generators</buildtool_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```
Open `more_interfaces/CMakeLists.txt` and add the following lines:
```cmake
# find the package that generates message code from msg/srv files:
find_package(rosidl_default_generators REQUIRED)
# declare the list of messages you want to generate:
set(msg_files
  "msg/AddressBook.msg"
)
# generate the messages:
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
)
# export the message runtime dependency:
ament_export_dependencies(rosidl_default_runtime)
```
## Use an interface from the same package
In `more_interfaces/src` create a file called `publish_address_book.cpp` and paste the following code:
```c
#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "more_interfaces/msg/address_book.hpp"

using namespace std::chrono_literals;

class AddressBookPublisher : public rclcpp::Node
{
public:
  AddressBookPublisher()
  : Node("address_book_publisher")
  {
    address_book_publisher_ =
      this->create_publisher<more_interfaces::msg::AddressBook>("address_book", 10);

    auto publish_msg = [this]() -> void {
        auto message = more_interfaces::msg::AddressBook();

        message.first_name = "John";
        message.last_name = "Doe";
        message.age = 30;
        message.gender = message.MALE;
        message.address = "unknown";

        std::cout << "Publishing Contact\nFirst:" << message.first_name <<
          "  Last:" << message.last_name << std::endl;

        this->address_book_publisher_->publish(message);
      };
    timer_ = this->create_wall_timer(1s, publish_msg);
  }

private:
  rclcpp::Publisher<more_interfaces::msg::AddressBook>::SharedPtr address_book_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AddressBookPublisher>());
  rclcpp::shutdown();

  return 0;
}
```
### Build publisher
We need to create a new target for this node in the `CMakeLists.txt`:
```cmake
find_package(rclcpp REQUIRED)

add_executable(publish_address_book
  src/publish_address_book.cpp
)

ament_target_dependencies(publish_address_book
  "rclcpp"
)

install(TARGETS publish_address_book
 DESTINATION lib/${PROJECT_NAME})
```
### Link against the interface
In order to use the messages generated in the same package we need to use the following CMake code:
```cmake
rosidl_target_interfaces(publish_address_book
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
```
## Try is out
Return to the root of the workspace to build the package:
```bash
cd ~/test_ws
colcon build --packages-up-to more_interfaces
```
Then source the workspace and run the publisher:
```bash
. install/local_setup.bash
ros2 run more_interfaces publish_address_book
```
To confirm the message is being published on the `address_book` topic, open another terminal, source the workspace, and call `topic echo`:
```bash
. install/setup.bash
ros2 topic echo /address_book
```
![](https://github.com/sh0hb0zbek/sms_lab/blob/main/pics/week_05_14.png)

# Using parameters in a class
## Create a package
Navigate into the `test_ws/src/` and create a new package
```bash
cd ~/test_ws/src/
ros2 pkg create --build-type ament_python python_parameters --dependencies rclpy
```
![](https://github.com/sh0hb0zbek/sms_lab/blob/main/pics/week_05_15.png)
### Update `package.xml'
Make sure to add the `description`, `maintainer` email and name, and `license` information to `package.xml`.
```xml
<description>Python parameter tutorial</description>
<maintainer email="shkhalimjonov@gmail.com">shokhbozbek</maintainer>
<license>Apache License 2.0</license>
```
## Write the Python node
Inside the `test_ws/src/python_parameters/python_parameters` directory, create a new file called `python_parameters_node.py` and paste the following code within:
```python
import rclpy
import rclpy.node

class MinimalParam(rclpy.node.Node):
    def __init__(self):
        super().__init__('minimal_param_node')

        self.declare_parameter('my_parameter', 'world')

        self.timer = self.create_timer(1, self.timer_callback)

    def timer_callback(self):
        my_param = self.get_parameter('my_parameter').get_parameter_value().string_value

        self.get_logger().info('Hello %s!' % my_param)

        my_new_param = rclpy.parameter.Parameter(
            'my_parameter',
            rclpy.Parameter.Type.STRING,
            'world'
        )
        all_new_parameters = [my_new_param]
        self.set_parameters(all_new_parameters)

def main():
    rclpy.init()
    node = MinimalParam()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
```
### Add an entry point
Open the `python_parameters/setup.py` file. Again, match the `maintainer`, `maintainer_email`, `description` and `license` fields to `python_parameters/package.xml`:
```python
maintainer='Shokhbozbek',
maintainer_email='shkhalimjonov@gmail.com',
description='Python parameter tutorial',
license='Apache License 2.0',
```
Add the following line within the `console_scripts` brackets of the `entry_points` field:
```python
entry_points={
    'console_scripts': [
        'minimal_param_node = python_parameters.python_parameters_node:main',
    ],
},
```
## Build and run
Check for missing dependencies before building:
```bash
rosdep install -i --from-path src --rosdistro foxy -y
```
Navigate back to the root of the workspace, `test_ws`, and build the new package:
```bash
colcon build --packages-select python_parameters
```
Source the setup files:
```bash
. install/setup.bash
```
![](https://github.com/sh0hb0zbek/sms_lab/blob/main/pics/week_05_16.png)\
Now run the node:
```bash
ros2 run python_parameters minimal_param_node
```
![](https://github.com/sh0hb0zbek/sms_lab/blob/main/pics/week_05_17.png)

# Using `ros2doctor` to identify issues
## Check the setup

## Check a system
Open a terminal and run the following command:
```bash
ros2 run turtlesim turtlesim_node
```
Open a new terminal and run the following command:
```bash
ros2 run turtlesim turtle_teleop_key
```
Now run `ros2doctor` in a new terminal:
```bash
ros2 doctor
```
![](https://github.com/sh0hb0zbek/sms_lab/blob/main/pics/week_05_18.jpeg)
\
\
If you run commands to echo the `/color_sensor` and `/pose` topics, those warnings will disappear because the publishers will have subscribers.
![](https://github.com/sh0hb0zbek/sms_lab/blob/main/pics/week_05_19.jpeg)

## Get a full report
To get a full report, enter the following command in the terminal:
```bash
ros2 doctor --report
```
Result:
```bash

   NETWORK CONFIGURATION
inet         : 192.168.50.37
inet4        : ['192.168.50.37']
ether        : b4:2e:99:f8:6d:52
inet6        : ['fe80::fc3f:23:e540:73d7']
netmask      : 255.255.255.0
device       : eno1
flags        : 4163<UP,BROADCAST,RUNNING,MULTICAST> 
mtu          : 1500
broadcast    : 192.168.50.255
inet         : 127.0.0.1
inet4        : ['127.0.0.1']
inet6        : ['::1']
netmask      : 255.0.0.0
device       : lo
flags        : 73<UP,LOOPBACK,RUNNING> 
mtu          : 65536

   PACKAGE VERSIONS
turtlebot3_simulations                    : required=2.2.3, local=2.2.3
turtlebot3_gazebo                         : required=2.2.3, local=2.2.3
... 
joint_state_broadcaster                   : required=0.8.2, local=0.8.2
examples_rclpy_minimal_publisher          : required=0.9.4, local=0.9.4

   PLATFORM INFORMATION
system           : Linux
platform info    : Linux-5.15.0-50-generic-x86_64-with-glibc2.29
release          : 5.15.0-50-generic
processor        : x86_64

   RMW MIDDLEWARE
middleware name    : rmw_fastrtps_cpp

   ROS 2 INFORMATION
distribution name      : foxy
distribution type      : ros2
distribution status    : active
release platforms      : {'ubuntu': ['focal']}

   TOPIC LIST
topic               : /turtle1/cmd_vel
publisher count     : 1
subscriber count    : 1
topic               : /turtle1/color_sensor
publisher count     : 1
subscriber count    : 1
topic               : /turtle1/pose
publisher count     : 1
subscriber count    : 1
topic               : /turtle1/rotate_absolute/_action/feedback
publisher count     : 1
subscriber count    : 1
topic               : /turtle1/rotate_absolute/_action/status
publisher count     : 1
subscriber count    : 1
```