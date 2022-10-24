# Creating an `action`
## Create a package
Create a package named `action_tutorials_interfaces`:
```bash
cd ~/test_ws/src/
ros2 pkg create action_tutorials_interfaces
```
![](https://github.com/sh0hb0zbek/sms_lab/blob/main/pics/week_06_1.png)
## Defining an `action`
Action are defined in `.action` files of the form:
```txt
# Request
---
# Result
---
# Feedback
```
An `action` definition is made up of three message definitions separated by `---`.
- `Request` message is sent from an action client to an action server initiating a new goal.
- `Result` message is sent from an action server to an action client when a goal is done.
- `Feedback` messages are periodically sent from an action server to an action client with updates about a goal.
Create an `action` directory, create a file called `Fibonacci.action` with the following contents:
```action
int32 order
---
int32[] sequence
---
int32[] partial_sequence
```
The goal request is the `order` of the Fibonacci sequence we want to compute, the result to the final `sequence`, and the feedback is the `partial_sequence` computed so far.
## Building an `action`
We must pass the definition to the rosidl code generation pipeline.\
This is accomplished by adding the following lines to our `CMakeLists.txt` before the `ament_package()` line:
```txt
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Fibonacci.action"
)
```
We should also add the required dependencies to our `package.xml`:
```xml
<buildtool_depend>rosidl_default_generators</buildtool_depend>
<depend>action_msgs</depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```
Build the package containing the `Fibonacci` action definition.\
We can check that our action build successfully with the command line tool.\
At the end, the Fibonacci action definition is printed to the screen:
```bash
cd ~/test_ws/
colcon build
. install/setup.bash
ros2 interface show action_tutorials_interfaces/action/Fibonacci
```
![](https://github.com/sh0hb0zbek/sms_lab/blob/main/pics/week_06_2.png)

# Writing an action server and client
## Writing an `action` server
Open a new file called `fibonacci_action_server.py`, and add the following code:
```python
import time
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from action_tutorials_interfaces.action import Fibonacci


class FibonacciActionServer(Node):
    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        feedback_msg = Fibonacci.Feedback()
        feedback_msg.partial_sequence = [0, 1]
        for i in range(1, goal_handle.request.order):
            feedback_msg.partial_sequence.append(
                feedback_msg.partial_sequence[i] + feedback_msg.partial_sequence[i-1])
            self.get_logger().info('Feedback: {0}'.format(feedback_msg.partial_sequence))
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(1)
        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = feedback_msg.partial_sequence
        return result


def main(args=None):
    rclpy.init(args=args)
    fibonacci_action_server = FibonacciActionServer()
    rclpy.spin(fibonacci_action_server)


if __name__ == '__main__':
    main()
```
![](https://github.com/sh0hb0zbek/sms_lab/blob/main/pics/week_06_3.png)
## Writing an `action` client
Open a new file called `fibonacci_action_client.py`, and add the following code:
```python
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from action_tutorials_interfaces.action import Fibonacci
import sys


class FibonacciActionClient(Node):
    def __init__(self):
        super().__init__('fibonacci_action_client')
        self._action_client = ActionClient(self, Fibonacci, 'fibonacci')

    def send_goal(self, order):
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order
        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.sequence))
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback.partial_sequence))


def main(order=1, args=None):
    rclpy.init(args=args)
    action_client = FibonacciActionClient()
    action_client.send_goal(order)
    rclpy.spin(action_client)


if __name__ == '__main__':
    main(order=int(sys.argv[1]))
```
![](https://github.com/sh0hb0zbek/sms_lab/blob/main/pics/week_06_4.png)

# Creating a `launch` file
## Setup
Create a new directory to store launch files
```bash
cd ~/test_ws/
mkdir launch
```
## Write the `launch` file
Copy and past the complete code into the `launch/turtlesim_mimic_launch.py` file:
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            namespace='turtlesim1',
            executable='turtlesim_node',
            name='sim'
        ),
        Node(
            package='turtlesim',
            namespace='turtlesim2',
            executable='turtlesim_node',
            name='sim'
        ),
        Node(
            package='turtlesim',
            executable='mimic',
            name='mimic',
            remappings=[
                ('/input/pose', '/turtlesim1/turtle1/pose'),
                ('/output/cmd_vel', '/turtlesim2/turtle1/cmd_vel'),
            ]
        )
    ])
```
## `ros2 launch`
```bash
cd ~/test_ws/launch
ros2 launch turlesim_mimic_launch.py
```
![](https://github.com/sh0hb0zbek/sms_lab/blob/main/pics/week_06_5.png)

# Integrating `launch` files into ROS2 packages
## Create a package
```bash
cd ~/test_ws/src/
ros2 pkg create py_launch_example --build-type ament_python
```
![](https://github.com/sh0hb0zbek/sms_lab/blob/main/pics/week_06_6.png)
## Creating the structure to hold `launch` files
`setup.py` file:
```python
from setuptools import setup
import os
from glob import glob

package_name = 'py_launch_example'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='shokhbozbek',
    maintainer_email='shkhalimjonov@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)

```
## Writing the `launch` file
Create a new directory called `launch` and paste the following code to a new file named `my_script_launch.py`:
```bash
cd ~/test_ws/src/py_launch_example/
```
my_script_launch.py
```python
import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='demo_nodes_cpp',
            executable='talker',
            name='talker'),
  ])
```
## Building and running the `launch` file
```bash
cd ~/test_ws/
colcon build --packages-select py_launch_example
. install/setup.bash
ros2 launch py_launch_example my_script_launch.py
```
![](https://github.com/sh0hb0zbek/sms_lab/blob/main/pics/week_06_7.png)
