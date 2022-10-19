# Using `colcon` to build packages
## Install `colcon`
```bash
sudo apt install python3-colcon-common-extensions
```
## Install ROS2
If you have not installed ROS2, then install it first.
Follow the installation [instructions](./week_03.md).
## Setup `colcon_cd`
The command `colcon_cd` allows you to quickly change the current working directory of your shell to the directory of a package.
```bash
echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> ~/.bashrc
echo "export _colcon_cd_root=/opt/ros/foxy/" >> ~/.bashrc
```
## Setup `colcon` tab completion
The command `colcon` [support command completion](https://colcon.readthedocs.io/en/released/user/installation.html#enable-completion) for bash and bash-like shells if the `colcon-argcomplete` package is installed.
```bash
echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc
```

# Creating a workspace
## Prerequisites
- [ROS2 installation](./week_03.md)
- `colcon` installation
- [`git` installation](https://git-scm.com/book/en/v2/Getting-Started-Installing-Git)
- [`turtlesim` installation](./week_03.md)
## Source ROS2 environment
```bash
source /opt/ros/foxy/setup.bash
```
## Create a new directory
```bash
mkdir -p ~/test_ws/src
cd ~/test_ws/src
```
## Clone a sample repo
```bash
git clone https://github.com/ros/ros_tutorials.git -b foxy-devel
```
## Resolve dependencies
```bash
# cd if you're still in the ``src`` directory with the ``ros_tutorials`` clone
cd ..
rosdep install -i --from-path src --rosdistro foxy -y
```
If you already have all your dependencies, the  console will return:
![](https://github.com/sh0hb0zbek/sms_lab/blob/main/pics/week_04_1.png)
## Build the workspace with `colcon`
```bash
colcon build
```
![](https://github.com/sh0hb0zbek/sms_lab/blob/main/pics/week_04_2.jpeg)

# Creating a package
## Create a package
Use the workspace you created `test_ws` \
Make sure you are in the `src` folder before running the package creation command
```bash
cd ~/test_ws/src
```
The command syntax for creating a new package in ROS2 is:
```bash
ros2 pkg create --build-type ament_python <package_name>
```
Enter the following command in your terminalL
```bash
ros2 pkg create --build-type ament_python --node-name my_node my_package
```
![](https://github.com/sh0hb0zbek/sms_lab/blob/main/pics/week_04_3.png)

## Build a package
Return to the root of your workspace and build your packages:
```bash
cd ~/test_ws/
colcon build
```
![](https://github.com/sh0hb0zbek/sms_lab/blob/main/pics/week_04_4.png)
To build only the `my_package` package next time, you can run:
```bash
colcon build --package-select my_package
```
![](https://github.com/sh0hb0zbek/sms_lab/blob/main/pics/week_04_5.png)
## Source the `setup.bash` file
Inside the root of your workspace run the following command to source your workspace:
```bash
. install/setup.bash
```
## Use the package
To tun the executable enter the command:
```bash
ros2 run my_package my_node
```
![](https://github.com/sh0hb0zbek/sms_lab/blob/main/pics/week_04_6.png)
## Examine package contents
Inside `~/test_ws/src/my_package`, you will see the files and folders that `ros2 pkg create` automatically generated:
![](https://github.com/sh0hb0zbek/sms_lab/blob/main/pics/week_04_7.png)
Note! `my_node.py` is inside the `my_package` directory. This is where all your custom Python nodes will go in the future.
## Customize `package.xml`
```xml
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
<name>my_package</name>
<version>0.0.0</version>
<description>TODO: Package description</description>
<maintainer email="shkhalimjonov@gmail.com">shokhbozbek</maintainer>
<license>TODO: License declaration</license>
<test_depend>ament_copyright</test_depend>
<test_depend>ament_flake8</test_depend>
<test_depend>ament_pep257</test_depend>
<test_depend>python3-pytest</test_depend>
<export>
<build_type>ament_python</build_type>
</export>
</package>
```
Customize your name and email on the `maintainer` line if it has not been automatically populated for you. Then edit the `description` line to summarize the package:
```xml
<description>Beginner client libraries tutorials practice package</description>
```
Then update the `licence` line. You can read more about open source licenses [here](https://opensource.org/licenses/alphabetical). Since this package is only for practice, it is safe to use any license. We use `Apache License 2.0`:
```xml
<license>Apache License 2.0</license>
```
## Customize `setup.py`
```python
from setuptools import setup

package_name = 'my_package'

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
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_node = my_package.my_node:main'
        ],
    },
)
```
Edit the `maintainer`, `maintainer_email`, `description` and `license` lines to match `package.xml`.
Do not forget to save the files.