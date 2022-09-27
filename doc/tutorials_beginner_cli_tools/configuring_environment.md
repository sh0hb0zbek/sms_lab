# Tasks
## 1 Source the setup files
You will need to run this command on every new shell you open to have access to the ROS 2 commands, like so:
```bash
  # Replace ".bash" with your shell if you are not using bash
  # Possible values are: setup.bash, setup.sh, setup.zsh
  source /opt/ros/foxy/setup.bash
```
## 2 Add sourcing to your shell setup script
If you don't want to have to source the setup file every time you open a new shell (skipping Task 1), then you can add the command to your shell startup script:
```bash
  echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc
```
To undo this local your system's shell script and remove the appended source command

## 3 Check environment variiables
Sourcing ROS2 setup files will set several environment variables neccessary for operating ROS 2. If you ever have problems finding or using your ROS 2 packages, make sure that your environment is properly setup the following command.
```bash
  printenv | grep -i ROS
```
Cehck that variables like `ROS_DISTRO` and `ROS_VERSION` are set and output will be:
```
  ROS_VERSION=2
  ROS_PYTHON_VERSION=3
  ROS_DISTRO=foxy
```
### 3.1 The `ROS_DOMAIN_ID` variable
```bash
  export ROS_DOMAIN_ID=<your_domain_id>
```
To maintain this setting shell sessions, you can add the command to your shell startup script:
```bash
  echo "export ROS_DOMAIN_ID=<your_domain_id>" >> ~/.bashrc
```
