# chip_bldc_driver
Chip Robotics BLDC ROS Driver 

To get started make sure you have ros serial installed
e.g. for melodic
sudo apt-get install ros-melodic-serial

create a directory to install the driver and cd into the director
mkdir src 
cd src
catkin_init_workspace
git clone https://github.com/chiprobotics/chip_bldc_driver.git
cd ..
catkin_make
source ./devel/setup.bash
roslaunch chip_bldc_driver example.launch

make sure you have the right permission set on /dev/ttyUSB0 (for a quick hack just use sudo chmod 777 /dev/ttyUSB0)
instruction on using udev will be coming soon. 

For a quick test open a new terminal 
cd into the folder and run source ./devel/setup.bash 
then run rostopic pub -r 10 /bldc_driver_node/Command chip_bldc_driver/Command "motor_command: 100" to see the motor spin. 
It is recommended you first setup the motor with the windows or linux software before running ROS to control the motor. 