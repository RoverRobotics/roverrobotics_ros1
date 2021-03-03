# Roverrobotics_ros1
## About:
- This is a ROS wrapper to interface with roverrobotics' robots
- Librover is required in order to use this wrapper

## Installation instructions

1. Cloning this repository into your workspace
```
cd workspace/src/
git clone https://github.com/RoverRobotics/roverrobotics_ros1
```
2. Install shared library
``` 
cd ~/
mkdir library/
cd library/
git clone https://github.com/RoverRobotics/librover
cd librover/
cmake .
make
sudo make install 
```
2. Rebuild your workspace
```
cd workspace/
catkin_make
```
3. Update auto complete
```
source devels/setup.bash
```
4. Launch Robot (replace <launch file name> with your robot config.)
```
roslaunch roverrobotics_driver pro.launch
```
