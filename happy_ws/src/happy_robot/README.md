# Happy Robot Autonomous Driving Project


## Software Environment

```
Linux Ubuntu 20.04 LTS
ROS Noetic
Gazebo 11
Python 3.9
gcc / g++ 9
FastAPI
```

## Compile

You can use the following commands to download and compile the package.

```
cd ~/happy_ws/src
git clone repo
cd ..
catkin build 
```

## Run

```
roslaunch happy_robo parking.launch
roslaunch happy_robo execute.launch
```

## Teleop

1. Install teleop package: `sudo apt-get install ros-kinetic-teleop-twist-keyboard`
2. Run: `rosrun teleop_twist_keyboard teleop_twist_keyboard.py`

## Gmapping

1. Install: `sudo apt-get install ros-kinetic-slam-gmapping`
2. Run: `roslaunch jackal_velodyne gmapping_demo.launch`
3. Set the `2D Nav Goal` in Rviz
