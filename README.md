## 🤖 Project: Multi-robot
**Course: Robotics Lab**

**Student: Leonardo Riccardi / P38000358**

---

## 🥅 Project Objective

The goal is to simulate an active tracking system. On one side, we have the Fra2mo mobile robot, which executes a logistics task by navigating autonomously through the environment using SLAM and AMCL algorithms. On the other side, we have the KUKA iiwa 7 GDL manipulator acting as an active observer.


## ⛏️Build
Clone this package in the `src` folder of your ROS 2 workspace.
```
git clone https://github.com/leonardoricc2002/Multi-robot.git
```
Build and source the setup files
```
colcon build
```
```
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$(ros2 pkg prefix multi-robot_project)/share/multi-robot_project
```
```
echo $GZ_SIM_RESOURCE_PATH
source install/setup.bash
```

## 🚀 HOW TO LAUNCH
Terminal 1. Launch the Gazebo world.
```
ros2 launch multi-robot_project warehouse.launch.py
```
Terminal 2.Activate the slam
```
source install/setup.bash
ros2 launch multi-robot_projectt slam.launch.py
```
Terminal 3. Activate Navigation 
```
source install/setup.bash
ros2 launch multi-robot_project navigation.launch.py
```
## 🚀Tracking 

Terminal 4. Iiwa follows fra2mo during its path.

```
source install/setup.bash
python3 src/multi-robot_project/src/scripts/tracking.py 
```
Terminal 5. seeing fra2mo with iiwa camera during the trajectory
```
source install/setup.bash
ros2 run rqt_image_view rqt_image_view
```
## 🚗Autonomus Navigation
Terminal 6. With this command fra2mo goes to the target point,
```
source install/setup.bash
python3 src/multi-robot_project/src/scripts/send_goal.py 
```
## Configuration
If you want see only configuration rviz2, after you launch gazebo world.
Terminal 2. real-time configuration 
```
source install/setup.bash
ros2 launch multi-robot_project rviz.launch.py
```
Terminal 1. static configuration
```
ros2 launch multi-robot_project view_robots.launch.py 
```
