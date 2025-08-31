# Giraffe-robot
This is the final project for the "Introduction to Robotics" course of Unitn. It focuses on the design and control of a giraffe robot to automate microphone handling during Q&A sessions at talks in conference rooms. All the details can be found in the `giraffe_robot.pdf` file.

## How to run the code
To initialize the ROS environment:
- Ensure to have a ROS distribution (here used Noetic)
- Clone the repository
- Compile the workspace with catkin
```
cd Giraffe-robot/src
catkin init
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin build
```

- Source the environment
```
source ../devel/setup.bash
```
- Run the code
```
rosrun main.py
```