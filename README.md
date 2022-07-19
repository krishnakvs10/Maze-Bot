# CONCLAVE readme

# **Description**

The micro_mouse package contains a differential drive robot, a Maze world with a camera attached on top and contains a python script to compute the shortest path required to finish the maze. 

The shortest path for the maze is calculated already whose NumPy file (array of points leading to the goal pose) is also added in the scripts folder along with the controller script which navigates the robot to complete the maze from the generated array of points.  

# Framework and Simulation Version

ROS melodic 

Gazebo 9

python 3 (3.6 or higher)

# Packages Required

```bash
***pip3** **install scikit-image
pip3** **install opencv-contrib-python
pip3 install python3-numpy
pip3 install libboost-python-dev***
```

# **Build and Launch instructions**

```bash
cd ~/<your_ws>/src
git clone <repo URL>
cd ~/<your_ws>
catkin_make
Roslaunch micro_mouses gazebo.launch
```
