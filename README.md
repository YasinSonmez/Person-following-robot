# Person Following, Dynamic Obstacle Avoiding Autonomous Robot
The goal of this project is to develop a robot that is able to follow a human by avoiding static anddynamic obstacles by generating socially aware trajectories that don’t interfere with the paths ofdynamic objects in the scene.  The robot possesses computers and various sensors such as camerasand laser range sensors onboard so that it can calculate everything on the go.  The aim of the robotis to help humans in various scenarios by following them and aiding them.

## 1. Quick Start

The project has been tested on Ubuntu 16.04 (ROS Kinetic) and 18.04 (ROS Melodic). We highly recommend using Ubuntu 18.04 since Ubuntu 16.04 will no longer be supported after April 2021. In the following we will take ROS Melodic version as the example. The navigation simulation is performed by the powerful ROS navigation stack, wherein two local planners of DWA and TEB are tested. Therefore, please install these packages first:

```
$ sudo apt install ros-melodic-navigation ros-melodic-teb-local-planner
```
The distance to the closest obstacle is computed by performing bicubic interpolation on top of the Euclidean distance grid (EDG). EDG is constructed by an efficient distance transform algorithm implemented in OpenCV, and bicubic interpolation is implemented in [Google's Ceres solver](http://ceres-solver.org/). Therefore, please install Ceres solver following the official [installation tutorial](http://ceres-solver.org/installation.html). 

After the above preparation, please create and initialize a ROS workspace. We assume that your workspace is named catkin_ ws. Then, run the following commands to clone this repo and build it:

```
$ cd ~/catkin_ws
$ git clone https://github.com/YasinSonmez/Person-following-robot.git
$ cd src
$ git clone https://github.com/NKU-MobFly-Robotics/p3dx.git
$ cd ../
$ catkin_make
```

Finally, open a new terminal and start a simulation: 
```
$ source ~/catkin_ws/devel/setup.bash
$ roslaunch move_base_benchmark move_base_benchmark.launch
```
Open another terminal and send the goal for the robot:
```
$ source ~/catkin_ws/devel/setup.bash
$ roslaunch move_base_benchmark simple_navigation_goals.launch
```
You can also select goals for the robot using the ```2D Nav Goal``` tool in ```RViz```.

## 2. Setup

We use [**Actor Collisions Plugin**](https://github.com/osrf/gazebo/tree/gazebo11/examples/plugins/actor_collisions) to give dynamic pedestrians collision properties, so that they can be swept by the laser rangefinder. From the actor_collisions directory
```
$ mkdir build
$ cd build
$ cmake ../
$ make
```
After that, a library named "libActorCollisionsPlugin.so" will be generated in the build directory. Please update the reference path of "libActorCollisionsPlugin.so" in the xxx_dynamic.world files in the gazebo_world/world directory before you use the dynamic world models. For example, open office02_dynamic.world and use "ctrl+F" to find "libActorCollisionsPlugin.so". Then, replace the value of "filename" with the absolute path of "libActorCollisionsPlugin.so" in your build directory of actor_collisions. Each animated actor needs to call this plugin. Therefore, please check all the reference paths of this plugin in the dynamic world models.

## 3. Using Target Generation Node
2. Run roscore:
```
roscore
```
2. Launch any launch file from local-planning-benchmark/move_base_benchmark/launch directory:
```
roslaunch move_base_benchmark move_base_benchmark_office02_dynamic.launch
```
3. Run the target generation node:
```
rosrun target_generation target_generation_node
```

# Instructions if above instructions doesn't work
1. Instructions here may be missing. In that case do a clean install of MRPB benchmark by following the instructions [here](https://github.com/NKU-MobFly-Robotics/local-planning-benchmark)
2. Then download and add the target generation folder inside the ~/catkin_ws/src folder
