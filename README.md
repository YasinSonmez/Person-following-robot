# Person Following, Dynamic Obstacle Avoiding Autonomous Robot
The goal of this project is to develop a robot that is able to follow a human by avoiding static and dynamic obstacles by generating socially aware trajectories that don’t interfere with the paths of dynamic objects in the scene.  The robot possesses computers and various sensors such as cameras and laser range sensors onboard so that it can calculate everything on the go.  The aim of the robot is to help humans in various scenarios by following them and aiding them.




https://user-images.githubusercontent.com/37510173/151413131-d7237742-6661-4421-b778-63c1afeced60.mp4


## Dependencies
1. ROS (Melodic or Noetic preferred)
2. Darknet ROS (CUDA, OpenCV, boost)
3. ROS Navigation Stack
4. TEB Local Planner

## 1. Quick Start

The project has been tested on Ubuntu 16.04 (ROS Kinetic) and 18.04 (ROS Melodic). We highly recommend using Ubuntu 18.04 since Ubuntu 16.04 will no longer be supported after April 2021. In the following we will take ROS Melodic version as the example. The navigation simulation is performed by the powerful ROS navigation stack, wherein two local planners of DWA and TEB are tested. Therefore, please install these packages first:

```
sudo apt install ros-melodic-navigation ros-melodic-teb-local-planner ros-melodic-arbotix-python
```
The distance to the closest obstacle is computed by performing bicubic interpolation on top of the Euclidean distance grid (EDG). EDG is constructed by an efficient distance transform algorithm implemented in OpenCV, and bicubic interpolation is implemented in [Google's Ceres solver](http://ceres-solver.org/). Therefore, please install Ceres solver following the official [installation tutorial](http://ceres-solver.org/installation.html). 

After the above preparation, please create and initialize a ROS workspace. We assume that your workspace is named catkin_ ws. Then, run the following commands to clone this repo and build it:

```
cd ~/catkin_ws
git clone https://github.com/YasinSonmez/Person-following-robot.git
cd src
cd ../
catkin_make
```

Finally, open a new terminal and start a simulation: 
```
source ~/catkin_ws/devel/setup.bash
roslaunch move_base_benchmark move_base_benchmark.launch
```
Open another terminal and send the goal for the robot:
```
source ~/catkin_ws/devel/setup.bash
roslaunch move_base_benchmark simple_navigation_goals.launch
```
You can also select goals for the robot using the ```2D Nav Goal``` tool in ```RViz```.

## 2. Setup
### 2.1 Darknet ROS
Follow the installation instructions [**here**](https://github.com/leggedrobotics/darknet_ros) to install Darknet ROS. Install it inside the workspace.
### 2.2 Gazebo-Map-Actor-Plugin
Follow the steps from [Gazebo-Map-Actor-Plugin](https://github.com/YasinSonmez/Gazebo-Map-Actor-Plugin) to compile the gazebo plugin for the actors and see how to use the actors and actor position publisher node. You don't need to download the code again, plugin codes are already in this repository, all you need to do is to build the plugin following the steps there.
### 2.3 Actor Collisions Plugin
We use [**Actor Collisions Plugin**](https://github.com/osrf/gazebo/tree/gazebo11/examples/plugins/actor_collisions) to give dynamic pedestrians collision properties, so that they can be swept by the laser rangefinder. From the actor_collisions directory
```
mkdir build
cd build
cmake ../
make
```
After that, a library named "libActorCollisionsPlugin.so" will be generated in the build directory. Please update the reference path of "libActorCollisionsPlugin.so" in the xxx_dynamic.world files in the gazebo_world/world directory before you use the dynamic world models. For example, open office02_dynamic.world and use "ctrl+F" to find "libActorCollisionsPlugin.so". Then, replace the value of "filename" with the absolute path of "libActorCollisionsPlugin.so" in your build directory of actor_collisions. Each animated actor needs to call this plugin. Therefore, please check all the reference paths of this plugin in the dynamic world models.

## 3. Running the Perception, Planning, and Control Parts
1. Source the robot config file:
```
source src/segway/segway_v3/segway_v3_config/std_configs/segway_config_RMP_220.bash
```
2. Launch any launch file from local-planning-benchmark/move_base_benchmark/launch directory:
```
roslaunch move_base_benchmark move_base_benchmark_small_house_map_segway.launch
```
3. Run the actor pos publisher node (if not run inside the .launch file): 
```
rosrun actor_pos_publish actor_pos_publish_node
```
4. Run the human detection node:
```
roslaunch yolo_pp yolo_v3.launch
```
5. Run the package that controls the head of the robot to focus on human:
```
rosrun widowx_turret_controller pt_cmd
```
7. Run the target generation node:
```
rosrun target_generation target_generation_node
```
## TODO
- [ ] Synchronization of bounding box and depth messages(need tuning)
- [ ] Localization using any method (currently odometry is taken from gazebo)
- [ ] Memorization of human positions when lost from view
- [ ] Collision objects for humans
- [ ] Sensor fusion for laser and camera to better estimate the human positions

## References
1. https://github.com/bach05/gazebo-plugin-autonomous-actor
2. https://github.com/NKU-MobFly-Robotics/local-planning-benchmark
3. https://github.com/NKU-MobFly-Robotics/p3dx
4. https://github.com/leggedrobotics/darknet_ros
5. https://github.com/YasinSonmez/Gazebo-Map-Actor-Plugin
