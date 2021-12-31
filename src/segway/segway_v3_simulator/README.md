# segway_v3_simulator
Gazebo ROS packages for the Segway RMP V3 provided by Stanley Innovation. This document roughly outlines the various steps required to setup an RMP V3 platform in Gazebo simulation.

**This document runs through the basic setup required to get Segway V3 simulation setup on a desktop**

## Installation
Until we have released our packages in the ROS distro please follow these instructions for installing from source. The following instructions are valid for Ubuntu 14.04LTS and ROS Indigo. Before proceding please install Ubuntu 14.04LTS.

### Required components
* **Desktop PC with GPU** 
  * This guide was completed on a PC with the following specifications, depending on the PC you are using simulation performance may vary but the as-tested setup will allow for realtime simulation.
    * Intel Core i7-6950X 25M Broadwell-E 10-Core 3.0 GHz LGA 2011-v3 140W BX80671I76950X Desktop Processor
    * X99-E Motherboard
    * 64GB DDR4 3200 quad channel Memory
    * 2x NVIDIA 980Ti GPU with SLI enabled
    * Samsung 950 PRO M.2 512GB NVMe SSD
    * Fully updated fresh install of Ubuntu 14.04.5 LTS Desktop
      *As of 02/18/17

* **Other Required Components**
  * Joystick 
    * 110/220/440: Logitech F310/F710 
    * Flex Omni: Logitech Extreme 3D Pro
    
### Install ROS Indigo
From a linux machine connected to the internet run the following commands

1. **Setup your ROS sources.list**
  * Setup your computer to accept software from packages.ros.org. ROS Indigo ONLY supports Saucy (13.10) and Trusty (14.04) for debian packages.
  ```
  sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
  ```
2. **Set up your ROS keys**
  * Use the following command
  ```
  sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
  ```
3. **ROS Installation**
  * First, make sure your Debian package index is up-to-date:
  ```
  sudo apt-get update
  sudo apt-get install ros-indigo-desktop-full
  ``` 

4. **Initialize rosdep**
  * You must initialize rosdep
  ```
  sudo rosdep init
  rosdep update
  ``` 

5. **Environment setup**
  * Edit the local bash environment to add a few useful aliases
  ```
  gedit ~/.bashrc
  ``` 
  * Add the following lines to the end of the file each provides a few shortcuts:
  ```
  function save_map()
  {
    if [ ! -z "$1" ]
    then
      local dest='/home/**USERNAME**/segway_ws/src/segway_v3/segway_navigation/segway_navigation_apps/maps/'$1
    else
      local dest='/home/**USERNAME**/segway_ws/src/segway_v3/segway_navigation/segway_navigation_apps/maps/mymap'
    fi

    rosrun map_server map_saver -f $dest
  }
  
  source /opt/ros/indigo/setup.bash
  alias sws='source ./devel/setup.bash'
  ``` 

6. **Getting rosinstall**
  * rosinstall is a frequently used command-line tool in ROS that is distributed separately. It enables you to easily download many source trees for ROS packages with one command.
  ```
  sudo apt-get install python-rosinstall
  ```

### Install required packages
From a linux machine connected to the internet run the following commands

1. **Install useful linux utilities**
  * These tools are useful for monitoring system processes, setting up networking, and setting up NTPD for the remote computer. They are not neccessary but recommended.
  ```
  sudo apt-get install iperf chrony htop bridge-utils
  ```
2. **Install required ROS third party packages for segway_v3_robot**
  * These are the packages that RMP V3 depends on
  ```
  sudo apt-get install ros-indigo-navigation ros-indigo-gmapping ros-indigo-robot-localization ros-indigo-yocs-cmd-vel-mux ros-indigo-joy ros-indigo-urg-node ros-indigo-sick-tim ros-indigo-pointgrey-camera-driver ros-indigo-cmake-modules daemontools openssh-server libpcap0.8-dev ros-indigo-um7 ros-indigo-imu-tools ros-indigo-jsk-recognition ros-indigo-ros-controllers ros-indigo-scan-tools ros-indigo-gazebo-ros  ros-indigo-gazebo-plugins ros-indigo-moveit-ros ros-indigo-gazebo-ros-control ros-indigo-hector-gazebo-plugins
  ```

4. **Create a workspace in your home directory**
  * This step is a general method for creating a new workspace and compiling the Segway V3 stack
  ```
  mkdir -p ~/segway_ws/src && cd ~/segway_ws/src
  catkin_init_workspace
  cd ..
  catkin_make
  cd ~/segway_ws/src
  git clone https://github.com/StanleyInnovation/segway_v3.git
  git clone https://github.com/StanleyInnovation/segway_v3_robot.git
  git clone https://github.com/StanleyInnovation/segway_v3_desktop.git
  git clone https://github.com/StanleyInnovation/segway_v3_simulator.git
  ```
  
### Robot Customization
1. **Edit the setup configuration**
  * To setup your robot configuration edit the segway_config.sh file
  ```
  cd ~/segway_ws/src/segway_v3/segway_v3_config
  gedit segway_config.bash
  ```
  * Run through it, it allows you to set all the variables needed to customize your platform
  * By default it is setup for an RMP Flex OMNI with a front and rear laser
  * To change the platform to another model (available models RMP_210, RMP_220, RMP_440LE, RMP_440SE, RMP_OMNI)
    * Edit the variable **SEGWAY_BASE_PLATFORM** (This is the platform the RMP is based on)
    * Edit the variable **SEGWAY_PLATFORM_NAME** (This is the name of the custom robot)
    * **SEGWAY_BASE_PLATFORM** is generally the same as **SEGWAY_PLATFORM_NAME**; unless you create a custom platform or buy one from us
  * For all models except the 210 set **SEGWAY_HAS_BSA** to true
  * If you have a joystic make sure you set it up in this file **SEGWAY_HAS_ONBOARD_JOY**
  * The rest of the variables are fairly straight forward

2. **Compile from source**
  ```
  cd ~/segway_ws
  catkin_make
  ```

# Assuming you followed the instructions up to this point you should have successfully compiled, a little more setup and your on your way
  
## Start the simulation

  Make sure to source with workspace so environment variables that control the Segway V3 stack get pulled in
  ```
  cd ~/segway_ws
  source ./devel/setup.bash
  ```

  To launch a platform with a test environment use:  
  ```
  roslaunch segway_gazebo segway_playpen.launch
  roslaunch segway_viz view_robot.launch
  ```
  NOTE: It may take a few minutes to load the gazebo SDF models for the environment
  
   To launch a platform with an empty environment use:  
  ```
  roslaunch segway_gazebo segway_empty_world.launch
  roslaunch segway_viz view_robot.launch
  ```

# Congratulations!!! If you got here the simulation is all setup!!! You can run any of the demo applications in ~/segway_ws/src/segway_v3/segway_navigation/segway_navigation_apps/launch for mapping and navigation. Remember to add the sim:=true argument.

#You can control the simulated platform with the interactive marker in RVIZ or if you have a joystick by launching the joy node with roslaunch segway_remote_teleop segway_remote_teleop.launch

#To shortcut the whole process just cut and paste the following and make sure you replace **USERNAME** with your username  

```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' &&   sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116 && sudo apt-get update && sudo apt-get install ros-indigo-desktop-full && sudo rm -f /etc/ros/rosdep/sources.list.d/20-default.list && sudo rosdep init && rosdep update && echo "  function save_map()
  {
    if [ ! -z "$1" ]
    then
      local dest='/home/**USERNAME**/segway_ws/src/segway_v3/segway_navigation/segway_navigation_apps/maps/'$1
    else
      local dest='/home/**USERNAME**/segway_ws/src/segway_v3/segway_navigation/segway_navigation_apps/maps/mymap'
    fi

    rosrun map_server map_saver -f $dest
  }
  
  source /opt/ros/indigo/setup.bash
  alias sws='source ./devel/setup.bash'" >> ~/.bashrc && source ~/.bashrc && sudo apt-get install python-rosinstall && sudo apt-get install iperf chrony htop bridge-utils && sudo apt-get install ros-indigo-navigation ros-indigo-gmapping ros-indigo-robot-localization ros-indigo-yocs-cmd-vel-mux ros-indigo-joy ros-indigo-urg-node ros-indigo-sick-tim ros-indigo-pointgrey-camera-driver ros-indigo-cmake-modules daemontools openssh-server libpcap0.8-dev ros-indigo-um7 ros-indigo-imu-tools ros-indigo-jsk-recognition ros-indigo-ros-controllers ros-indigo-scan-tools ros-indigo-gazebo-ros  ros-indigo-gazebo-plugins ros-indigo-moveit-ros ros-indigo-gazebo-ros-control ros-indigo-hector-gazebo-plugins && rm -rf ~/segway_ws && mkdir -p ~/segway_ws/src && cd ~/segway_ws/src && catkin_init_workspace && cd .. && catkin_make && cd ~/segway_ws/src && git clone https://github.com/StanleyInnovation/segway_v3.git && git clone https://github.com/StanleyInnovation/segway_v3_robot.git && git clone https://github.com/StanleyInnovation/segway_v3_desktop.git && git clone https://github.com/StanleyInnovation/segway_v3_simulator.git && cd ~/segway_ws && catkin_make && source ./devel/setup.bash && roslaunch segway_gazebo segway_playpen.launch
```
   


  
  

    
   
     
  
  
  
   

