# segway_v3_robot
Onboard PC ROS packages for the Segway RMP V3 provided by Stanley Innovation. This document roughly outlines the various steps required to setup an RMP V3 platform that has not been provided by Stanley Innovation.

**This document runs through the basic setup required to get a Segway RMP V3 system running**

**If you have purchased a Navigator or Navigator Elite package, this is already installed and setup with all sorts of extras, so no need to worry about going through this installation guide**

**For all you brave DIYers that decided they didn't need Stanley Innovation's expertise, and went with the barebones V3 system......ENJOY!!!**

**If you haven't decided between barebones and a Navigator Package; go through this and the segway_v3_desktop setup before you decide......How valuable is your time??? I bet the development your working on is much more valuable than spending a bunch of time mucking around with setup......GO WITH A NAVIGATOR PACKAGE!!! That's the pitch, we'll leave the decision up to you**

# We provide fully integrated systems with support
**We provide a Navigator package, a Navigator Elite package and fully integrated custom solutions with all robot setup, networking, timing, sensor/peripheral integration, calibration, tailored navigation tuning, extended functionality, and remote desktop VM. Our integrated packages come with fully setup onboard PC (for robot control) and a VM for remote monitoring and control. This tutorial is for seasoned ROS integrators that can complete that work themselves with our base RMP V3 platforms. Please contact Stanley Innovation for pricing and information on fully integrated packages and base platforms http://stanleyinnovation.com/contact-us/. Stanley Innovation is the _ONLY_ supplier of Segway RMP V3 compatible hardware! Please do not expect any of this to work if you did not purchase the system or an upgrade from Stanley Innovation, Inc.**

**If you have a Segway RMP from another source it can be upgraded. Do not try and load the V3 software without an upgrade performed by Stanley Innovation!!!!! It will render the machine inoperable, even if you try and go back to the standard RMP Release. If you do this you will likely have no option other than buying the upgrade to get the machine running again. You've been warned here and all over the place....don't do it.....**

**_Make_ _your_ _intern_ _do_ _it_, and then blame him when you tell your boss you have to upgrade your system ;)**

**If you want one-on-one engineering support for a system with onboard PC, remote desktop VM, sensors, nav, etc...please buy a Navigator package from us or contact us for an engineering support quote. Otherwise if you plan to buy your own sensors and your own computer for integration it is assumed you know what you're doing. Please use the community for support in integrating your own hardware, we will only address RMP specific questions for these customers if contacted directly. For example if you buy a barebones RMP V3 mobility platform and you need to know how to pull a faultlog, feel free to ask us; if you are trying to setup your PC and all your sensors, please rely on the community. We may help as part of the community for non-platform related questions, but there is no gaurantee**
  
## Mechanical and Electrical Integration of Sensors, Manipulators and other peripherals
* Before starting software setup make sure you have all your sensors mounted and electrically integrated
* You will need their location relative to the **SEGWAY_PARENT_LINK** defined in the configuration (see below in Robot Customization)
* Sensors need to be configured to work with the network and serial configurations we use by default or you need to modify them.
* For questions about electrical integration into the RMP power system see the manual and ask us if you have questions
* For mechanical 3D models please visit our website http://stanleyinnovation.com/resources/ at the bottom of the page

## Software Integration of Sensors, Manipulators and other peripherals
* We provide a subset of the available peripherals we support in the open software
* We can integrate pretty much anything in a custom integration system and support that
* Please use the community for software support unless you purchased a fully integrated system 
  * **If you did purchase a Navigator package or a custom robot; no need to read further unless your just interested. We did all this a quite a bit more setting up your system already.**

## Installation
Until we have released our packages in the ROS distro please follow these instructions for installing from source. The following instructions are valid for Ubuntu 14.04LTS and ROS Indigo. Before proceding please install Ubuntu 14.04LTS.

### Required components
* **Segway RMP V3 platform provided by Stanley Innovation** 
  * For available platforms (http://stanleyinnovation.com/products-services/robotics/robotic-mobility-platforms/)
  * For upgrading existing RMP's (http://stanleyinnovation.com/contact-us/) 
  * Updated with the latest firware (https://github.com/StanleyInnovation/segway_v3_embedded_firmware)
* **PC or VM Running Ubuntu 14.04 LTS**
  * If using onboard PC powered by RMP, make sure it runs on one of the voltages available on the Aux Power
    * Standard Aux Power includes 12VDC@150W
    * 48VDC, 24VDC, and 5VDC optional supplies available 
  * Minimum 8GB RAM (16 GB preferably)
  * Preferably a reasonably fast SSD
  * Preferably 2 Gigabit NICS 
    * You don't need 2 but it is better to have one dedicated to ROS and one to hardware)
  * Atleast 4 USB 2.0 or higher ports 
    * Only if using IMU, GPS and joystick
  * Atleast 1 USB 3.0 port 
    * Only if using PGR Flea3 USB3 camera
  * Some sort of graphics for setup
  * Wireless router (preferably AC Dual-Band)
    * We provide the WRT-AC1900 with our integrated systems
* **Components for Setup**
  * Monitor
  * Keyboard (and mouse probably)
  * Internet Connection
  * Power supply for PC


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
  wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | sudo apt-key add -
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
      local dest='~/segway_ws/src/segway_v3/segway_navigation/segway_navigation_apps/maps/'$1
    else
      local dest='~/segway_ws/src/segway_v3/segway_navigation/segway_navigation_apps/maps/mymap'
    fi

    rosrun map_server map_saver -f $dest
  }
  
  source ~/segway_ws/devel/setup.bash
  source /opt/ros/indigo/setup.bash
  alias sws='source ./devel/setup.bash'
  alias clean_backups='find ./ -name '*~' | xargs rm'
  alias clean_pyc='find ./ -name '*.pyc' | xargs rm'
  alias clean_rosbuild='rm -rf build devel install'
  alias segstop='sudo service segway-core stop'
  alias segstart='sudo service segway-core start'
  alias segchk='sudo tail /var/log/upstart/segway-core.log -n 30'
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
  sudo apt-get install ros-indigo-navigation ros-indigo-gmapping ros-indigo-robot-localization ros-indigo-yocs-cmd-vel-mux ros-indigo-joy ros-indigo-urg-node ros-indigo-lms1xx ros-indigo-pointgrey-camera-driver ros-indigo-cmake-modules daemontools openssh-server libpcap0.8-dev ros-indigo-um7 ros-indigo-imu-tools
  ```
3. **Add yourself to the dialout group** 
  * This is necessary if you have serial, or serial-USB devices
  ```
  sudo adduser **_USERNAME_** dialout
  ```
4. **Create a workspace in your home directory**
  * You only need segway_v3_desktop if you want to visualize on this machine
  ```
  mkdir -p ~/segway_ws/src
  cd ~/segway_ws/src
  catkin_init_workspace
  cd ..
  catkin_make
  cd ~/segway_ws/src
  git clone https://github.com/StanleyInnovation/segway_v3.git
  git clone https://github.com/StanleyInnovation/segway_v3_robot.git
  git clone https://github.com/StanleyInnovation/segway_v3_desktop.git
  ```
  
### Robot Customization
1. **Edit the setup configuration**
  * To setup your robot configuration edit the segway_config.sh file
  ```
  cd ~/segway_ws/src/segway_v3/segway_v3_config
  gedit segway_config.sh
  ```
  * Run through it, it allows you to set all the variables needed to customize your platform
  * By default it is setup for an RMP 210 V3 with no sensors
  * Make sure to set **SEGWAY_POWERS_PC_ONBOARD** appropriately
    * Setting it true will make sure the PC shutsdown before RMP power is removed
    * Setting it false means that the PC will be powered from something other than the RMP (like a laptop battery)
  * Make sure **ROBOT_NETWORK** is set to the physical port ROS will communicate to the outside world on
    * See the next section for details on network configuration
  * To change the platform to another model (available models RMP_210, RMP_220, RMP_440LE, RMP_440SE, RMP_OMNI)
    * Edit the variable **SEGWAY_BASE_PLATFORM** (This is the platform the RMP is based on)
    * Edit the variable **SEGWAY_PLATFORM_NAME** (This is the name of the custom robot)
    * **SEGWAY_BASE_PLATFORM** is generally the same as **SEGWAY_PLATFORM_NAME**; unless you create a custom platform or buy one from us
  * For all models except the 210 set **SEGWAY_HAS_BSA** to true
  * **SEGWAY_RUNS_IN_BALANCE_MODE** should only be set for the 220 if you want to run in Balance mode
    * **WARNING!! Do not run navigation in balance mode unless you have read the manual and fully understand the caveats of balance mode**
  * The rest of the variables are fairly straight forward

2. **Compile from source**
  * **Once you compile from source your configuration file that gets sourced is located in ~/segway_ws/devel/etc/catkin/profile.d/50.segway_config.sh**
  ```
  cd ~/segway_ws
  catkin_make
  ```

# Assuming you followed the instructions up to this point you should have successfully compiled, a little more setup and your on your way

# Don't you wish you sprung for that fully integrated system???

### Setup Network
You need to set the network up for our platforms and the various ethernet enabled sensors.
This is an outline but **we also provide fully integrated packages**.

**NOTE: The ROBOT_NETWORK environment variable must match the port you use for #1 below.**
**You can pull the network off with 1 NIC with some modifications.** 
**We suggest having 2 NICS and following this configuration**

1. **Set the IP of eth1 to the robot and sensor network**
  * The default IP of the network interface that talks to the platform is **10.66.171.4**
  * You can set this IP static by editing /etc/network/interfaces or using the network manager
  ```
  sudo gedit /etc/network/interfaces
  ```
  * Then add these lines assuming eth1 is connected to all the sensors using ethernet and the platform 
  ```
  auto eth1
  iface eth1 inet static
  address 10.66.171.4
  netmask 255.255.255.0
  ```
  * Then restart the interface
  ```
  sudo ifdown eth1
  sudo ifup eth1
  ```
2. **Set the IP of eth0 to the ROS Network**
  * This is the interface that any remote machines will talk to
  * It should be the one connected to your internal network via wireless bridge or its own network via wireless AP
  * For simplicity we will assume you have a wireless AP connected to eth0 with the following configurations
    * IP address 10.66.172.1
    * Subnet Mask: 255.255.255.0
    * Gateway: 10.66.172.1
    * DHCP enabled with range 100-150
  * Configure eth0 there really is no default but it should be something outside the DHCP range we use **10.66.172.4**
  ```
  sudo gedit /etc/network/interfaces
  ```
  * Then add these lines assuming eth1 is connected to all the sensors using ethernet and the platform 
  ```
  auto eth0
  iface eth0 inet static
  address 10.66.172.4
  netmask 255.255.255.0
  ```
  * **Note** that if you want to be able to connect to the internet you need to set the connection up to bridge to your internal wireless network, we will not cover that here
  * Then restart the interface
  ```
  sudo ifdown eth0
  sudo ifup eth0
  ```
4. **Make sure you setup the networking for all the sensors**
  * Default sensor addresses are defined in 50.segway_config.sh see above 
  * **Fully integrated machines delivered by Stanley come with this all setup**
3. **Setup chrony**
  * If you are going to be running ROS nodes on a remote computer it is a good idea to setup chrony to synchronize time between the machines
  * The onboard robot PC should ideally run the server
  * There is information on how to do this out there we will not cover it here
  * **Fully integrated machines delivered by Stanley come with this all setup**
  
### Additional steps
You should probably do the regardless, but these steps are really only required PGR Flea3 camera and onboard PC powered from RMP.

1. **Open the grub configuration**
  * Open a terminal
  ```
  sudo gedit /etc/default/grub
  ```
2. **Add this line**
  * Configures the system to not wait at startup if boot fails
  ```
  # disable getting stuck in menu after fail
  GRUB_RECORDFAIL_TIMEOUT=0
  ```
3. **Update the USB memory buffer to handle USB3**
  * Locate this line
  ```
  GRUB_CMDLINE_LINUX_DEFAULT="quiet splash"
  ```
  * Change it to this
  ```
  GRUB_CMDLINE_LINUX_DEFAULT="quiet splash usbcore usbfs_memory_mb=1000"
  ```
4. **Save and exit**
5. **Update GRUB**
  * From the terminal
  ```
  sudo update-grub
  sudo modprobe usbcore usbfs_memory_mb=1000
  ```
6. **Restart the computer**

### Almost Done!
**You are so close! Only a few extra steps...**

**NOTE: Power the PC from an external power supply until you have finished testing**

1. **If you have followed the instructions and set everything up correctly you should be able to launch the system manually**
  * Ensure the Disable button is not pressed
  * Power the RMP on with the silver button
    * LED Ring should pulse blue
    * The Power LED will blink green
    * The status LED should blink yellow indicating the state
  * In a new terminal
  ```
  cd ~/segway_ws
  sws
  roslaunch segway_bringup stanley_innovation_system.launch
  ```
  * You should hear 2 beeps when the configuration server is initialized and 2 more when the platform is ready to accept commands
  * The launch is time staged so it takes ~15 seconds to complete
  * If you are having issues communicating with the platform
    * you may have something wrong with your network configuration
    * you may have set the wrong platform in the configuration
    * you may have the wrong embedded firmware
    * the platform may not be on
    * the kill switch is pressed
  * If you are getting errors on sensors
    * you may have the wrong ones or they may not be setup correctly
2. **Manual launch works, time to install the service**
  * Kill any ROS nodes that may be running <Ctrl-C> and close all terminals
  * Open a new terminal and install the upstart service
  ```
  cd ~/segway_ws
  sws
  rosrun segway_bringup install_segway_core
  ```
3. **Start the service and make sure it starts up fine**
  * In the terminal enter
  ```
  segstart
  ```
  * Make sure everything starts fine you can repeatedly enter the following until the launch is finished
  ```
  segchk
  ```
  * You should hear 2 beeps when the configuration server is initialized and 2 more when the platform is ready to accept commands
  * The launch is time staged so it takes ~15 seconds to complete
4. **Save anything you might have open**
5. **Power off the RMP**
  * Press the silver power button
    * You should hear the platform play the shutdown song
    * The status LED will turn solid red
    * The blue LED ring on the power button will pulse quickly
  * The PC should enter shutdown if you have configured it to run on RMP power
  * Wait for the system to turn off.
    * All LED will turn off after 30 sec
7. **Connect the PC power input to RMP power output if you are powering the onboard PC from the RMP**
8. **Power on the system**
  * Power the RMP on with the silver button
    * LED Ring should pulse blue
    * The Power LED will blink green
    * The status LED should blink yellow indicating the state
  * If the PC is powered by the RMP, everything should start up when the platform is powered on
    * You should hear 2 beeps when the configuration server is initialized and 2 more when the platform is ready to accept commands
    * The launch is time staged so it takes ~15 seconds to complete
  * If the PC has its own power source power it on after the embedded system comes up

If you want to stop the service and restart it, open a terminal:
```
segstop
segstart
```

# Congratulation!!! If you got here the robot is all setup!!! Now you just need to configure a remote PC for visualization and control.....
# Don't you wish you sprung for that fully integrated system???
 
   


  
  

    
   
     
  
  
  
   

