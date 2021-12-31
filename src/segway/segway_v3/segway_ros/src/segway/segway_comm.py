"""--------------------------------------------------------------------
COPYRIGHT 2014 Stanley Innovation Inc.

Software License Agreement:

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 
 \file   segway_comm.py

 \brief  runs the driver

 \Platform: Linux/ROS Indigo
--------------------------------------------------------------------"""
from system_defines import *
from utils import *
from segway_msgs.msg import *
from geometry_msgs.msg import Twist
from segway_ros.cfg import segwayConfig
from dynamic_reconfigure.server import Server
from dynamic_reconfigure.client import Client
from dynamic_reconfigure.msg import Config
from io_eth import IoEthThread
from io_usb import IoUsbThread
from segway_data_classes import RMP_DATA
import rosparam
import multiprocessing
import rospy
import math
import select
import threading
import re
import os
"""
Dictionary for all RMP configuration command ID's
"""
command_ids = dict({"GENERAL_PURPOSE_CMD_NONE":                   0,
                    "GENERAL_PURPOSE_CMD_SET_AUDIO_COMMAND":      1,
                    "GENERAL_PURPOSE_CMD_SET_OPERATIONAL_MODE":   2,
                    "GENERAL_PURPOSE_CMD_SEND_SP_FAULTLOG":       3,
                    "GENERAL_PURPOSE_CMD_RESET_INTEGRATORS":      4,
                    "GENERAL_PURPOSE_CMD_RESET_PARAMS_TO_DEFAULT":5})

class SegwayDriver:
    def __init__(self):

        """
        Variables to track communication frequency for debugging
        """
        self.summer=0
        self.samp = 0
        self.avg_freq = 0.0
        self.start_frequency_samp = False
        self.need_to_terminate = False
        self.flush_rcvd_data=True
        self.update_base_local_planner = False
        self.last_move_base_update = rospy.Time.now().to_sec()
        
        """
        Make sure we have a valid platform
        """
        self.platform = rospy.get_param('~platform',None)
        if not (self.platform in SUPPORTED_PLATFORMS):
            rospy.logerr("Platform defined is not supported: %s",self.platform)
            return
        
        """
        The 440 platforms use the same drivers but different URDF
        """
        pattern = re.compile("RMP_440")
        if not (None == pattern.search(self.platform)):
            self.platform = "RMP_440"

        """
        Initialize the publishers for RMP
        """
        self.rmp_data = RMP_DATA()
        
        """
        Initialize faultlog related items
        """
        self.is_init = True
        self.extracting_faultlog = False
        
        """
        Initialize the dynamic reconfigure server for RMP
        """
        self.param_server_initialized = False
        self.dyn_reconfigure_srv = Server(segwayConfig, self._dyn_reconfig_callback)

        """
        Wait for the parameter server to set the configs and then set the IP address from that.
        Note that this must be the current ethernet settings of the platform. If you want to change it
        set the ethernet settings at launch to the current ethernet settings, power up, change them, power down, set the
        the ethernet settings at launch to the new ones and relaunch
        """
        r = rospy.Rate(10)
        start_time = rospy.Time.now().to_sec()
        while ((rospy.Time.now().to_sec() - start_time) < 3.0) and (False == self.param_server_initialized):
            r.sleep()
        
        if (False == self.param_server_initialized):
            rospy.logerr("Parameter server not found, you must pass an initial yaml in the launch! exiting...")
            return            
        
        """
        Create the thread to run RMP communication
        """
        interface = rospy.get_param('~interface','eth')
        self.tx_queue_ = multiprocessing.Queue()
        self.rx_queue_ = multiprocessing.Queue()
        if ('eth' == interface):
            self.comm = IoEthThread((os.environ['SEGWAY_IP_ADDRESS'],int(os.environ['SEGWAY_IP_PORT_NUM'])),
                                    self.tx_queue_,
                                    self.rx_queue_,
                                    max_packet_size=1248)
        elif ('usb' == interface):
            self.comm = IoUsbThread('ttyACM0',
                                    self.tx_queue_,
                                    self.rx_queue_,
                                    max_packet_size=1248)
                                    
        
        if (False == self.comm.link_up):
            rospy.logerr("Could not open socket for RMP...")
            self.comm.close()
            return
        
        """
        Initialize the publishers and subscribers for the node
        """
        self.faultlog_pub = rospy.Publisher('/segway/feedback/faultlog', Faultlog, queue_size=10,latch=True)
        rospy.Subscriber("/segway/cmd_vel", Twist, self._add_motion_command_to_queue)
        rospy.Subscriber("/segway/gp_command",ConfigCmd,self._add_config_command_to_queue)
        rospy.Subscriber("/move_base/TrajectoryPlannerROS/parameter_updates",Config,self._update_move_base_params)
        rospy.Subscriber("/move_base/DWAPlannerROS/parameter_updates",Config,self._update_move_base_params)

        """
        Start the receive handler thread
        """
        self.terminate_mutex = threading.RLock()
        self.last_rsp_rcvd = rospy.Time.now().to_sec()
        self._rcv_thread   = threading.Thread(target = self._run)
        self._rcv_thread.start()
        
        """
        Start streaming continuous data
        """
        rospy.loginfo("Stopping the data stream")
        if (False == self._continuous_data(False)):
            rospy.logerr("Could not stop RMP communication stream")
            self.__del__()
            return
        
        """
        Extract the faultlog at startup
        """
        self.flush_rcvd_data=False
        rospy.loginfo("Extracting the faultlog")
        self.extracting_faultlog = True
        
        if (False == self._extract_faultlog()):
            rospy.logerr("Could not get retrieve RMP faultlog")
            self.__del__()
            return          
        
        """
        Start streaming continuous data
        """
        rospy.loginfo("Starting the data stream")
        if (False == self._continuous_data(True)):
            rospy.logerr("Could not start RMP communication stream")
            self.__del__()
            return
            
        self.start_frequency_samp = True
        
        """
        Ensure that the platform is the right one, we don't want to allow the wrong platform because
        some things depend on knowing which platform is operating
        """
        if (self.platform != PLATFORM_IDS[(self.rmp_data.status.platform_identifier & 0x1F)]):
            rospy.logerr("Platform ID is not correct!!! Platform reports %(1)s, user set %(2)s..."%{"1":PLATFORM_IDS[(self.rmp_data.status.platform_identifier & 0x1F)],"2":self.platform} )
            self.__del__()
            return
        
        """
        Force the configuration to update the first time to ensure that the variables are set to
        the correct values on the machine
        """
        if (False == self._initial_param_force_update()):
            rospy.logerr("Initial configuration parameteters my not be valid, please check them in the yaml file")
            rospy.logerr("The ethernet address must be set to the present address at startup, to change it:")
            rospy.logerr("start the machine; change the address using rqt_reconfigure; shutdown; update the yaml and restart")
            self.__del__()
            return
        
        rospy.loginfo("Segway Driver is up and running")
        
        """
        Indicate the driver is up with motor audio
        """
        cmds = [GENERAL_PURPOSE_CMD_ID,[GENERAL_PURPOSE_CMD_SET_AUDIO_COMMAND,MOTOR_AUDIO_PLAY_EXIT_ALARM_SONG]]
        self._add_command_to_queue(cmds)
    
    def __del__(self):
        rospy.logerr("Segway Driver has called the __del__ method, terminating")
        with self.terminate_mutex:
            self.need_to_terminate = True
        
        assert(self._rcv_thread)
        self._rcv_thread.join()        
    
    def _run(self):
        
        while not rospy.is_shutdown():
            with self.terminate_mutex:
                if (self.need_to_terminate):
                    break
            """
            Run until signaled to stop
            Perform the actions defined based on the flags passed out
            """
            result = select.select([self.rx_queue_._reader],[],[],0.5)
            if len(result[0]) > 0:
                try:
                    data = result[0][0].recv()
                    self._handle_rsp(data)
                except:
                    rospy.logdebug("select did not return interface data")

        self.comm.Close()
        self.tx_queue_.close()
        self.rx_queue_.close()
        
    def _add_command_to_queue(self,command):
        
        """
        Create a byte array with the CRC from the command
        """
        cmd_bytes = generate_cmd_bytes(command)
        
        """
        Send it
        """
        self.tx_queue_.put(cmd_bytes)
        
    def _update_rcv_frq(self):
        if (True == self.start_frequency_samp):
            self.samp+=1
            self.summer+=1.0/(rospy.Time.now().to_sec() - self.last_rsp_rcvd)
            self.avg_freq = self.summer/self.samp
        self.last_rsp_rcvd = rospy.Time.now().to_sec()
                        
    def _handle_rsp(self,data_bytes):
        self._update_rcv_frq()
        if (True == self.flush_rcvd_data):
            return

        valid_data,rsp_data = validate_response(data_bytes)
        if (False == valid_data):
            rospy.logerr("bad RMP packet")
            return

        if (self.extracting_faultlog):
            if (len(rsp_data) == NUMBER_OF_FAULTLOG_WORDS):
                self.extracting_faultlog = False
                faultlog_msg = Faultlog()
                faultlog_msg.data = rsp_data
                self.faultlog_pub.publish(faultlog_msg)
        elif (len(rsp_data) == NUMBER_OF_RMP_RSP_WORDS):
            
            self.rmp_data.status.parse(rsp_data[START_STATUS_BLOCK:END_STATUS_BLOCK])
            self.rmp_data.auxiliary_power.parse(rsp_data[START_AUX_POWER_BLOCK:END_AUX_POWER_BLOCK])
            self.rmp_data.propulsion.parse(rsp_data[START_PROPULSION_POWER_BLOCK:END_PROPULSION_POWER_BLOCK])
            self.rmp_data.dynamics.parse(rsp_data[START_DYNAMICS_BLOCK:END_DYNAMICS_BLOCK])
            self.rmp_data.config_param.parse(rsp_data[START_CONFIG_BLOCK:END_CONFIG_BLOCK])
            self.rmp_data.imu.parse_data(rsp_data[START_IMU_BLOCK:END_IMU_BLOCK])
            
            rospy.logdebug("feedback received from rmp")
        
    def _add_motion_command_to_queue(self,command):
        
        """
        Add the command to the queue, platform does command limiting and mapping
        """
        cmds = [MOTION_CMD_ID,[convert_float_to_u32(command.linear.x),
                               convert_float_to_u32(command.linear.y),
                               convert_float_to_u32(command.angular.z)]]
        self._add_command_to_queue(cmds)
            
    def _add_config_command_to_queue(self,command):
        try:
            cmds = [GENERAL_PURPOSE_CMD_ID,[command_ids[command.gp_cmd],command.gp_param]]
            self._add_command_to_queue(cmds)
        except:
            rospy.logerr("Config param failed, it is probably not known")
            return

    def _dyn_reconfig_callback(self,config,level):

        """
        The first time through we want to ignore the values because they are just defaults from the ROS
        parameter server which has no knowledge of the platform being used
        """     
        if (True == self.is_init):
            self.is_init = False
            return config
    
        """
        Create the configuration bitmap from the appropriate variables
        """
        config_bitmap = (((config.enable_audio^1) << AUDIO_SILENCE_REQUEST_SHIFT)|
                         ((config.motion_while_charging^1) << DISABLE_AC_PRESENT_CSI_SHIFT)|
                         (config.balace_gains << BALANCE_GAIN_SCHEDULE_SHIFT)|
                         (config.balance_mode_enabled << BALANCE_MODE_LOCKOUT_SHIFT) |
                         (config.vel_ctl_input_filter << VEL_CTL_FILTER_SHIFT) |
                         (config.yaw_ctl_input_filter << YAW_CTL_FILTER_SHIFT))
        
        """
        Define the configuration parameters for all the platforms
        """
        self.valid_config_cmd  = [LOAD_MACH_CONFIG_CMD_ID,
                                  [convert_float_to_u32(config.vel_limit_mps),
                                   convert_float_to_u32(config.accel_limit_mps2),
                                   convert_float_to_u32(config.decel_limit_mps2),
                                   convert_float_to_u32(config.dtz_decel_limit_mps2),
                                   convert_float_to_u32(config.yaw_rate_limit_rps),
                                   convert_float_to_u32(config.yaw_accel_limit_rps2),
                                   convert_float_to_u32(config.lateral_accel_limit_mps2),
                                   convert_float_to_u32(config.tire_rolling_diameter_m),
                                   convert_float_to_u32(config.wheel_base_length_m),
                                   convert_float_to_u32(config.wheel_track_width_m),
                                   convert_float_to_u32(config.gear_ratio),
                                   config_bitmap]]
        
        rospy.loginfo("Reconfigure Requested!")
        rospy.loginfo("vel_limit_mps:            %f"%config.vel_limit_mps)
        rospy.loginfo("accel_limit_mps2:         %f"%config.accel_limit_mps2)
        rospy.loginfo("decel_limit_mps2:         %f"%config.decel_limit_mps2)
        rospy.loginfo("dtz_decel_limit_mps2:     %f"%config.dtz_decel_limit_mps2)
        rospy.loginfo("yaw_rate_limit_rps:       %f"%config.yaw_rate_limit_rps)
        rospy.loginfo("yaw_accel_limit_rps2:     %f"%config.yaw_accel_limit_rps2)
        rospy.loginfo("lateral_accel_limit_mps2: %f"%config.lateral_accel_limit_mps2)
        rospy.loginfo("tire_rolling_diameter_m:  %f"%config.tire_rolling_diameter_m)
        rospy.loginfo("wheel_base_length_m:      %f"%config.wheel_base_length_m)
        rospy.loginfo("wheel_track_width_m:      %f"%config.wheel_track_width_m)
        rospy.loginfo("gear_ratio:               %f"%config.gear_ratio)
        rospy.loginfo("enable_audio:             %u"%config.enable_audio)
        rospy.loginfo("motion_while_charging:    %u"%config.motion_while_charging)
        rospy.loginfo("balance_mode_enabled:     %u"%config.balance_mode_enabled)
        rospy.loginfo("balance_gain_schedule:    %u"%config.balace_gains)
        rospy.loginfo("vel_ctl_input_filter:     %u"%config.vel_ctl_input_filter)
        rospy.loginfo("yaw_ctl_input_filter:     %u"%config.yaw_ctl_input_filter)
             
        """
        The teleop limits are always the minimum of the actual machine limit and the ones set for teleop
        """
        config.teleop_vel_limit_mps = minimum_f(config.teleop_vel_limit_mps, config.vel_limit_mps)
        config.teleop_accel_limit_mps2 = minimum_f(config.teleop_accel_limit_mps2, config.accel_limit_mps2)
        config.teleop_yaw_rate_limit_rps = minimum_f(config.teleop_yaw_rate_limit_rps, config.yaw_rate_limit_rps)
        config.teleop_yaw_accel_limit_rps2 = minimum_f(config.teleop_yaw_accel_limit_rps2, config.teleop_yaw_accel_limit_rps2)      
        
        """
        Set the teleop configuration in the feedback
        """
        self.rmp_data.config_param.SetTeleopConfig([config.teleop_vel_limit_mps,
                                                    config.teleop_accel_limit_mps2,
                                                    config.teleop_yaw_rate_limit_rps,
                                                    config.teleop_yaw_accel_limit_rps2]) 
        
        """
        Check and see if we need to store the parameters in NVM before we try, although the NVM is F-RAM
        with unlimited read/write, uneccessarily setting the parameters only introduces risk for error 
        """
        if self.param_server_initialized:
            load_params = False
            for i in range(NUMBER_OF_CONFIG_PARAM_VARIABLES-4):
                if (self.rmp_data.config_param.configuration_feedback[i] != self.valid_config_cmd[1][i]):
                    load_params = True
            if (True == load_params):
                self._add_command_to_queue(self.valid_config_cmd)
                rospy.loginfo("Sent config update command")
            
            """
            Just update the peak torque as it is not a persistant command
            """
            
            if ((1<<17) == ((1<<17)&level)):
                rospy.loginfo("level is %u"%level)
                cmd = [GENERAL_PURPOSE_CMD_ID,
                       [GENERAL_PURPOSE_CMD_SET_TORQUE_LIMIT,
                        convert_float_to_u32(config.torqe_limit/100.0)]]
                self._add_command_to_queue(cmd)
        
        self.param_server_initialized = True
        self.valid_config = config
        self.update_base_local_planner = True
        self._update_move_base_params(None)
        return config
    
    def _update_move_base_params(self,config):
        
        """
        If parameter updates have not been called in the last 5 seconds allow the
        subscriber callback to set them
        """
        if ((rospy.Time.now().to_sec()-self.last_move_base_update) > 5.0):
            self.update_base_local_planner = True
            
        if self.update_base_local_planner:
            self.update_base_local_planner = False
            self.last_move_base_update = rospy.Time.now().to_sec()
            
            try:
                dyn_reconfigure_client= Client("/move_base/TrajectoryPlannerROS",timeout=1.0)
                changes = dict()
                changes['acc_lim_x'] = minimum_f(self.valid_config.accel_limit_mps2, self.valid_config.decel_limit_mps2)
                changes['acc_lim_theta'] = self.valid_config.yaw_accel_limit_rps2
                changes['max_vel_x'] = self.valid_config.vel_limit_mps
                changes['max_vel_theta'] = self.valid_config.yaw_rate_limit_rps
                changes['min_vel_theta'] = -self.valid_config.yaw_rate_limit_rps
                dyn_reconfigure_client.update_configuration(changes)
                dyn_reconfigure_client.close()
            except:
                pass
    
            try:
                dyn_reconfigure_client= Client("/move_base/DWAPlannerROS",timeout=1.0)
                changes = dict()
                changes['acc_lim_x'] = minimum_f(self.valid_config.accel_limit_mps2, self.valid_config.decel_limit_mps2)
                changes['acc_lim_th'] = self.valid_config.yaw_accel_limit_rps2
                changes['max_vel_x'] = self.valid_config.vel_limit_mps
                changes['max_rot_vel'] = self.valid_config.yaw_rate_limit_rps
                dyn_reconfigure_client.update_configuration(changes)
                dyn_reconfigure_client.close()
            except:
                pass
            
            
            rospy.loginfo("Segway Driver updated move_base parameters to match machine parameters")   

    def _continuous_data(self,start_cont):
        set_continuous = [GENERAL_PURPOSE_CMD_ID,[GENERAL_PURPOSE_CMD_SEND_CONTINUOUS_DATA,start_cont]]
        ret = False
        
        if (True == start_cont):
            r = rospy.Rate(10)
            start_time = rospy.Time.now().to_sec()
            while ((rospy.Time.now().to_sec() - start_time) < 3.0) and (True == self.rmp_data.status.init):
                self._add_command_to_queue(set_continuous)
                r.sleep()
            ret = not self.rmp_data.status.init
        else:
            r = rospy.Rate(5)
            start_time = rospy.Time.now().to_sec()
            while ((rospy.Time.now().to_sec() - start_time) < 3.0) and (False == ret):
                self._add_command_to_queue(set_continuous)
                if ((rospy.Time.now().to_sec() - self.last_rsp_rcvd) > 0.1):
                    ret = True
                r.sleep()
            self.rmp_data.status.init = True

        return ret
    
    def _extract_faultlog(self):
        r = rospy.Rate(2)        
        start_time = rospy.Time.now().to_sec()
        while ((rospy.Time.now().to_sec() - start_time) < 3.0) and (True == self.extracting_faultlog):
            self._add_command_to_queue([GENERAL_PURPOSE_CMD_ID,[GENERAL_PURPOSE_CMD_SEND_SP_FAULTLOG,0]]) 
            r.sleep()
            
        return not self.extracting_faultlog
    
    def _initial_param_force_update(self):
        """
        Load all the parameters on the machine at startup; first check if they match, if they do continue.
        Otherwise load them and check again.
        """
        r = rospy.Rate(2)
        start_time = rospy.Time.now().to_sec()
        load_params = True
        while ((rospy.Time.now().to_sec() - start_time) < 3.0) and (True == load_params):
            load_params = False
            for i in range(NUMBER_OF_CONFIG_PARAM_VARIABLES-4):
                if (self.rmp_data.config_param.configuration_feedback[i] != self.valid_config_cmd[1][i]):
                    load_params = True
            if (True == load_params):
                self._add_command_to_queue(self.valid_config_cmd)
                r.sleep()
        
        return not load_params
    
