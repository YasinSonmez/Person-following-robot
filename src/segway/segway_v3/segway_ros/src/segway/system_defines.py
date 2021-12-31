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
 
 \file   system_defines.py

 \brief  This module defines the interface for the RMP

 \Platform: Linux/ROS Indigo
--------------------------------------------------------------------"""

"""
list the platforms supported by this driver
"""
SUPPORTED_PLATFORMS = ["RMP_220","RMP_210","RMP_440","RMP_440LE","RMP_440SE","RMP_OMNI","RMP_110"]
PLATFORM_IDS = dict({
                     0x1:"RMP_440",
                     0x2:"RMP_220"  ,
                     0x5:"RMP_210"  ,
                     0x6:"RMP_OMNI",
                     0x8:"RMP_110"})


"""------------------------------------------------------------------------
RMP Command structures
There are four types of messages the RMP will accept:
1. Non-holonomic motion command message: This message contains two 32-bit IEEE754 floating 
point variables. The first is the velocity command in m/s (range: -8.0...8.0).
The second is the yaw rate command (range: -4.5...4.5). Note that the command range is the
maximum, each will be limited on the platform by the user configurable limits stored in NVM. 

2. Holonomic motion command message: This message contains three 32-bit IEEE754 floating 
point variables. The first is the velocity command in m/s (range: -8.0...8.0).
The second is the yaw rate command (range: -4.5...4.5). The third is the yaw rate 
command (range: 0...2*pi) where 0 and 2pi point foward. Note that the command range is 
the maximum, each will be limited on the platform by the user configurable limits stored in NVM. 

4. User Configuration message: This message will contain 16 32-bit variables. Each defines a
user configurable parameter that is stored on the machine in NVM and loaded at startup.
The machine uses F-RAM NVM so these may be written as often as needed.

3. General purpose command message: This message will contain two 32-bit variables. The
first is a 32-bit integer general purpose command which is the ID of the configuration 
command (see below). The second is the general purpose parameter which may be integer
or 32-bit IEEE754 floating point value depending on the command
------------------------------------------------------------------------"""

"""
Defines for the structure of commands sent to the RMP each message
consists of an ID and a number 32-bit words depending on the command.
The data bytes are big-endian (highest byte in lowest index).
Command ID: 16-bit ID.
Command Variables: data_bytes[0....n]
Bytes 0-3: 32-bit variable 1 (MS byte at index 0 LS byte at index 3)
Bytes 4-7: 32-bit variable 2 (MS byte at index 4 LS byte at index 7)
repeats in 4 byte blocks for all the variables

The format to rmp_comm.py is 
[command_id (16-bit), [variable_1, variable_2,...variable_n]]
the checksum and breakout into a byte array is handled
by the communication driver
"""
"""
Non-holonomic motion command is used for all platforms except the OMNI
"""
MOTION_CMD_ID                = 0x1800
MOTION_CMD_VEL_INDEX         = 0
MOTION_CMD_YAW_RATE_INDEX    = 1

"""
Load machine configuration command is sent to update the NVM configuration
parameters that get loaded on the machine during initialization
"""
LOAD_MACH_CONFIG_CMD_ID  = 0x1801

"""------------------------------------------------------------------------
Start Variables Stored in NV F-RAM memory
------------------------------------------------------------------------"""

"""------------------------------------------------------------------------
32-bit floating point variable representing the maximum velocity 
limit in m/s.
------------------------------------------------------------------------"""
VEL_LIMIT_INDEX          = (0)
MAX_VELOCITY_MPS         = 8.047
MIN_VELOCITY_MPS         = 0.0

"""------------------------------------------------------------------------
32-bit floating point variable representing the maximum acceleration 
limit in m/s^2.
------------------------------------------------------------------------"""
ACCEL_LIMIT_INDEX        = (1)
MAX_ACCELERATION_MPS2    = 7.848
MIN_ACCELERATION_MPS2    = 0.0

"""------------------------------------------------------------------------
32-bit floating point variable representing the
maximum deceleration limit in m/s^2.
------------------------------------------------------------------------"""
DECEL_LIMIT_INDEX        = (2)
MAX_DECELERATION_MPS2    = 7.848 
MIN_DECELERATION_MPS2    = 0.0

"""------------------------------------------------------------------------
32-bit floating point variable representing the maximum DTZ response deceleration 
limit in m/s^2.
------------------------------------------------------------------------"""
DTZ_DECEL_LIMIT_INDEX     = (3)
MAX_DTZ_DECELERATION_MPS2 = 7.848
MIN_DTZ_DECELERATION_MPS2 = 0.0

"""------------------------------------------------------------------------
32-bit floating point variable representing the yaw rate limit in rad/s.
Yaw rate is defined as the differential wheel velocity
------------------------------------------------------------------------"""
YAW_RATE_LIMIT_INDEX      = (4)
MAX_YAW_RATE_RPS          = 4.5
MIN_YAW_RATE_RPS          = 0.0

"""------------------------------------------------------------------------
32-bit floating point variable representing yaw acceleration in rad/s^2.
------------------------------------------------------------------------"""
YAW_ACCEL_LIMIT_INDEX     = (5)
MAX_YAW_ACCEL_RPS2        = 28.274
MIN_YAW_ACCEL_RPS2        = 0.0

"""------------------------------------------------------------------------
32-bit floating point variable representing lateral acceleration limit in m/s^2.
------------------------------------------------------------------------"""
LATERAL_ACCEL_LIMIT_INDEX  = (6)
MAX_LATERAL_ACCEL_MPS2     = 9.81
MIN_LATERAL_ACCEL_MPS2     = 0.981

"""------------------------------------------------------------------------
32-bit floating point variable representing tire diameter in meters. 
This updates the rolling tire diameter used in software to calculate
velocity, position, differential wheel speed (yaw rate) and accelerations. 
------------------------------------------------------------------------"""
TIRE_DIAMETER_INDEX        = (7)
MAX_TIRE_DIAMETER_M        = 1.0
MIN_TIRE_DIAMETER_M        = 0.1524

"""
Some standard Segway tire diameter definitions
"""
I2_TIRE_DIAMETER_M         = 0.46228
X2_TIRE_DIAMETER_M         = 0.483616
OMNI_TIRE_DIAMETER_M       = 0.254

"""------------------------------------------------------------------------
32-bit floating point variable representing wheel base length in meters.
It is the distance between the center of the front and rear tire contact patches.
This updates the wheel base length used in software to calculate
velocity, position, differential wheel speed (yaw rate) and accelerations. It is always
defined but not used for 2 wheeled platforms.
------------------------------------------------------------------------"""
WHEEL_BASE_LENGTH_INDEX    = (8)
MAX_WHEEL_BASE_LENGTH_M    = 1.0
MIN_WHEEL_BASE_LENGTH_M    = 0.4142

"""
Some standard wheel base length definitions
"""
OMNI_WHEEL_BASE_WIDTH_M     = 0.572
FLEXOMNI_WHEEL_BASE_WIDTH_M = 0.693


"""------------------------------------------------------------------------
32-bit floating point variable representing track width in meters.
It is the distance between the center of the left and right tire contact patches.
This updates the track width used in software to calculate differential wheel speeds (yaw rate). 
------------------------------------------------------------------------"""
WHEEL_TRACK_WIDTH_INDEX    = (9)
MAX_WHEEL_TRACK_WIDTH_M    = 1.0
MIN_WHEEL_TRACK_WIDTH_M    = 0.506476

"""------------------------------------------------------------------------
Some standard wheel base length definitions
"""
I2_WHEEL_TRACK_WIDTH_M     = 0.569976
X2_WHEEL_TRACK_WIDTH_M     = 0.7112
OMNI_WHEEL_TRACK_WIDTH_M   = 0.693

"""------------------------------------------------------------------------
32-bit IEEE754 floating point variable representing gear ratio (unitless).
This updates the gearbox ratio used in software to convert from motor speed 
to gearbox output speed 
------------------------------------------------------------------------"""
GEAR_RATIO_INDEX            = (10)
MAX_GEAR_RATIO              = 200.0
MIN_GEAR_RATIO              = 1.0

"""
Standard gear ratio definition
"""
SEGWAY_STANDARD_GEAR_RATIO  = 24.2667

"""------------------------------------------------------------------------
32-bit integer variable with configuration bits representing each variable defined below.
This bitmap setscertain behaviors that need to be defined at startup
------------------------------------------------------------------------"""
CONFIG_BITMAP_INDEX         = (11)

"""
Enables or disables the machines sounds
"""
ALLOW_MACHINE_AUDIO           = 0
SILENCE_MACHINE_AUDIO         = 1

"""
Allows the machine to run with charger connected
"""
DISABLE_AC_PRESENT_CSI        = 1
ENABLE_AC_PRESENT_CSI         = 0

"""
Enables users ability to enter balance mode (220 only)
"""
BALANCE_MODE_DISABLED         = 0
BALANCE_MODE_ENABLED          = 1

"""
Selects gain schedule for balancing platforms (220 only; see manual)
"""
BALANCE_GAINS_DEFAULT         = (0x0)
BALANCE_GAINS_LIGHT           = (0x1)
BALANCE_GAINS_TALL            = (0x2)
BALANCE_GAINS_HEAVY           = (0x4)
BALANCE_GAINS_CUSTOM          = (0x8)
VALID_BALANCE_GAINS_MASK      = (0xF)

"""
Selects cutoff requency for input commands in the controllers. Used to smooth
out the user commands if the host controller issues stepwise commands
"""
CTL_INPUT_10HZ_FILTER         = (0x0)
CTL_INPUT_4HZ_FILTER          = (0x1)
CTL_INPUT_1HZ_FILTER          = (0x2)
CTL_INPUT_05HZ_FILTER         = (0x4)
CTL_INPUT_02HZ_FILTER         = (0x8)
VALID_VEL_CTL_FILTER_MASK     = (0xF)
VALID_YAW_CTL_FILTER_MASK     = (0xF)

"""
Shifts for the variables in the bitmaps
"""
AUDIO_SILENCE_REQUEST_SHIFT  = 0
DISABLE_AC_PRESENT_CSI_SHIFT = 1
BALANCE_GAIN_SCHEDULE_SHIFT  = 2
BALANCE_MODE_LOCKOUT_SHIFT   = 6
VEL_CTL_FILTER_SHIFT         = 7
YAW_CTL_FILTER_SHIFT         = 11

"""
CONFIG_BITMAP_EXAMPLE = ((ALLOW_MACHINE_AUDIO    << AUDIO_SILENCE_REQUEST_SHIFT) |
                         (DISABLE_AC_PRESENT_CSI << DISABLE_AC_PRESENT_CSI_SHIFT)|
                         (BALANCE_GAINS_DEFAULT  << BALANCE_GAIN_SCHEDULE_SHIFT) |
                         (BALANCE_MODE_DISABLED  << BALANCE_MODE_LOCKOUT_SHIFT)  |
                         (CTL_INPUT_10HZ_FILTER  << VEL_CTL_FILTER_SHIFT)        |
                         (CTL_INPUT_10HZ_FILTER  << YAW_CTL_FILTER_SHIFT))
"""


"""
Load ethernet configuration command is sent to update the NVM ethernet configuration
parameters that get loaded on the machine during initialization
"""
LOAD_ETH_CONFIG_CMD_ID  = 0x1802

"""------------------------------------------------------------------------
32-bit integer variable representing the ethernet IP address representing a
dotted quad IP address.
The conversion is:
integer = (first octet * 16777216) + (second octet * 65536) + (third octet * 256) + (fourth octet)
Bounds for this item are valid IP addresses
------------------------------------------------------------------------"""
ETH_IP_ADDRESS_INDEX                  = (0)

"""------------------------------------------------------------------------
32-bit integer variable representing the ethernet port used to communicate
with the RMP platform
Bounds for this item are valid ethernet ports
------------------------------------------------------------------------"""
ETH_PORT_NUMBER_INDEX                  = (1)

"""------------------------------------------------------------------------
32-bit integer variable representing the ethernet subnet mask representing a
dotted quad IP address.
The conversion is:
integer = (first octet * 16777216) + (second octet * 65536) + (third octet * 256) + (fourth octet)
Bounds for this item are valid IP subnet masks
------------------------------------------------------------------------"""
ETH_SUBNET_MASK_INDEX                  = (2)

"""------------------------------------------------------------------------
32-bit integer variable representing the ethernet gateway representing a
dotted quad IP address.
The conversion is:
integer = (first octet * 16777216) + (second octet * 65536) + (third octet * 256) + (fourth octet)
Bounds for this item are valid IP subnet masks
------------------------------------------------------------------------"""
ETH_SUBNET_MASK_INDEX                  = (3)


"""------------------------------------------------------------------------
End Variables Stored in NV F-RAM memory
------------------------------------------------------------------------"""

"""
General purpose command is used to set runtime variables
such as mode change requests, audio requests, etc.
"""
GENERAL_PURPOSE_CMD_ID       = 0x1803
GENERAL_PURPOSE_CMD_INDEX    = 0
GENERAL_PURPOSE_PARAM_INDEX  = 1

"""------------------------------------------------------------------------
This command results in no action but a response from the RMP. The general
purpose parameter is ignored.
------------------------------------------------------------------------"""
GENERAL_PURPOSE_CMD_NONE                   = (0)

"""------------------------------------------------------------------------
This command requests audio songs on the machine. The general purpose parameter 
is a 32 bit integer representing the song request ID. Some songs are presistant (ie they must be
manually cleared by sending the NO_SONG after set). Most are momentary.
------------------------------------------------------------------------"""
GENERAL_PURPOSE_CMD_SET_AUDIO_COMMAND       = (1)

MOTOR_AUDIO_PLAY_NO_SONG                    = (0)
MOTOR_AUDIO_PLAY_POWER_ON_SONG              = (1)
MOTOR_AUDIO_PLAY_POWER_OFF_SONG             = (2)
MOTOR_AUDIO_PLAY_ALARM_SONG                 = (3)
MOTOR_AUDIO_PLAY_MODE_UP_SONG               = (4)
MOTOR_AUDIO_PLAY_MODE_DOWN_SONG             = (5)
MOTOR_AUDIO_PLAY_ENTER_ALARM_SONG           = (6)
MOTOR_AUDIO_PLAY_EXIT_ALARM_SONG            = (7)
MOTOR_AUDIO_PLAY_FINAL_SHUTDOWN_SONG        = (8)
MOTOR_AUDIO_PLAY_CORRECT_ISSUE              = (9)
MOTOR_AUDIO_PLAY_ISSUE_CORRECTED            = (10)
MOTOR_AUDIO_PLAY_CORRECT_ISSUE_REPEATING    = (11)
MOTOR_AUDIO_PLAY_BEGINNER_ACK               = (12)
MOTOR_AUDIO_PLAY_EXPERT_ACK                 = (13)
MOTOR_AUDIO_ENTER_FOLLOW                    = (14)
MOTOR_AUDIO_TEST_SWEEP                      = (15)
MOTOR_AUDIO_SIMULATE_MOTOR_NOISE            = (16)

"""------------------------------------------------------------------------
This command updates the operational mode request on the machine. The general 
purpose parameter is a 32 bit integer representing the mode request ID.
------------------------------------------------------------------------"""
GENERAL_PURPOSE_CMD_SET_OPERATIONAL_MODE    = (2)

DISABLE_REQUEST   = 1
POWERDOWN_REQUEST = 2
DTZ_REQUEST       = 3
STANDBY_REQUEST   = 4
TRACTOR_REQUEST   = 5
BALANCE_REQUEST   = 6

RMP_MODES_DICT = dict({TRACTOR_REQUEST:4,STANDBY_REQUEST:3,BALANCE_REQUEST:5,POWERDOWN_REQUEST:6})
RMP_MODES_AUDIO_DICT = dict({TRACTOR_REQUEST:6,STANDBY_REQUEST:7,BALANCE_REQUEST:15,POWERDOWN_REQUEST:2})

"""------------------------------------------------------------------------
This command requests the faultlog from the machine. The general purpose parameter 
is 0. This request will send the entire faultlog for 1248 bytes includin the 
checksum. 311 words without the checksum
------------------------------------------------------------------------"""
GENERAL_PURPOSE_CMD_SEND_SP_FAULTLOG      = (3)
"""
Define the number of faultlog packets and the size (in 32-bit words) of each packet.
"""
NUMBER_OF_FAULTLOG_WORDS              = 311

"""
This command resets the position data on the machine. The general purpose command
is a bitmap of which integrators are to be reset.
""" 
GENERAL_PURPOSE_CMD_RESET_INTEGRATORS      = (4)

RESET_LINEAR_POSITION      = 0x00000001
RESET_RIGHT_FRONT_POSITION = 0x00000002
RESET_LEFT_FRONT_POSITION  = 0x00000004
RESET_RIGHT_REAR_POSITION  = 0x00000008
RESET_LEFT_REAR_POSITION   = 0x00000010
RESET_ALL_POSITION_DATA    = 0x0000001F

"""
This command resets all configurable parameters to their default values. 
The general purpose parameter is ignored for this request.
""" 
GENERAL_PURPOSE_CMD_RESET_PARAMS_TO_DEFAULT = (5)

"""
Sets the flag to send continuous data at system frequency (100.0Hz)
"""
GENERAL_PURPOSE_CMD_SEND_CONTINUOUS_DATA    = (6)

"""
Sets the output torque limit from 0 to 100 percent (parameter is scaled 0.0 to 1.0)
"""
GENERAL_PURPOSE_CMD_SET_TORQUE_LIMIT        = (7)

"""------------------------------------------------------------------------
RMP Fault definitions
This section is used to define the decoding of fault status words sent 
by the RMP. The meaning of specific faults can be found in the interface
guide.
------------------------------------------------------------------------"""
NO_FAULT                                    = 0x00000000
ALL_FAULTS                                  = 0xFFFFFFFF

"""
Transient faults: These faults are not latching and can be asserted and then
cleared during runtime. There are currently no transient faults for the RMP
"""
transient_fault_decode = dict({
    0x00000000: ""})

"""
Critical faults: These faults are latching.
"""
critical_fault_decode = dict({
    0x00000000: "",                          
    0x00000001:"CRITICAL_FAULT_INIT",
    0x00000002:"CRITICAL_FAULT_INIT_UIP_COMM",
    0x00000004:"CRITICAL_FAULT_INIT_PROPULSION",
    0x00000008:"CRITICAL_FAULT_INIT_TIMEOUT",
    0x00000010:"CRITICAL_FAULT_FORW_SPEED_LIMITER_HAZARD",
    0x00000020:"CRITICAL_FAULT_AFT_SPEED_LIMITER_HAZARD",
    0x00000040:"CRITICAL_FAULT_CHECK_STARTUP",
    0x00000080:"CRITICAL_FAULT_APP_VELOCITY_CTL_FAILED",
    0x00000100:"CRITICAL_FAULT_APP_POSITION_CTL_FAILED",
    0x00000200:"CRITICAL_FAULT_ABB_SHUTDOWN",
    0x00000400:"CRITICAL_FAULT_AP_MODE_TRANS_TIMEOUT",
    0x00000800:"CRITICAL_FAULT_PITCH_ANGLE_EXCEEDED",
    0x00001000:"CRITICAL_FAULT_ROLL_ANGLE_EXCEEDED",
    0x00002000:"CRITICAL_FAULT_BSB_INIT_FAILED",
    0x00004000:"CRITICAL_FAULT_BSB_COMM_FAILED",
    0x00008000:"CRITICAL_FAULT_BSB_LOST_POWER",
    0x00010000:"CRITICAL_FAULT_BSB_HW_FAULT"})

"""
Communication faults: These faults are latching.
"""
comm_fault_decode = dict({
    0x00000000: "",
    0x00000001:"COMM_FAULT_UIP_MISSING_UIP_DATA",
    0x00000002:"COMM_FAULT_UIP_UNKNOWN_MESSAGE_RECEIVED",
    0x00000004:"COMM_FAULT_UIP_BAD_CHECKSUM",
    0x00000008:"COMM_FAULT_UIP_TRANSMIT",
    0x00000010:"COMM_FAULT_UI_BAD_MOTION_CMD",
    0x00000020:"COMM_FAULT_UI_UNKOWN_CMD",
    0x00000040:"COMM_FAULT_UI_BAD_PACKET_CHECKSUM"})


"""
MCU faults: These faults are latching.
"""
mcu_fault_decode = dict({
    0x00000000: "",                     
    0x00000001:"MCU_FAULT_MCU_0_IS_DEGRADED",
    0x00000002:"MCU_FAULT_MCU_0_IS_FAILED",
    0x00000004:"MCU_FAULT_MCU_0_REQUESTS_REDUCED_PERFORMANCE",
    0x00000008:"MCU_FAULT_MCU_0_REQUESTS_ZERO_SPEED",
    0x00000010:"MCU_FAULT_MCU_1_IS_DEGRADED",
    0x00000020:"MCU_FAULT_MCU_1_IS_FAILED",
    0x00000040:"MCU_FAULT_MCU_1_REQUESTS_REDUCED_PERFORMANCE",
    0x00000080:"MCU_FAULT_MCU_1_REQUESTS_ZERO_SPEED",
    0x00000100:"MCU_FAULT_MCU_2_IS_DEGRADED",
    0x00000200:"MCU_FAULT_MCU_2_IS_FAILED",
    0x00000400:"MCU_FAULT_MCU_2_REQUESTS_REDUCED_PERFORMANCE",
    0x00000800:"MCU_FAULT_MCU_2_REQUESTS_ZERO_SPEED",
    0x00001000:"MCU_FAULT_MCU_3_IS_DEGRADED",
    0x00002000:"MCU_FAULT_MCU_3_IS_FAILED",
    0x00004000:"MCU_FAULT_MCU_3_REQUESTS_REDUCED_PERFORMANCE",
    0x00008000:"MCU_FAULT_MCU_3_REQUESTS_ZERO_SPEED",
    0x00010000:"MCU_FAULT_MISSING_MCU_0_DATA",
    0x00020000:"MCU_FAULT_MISSING_MCU_1_DATA",
    0x00040000:"MCU_FAULT_MISSING_MCU_2_DATA",
    0x00080000:"MCU_FAULT_MISSING_MCU_3_DATA",
    0x00100000:"MCU_FAULT_UNKNOWN_MESSAGE_RECEIVED"})

"""
Define a mask to indicate that the CCU has detected the fault and not the MCU
"""
CCU_DETECTED_MCU_FAULT_MASK = 0x001F0000

"""
Sensor faults: These faults are latching.
"""
sensor_fault_decode = dict({
    0x00000000: "",                        
    0x00000001:"SENSOR_FAULT_2P5V_VREF_RANGE_FAULT",
    0x00000002:"SENSOR_FAULT_7P2V_VBAT_RANGE_FAULT",
    0x00000004:"SENSOR_FAULT_7P2V_VBAT_WARNING",
    0x00000008:"SENSOR_FAULT_7P2V_BATT_INBALANCE_FAULT",
    0x00000010:"SENSOR_FAULT_7P2V_BATT_TEMPERATURE_FAULT",
    0x00000020:"SENSOR_FAULT_DIGITAL_INPUT",
    0x00000040:"SENSOR_FAULT_RANGE",
    0x00000080:"SENSOR_FAULT_DEFAULT",
    0x00000100:"SENSOR_FAULT_5V_MONITOR_RANGE_FAULT",
    0x00000200:"SENSOR_FAULT_12V_MONITOR_RANGE_FAULT"})
 
"""
BSA faults: These faults are latching.
"""
bsa_fault_decode = dict({
    0x00000000: "",                     
    0x00000001:"BSA_FAULT_SIDE_A_MISSING_BSA_DATA",
    0x00000002:"BSA_FAULT_SIDE_B_MISSING_BSA_DATA",
    0x00000004:"BSA_FAULT_UNKNOWN_MESSAGE_RECEIVED",
    0x00000008:"BSA_FAULT_TRANSMIT_A_FAILED",
    0x00000010:"BSA_FAULT_TRANSMIT_B_FAILED",
    0x00000020:"BSA_FAULT_DEFAULT",
    0x00000040:"BSA_FAULT_SIDE_A_RATE_SENSOR_SATURATED",
    0x00000080:"BSA_FAULT_SIDE_B_RATE_SENSOR_SATURATED",
    0x00000100:"BSA_FAULT_SIDE_A_TILT_SENSOR_SATURATED",
    0x00000200:"BSA_FAULT_SIDE_B_TILT_SENSOR_SATURATED",
    0x00000400:"PSE_FAULT_COMPARISON"})

"""
Architecture faults: These faults are latching.
"""
arch_fault_decode = dict({
    0x00000000: "",                      
    0x00000001:"ARCHITECT_FAULT_SPI_RECEIVE",
    0x00000002:"ARCHITECT_FAULT_SPI_TRANSMIT",
    0x00000004:"ARCHITECT_FAULT_SPI_RECEIVE_OVERRUN",
    0x00000008:"ARCHITECT_FAULT_SPI_RX_BUFFER_OVERRUN",
    0x00000010:"ARCHITECT_FAULT_COMMANDED_SAFETY_SHUTDOWN",
    0x00000020:"ARCHITECT_FAULT_COMMANDED_DISABLE",
    0x00000040:"ARCHITECT_FAULT_KILL_SWITCH_ACTIVE",
    0x00000080:"ARCHITECT_FAULT_FRAM_CONFIG_INIT_FAILED",
    0x00000100:"ARCHITECT_FAULT_FRAM_CONFIG_SET_FAILED",
    0x00000200:"ARCHITECT_FAULT_BAD_MODEL_IDENTIFIER",
    0x00000400:"ARCHITECT_FAULT_BAD_CCU_HW_REV",
    0x00000800:"ARCHITECT_FAULT_INVALID_SI_KEYS"})

"""
Internal faults: These faults are latching.
"""
internal_fault_decode = dict({
    0x00000000: "",                          
    0x00000001:"INTERNAL_FAULT_HIT_DEFAULT_CONDITION",
    0x00000002:"INTERNAL_FAULT_HIT_SPECIAL_CASE"})

"""
MCU specific faults: These faults are detected locally by the MCU
"""
mcu_specific_fault_decode = dict({
    0x00000000: "",                              
    0x00000001:"MCU_TRANS_BATTERY_TEMP_WARNING",
    0x00000002:"MCU_TRANS_BATTERY_COLD_REGEN",
    0x00000004:"MCU_UNKNOWN",
    0x00000008:"MCU_UNKNOWN",
    0x00000010:"MCU_TRANS_LOW_BATTERY",
    0x00000020:"MCU_TRANS_BATT_OVERVOLTAGE",
    0x00000040:"MCU_CRITICAL_BATT_OVERVOLTAGE",
    0x00000080:"MCU_CRITICAL_EMPTY_BATTERY",
    0x00000100:"MCU_CRITICAL_BATTERY_TEMP",
    0x00000200:"MCU_COMM_CU_BCU_LINK_DOWN",
    0x00000400:"MCU_COMM_INITIALIZATION_FAILED",
    0x00000800:"MCU_COMM_FAILED_CAL_EEPROM",
    0x00001000:"MCU_POWER_SUPPLY_TRANSIENT_FAULT",
    0x00002000:"MCU_POWER_SUPPLY_12V_FAULT",
    0x00004000:"MCU_POWER_SUPPLY_5V_FAULT",
    0x00008000:"MCU_POWER_SUPPLY_3V_FAULT",
    0x00010000:"MCU_JUNCTION_TEMP_FAULT",
    0x00020000:"MCU_MOTOR_WINDING_TEMP_FAULT",
    0x00040000:"MCU_MOTOR_DRIVE_FAULT",
    0x00080000:"MCU_MOTOR_DRIVE_HALL_FAULT",
    0x00100000:"MCU_MOTOR_DRIVE_AMP_FAULT",
    0x00200000:"MCU_MOTOR_DRIVE_AMP_ENABLE_FAULT",
    0x00400000:"MCU_MOTOR_DRIVE_AMP_OVERCURRENT_FAULT",
    0x00800000:"MCU_MOTOR_DRIVE_VOLTAGE_FEEDBACK_FAULT",
    0x01000000:"MCU_FRAME_FAULT",
    0x02000000:"MCU_BATTERY_FAULT",
    0x08000000:"MCU_MOTOR_STUCK_RELAY_FAULT",
    0x10000000:"MCU_ACTUATOR_POWER_CONSISTENCY_FAULT",
    0x20000000:"MCU_ACTUATOR_HALT_PROCESSOR_FAULT",
    0x40000000:"MCU_ACTUATOR_DEGRADED_FAULT"})

"""
All the fault groups are packed into four 32-bit fault status words. The following
defines how they are packed into the words
"""

"""
Fault status word 0
"""
FSW_ARCH_FAULTS_INDEX       = 0
FSW_ARCH_FAULTS_SHIFT       = 0
FSW_ARCH_FAULTS_MASK        = 0x00000FFF
FSW_CRITICAL_FAULTS_INDEX   = 0
FSW_CRITICAL_FAULTS_SHIFT   = 12
FSW_CRITICAL_FAULTS_MASK    = 0xFFFFF000
"""
Fault status word 1
"""
FSW_COMM_FAULTS_INDEX       = 1
FSW_COMM_FAULTS_SHIFT       = 0
FSW_COMM_FAULTS_MASK        = 0x0000FFFF
FSW_INTERNAL_FAULTS_INDEX   = 1
FSW_INTERNAL_FAULTS_SHIFT   = 16
FSW_INTERNAL_FAULTS_MASK    = 0x000F0000
"""
Fault status word 2
"""
FSW_SENSORS_FAULTS_INDEX    = 2
FSW_SENSORS_FAULTS_SHIFT    = 0
FSW_SENSORS_FAULTS_MASK     = 0x0000FFFF
FSW_BSA_FAULTS_INDEX        = 2
FSW_BSA_FAULTS_SHIFT        = 16
FSW_BSA_FAULTS_MASK         = 0xFFFF0000
"""
Fault status word 3
"""
FSW_MCU_FAULTS_INDEX        = 3
FSW_MCU_FAULTS_SHIFT        = 0
FSW_MCU_FAULTS_MASK         = 0xFFFFFFFF

"""
Fault group index definitions
"""
FAULTGROUP_TRANSIENT    = 0
FAULTGROUP_CRITICAL     = 1
FAULTGROUP_COMM         = 2
FAULTGROUP_SENSORS      = 3
FAULTGROUP_BSA          = 4
FAULTGROUP_MCU          = 5
FAULTGROUP_ARCHITECTURE = 6  
FAULTGROUP_INTERNAL     = 7
NUM_OF_FAULTGROUPS      = 8


"""
Defines the feedback array indices
"""
START_STATUS_BLOCK                    =(0)

CCU_FSW_1_INDEX                       =(0)
CCU_FSW_2_INDEX                       =(1)
CCU_FSW_3_INDEX                       =(2)
CCU_FSW_4_INDEX                       =(3)
CCU_MCU_0_FSW_INDEX                   =(4)
CCU_MCU_1_FSW_INDEX                   =(5)
CCU_MCU_2_FSW_INDEX                   =(6)
CCU_MCU_3_FSW_INDEX                   =(7)
CCU_FRAME_COUNT_INDEX                 =(8)
CCU_OPERATIONAL_STATE_INDEX           =(9)
CCU_DYNAMIC_RESPONSE_INDEX            =(10)
CCU_PACKED_BUILDID_INDEX              =(11)
CCU_MACHINE_ID_INDEX                  =(12)
END_STATUS_BLOCK                      =(13)

START_PROPULSION_POWER_BLOCK          =(13)

CCU_MIN_PROP_SOC_INDEX                =(13)
CCU_MCU_0_SOC_INDEX                   =(14)
CCU_MCU_1_SOC_INDEX                   =(15)
CCU_MCU_2_SOC_INDEX                   =(16)
CCU_MCU_3_SOC_INDEX                   =(17)
CCU_MCU_0_TEMP_INDEX                  =(18)
CCU_MCU_1_TEMP_INDEX                  =(19)
CCU_MCU_2_TEMP_INDEX                  =(20)
CCU_MCU_3_TEMP_INDEX                  =(21)
CCU_MCU_0_INST_POWER_INDEX            =(22)
CCU_MCU_1_INST_POWER_INDEX            =(23)
CCU_MCU_2_INST_POWER_INDEX            =(24)
CCU_MCU_3_INST_POWER_INDEX            =(25)
CCU_MCU_0_TOTAL_ENERGY_INDEX          =(26)
CCU_MCU_1_TOTAL_ENERGY_INDEX          =(27)
CCU_MCU_2_TOTAL_ENERGY_INDEX          =(28)
CCU_MCU_3_TOTAL_ENERGY_INDEX          =(29)
CCU_RF_MOTOR_CURRENT_INDEX            =(30)
CCU_LF_MOTOR_CURRENT_INDEX            =(31)
CCU_RR_MOTOR_CURRENT_INDEX            =(32)
CCU_LR_MOTOR_CURRENT_INDEX            =(33)
CCU_MAX_MOTOR_CURRENT_INDEX           =(34)
CCU_RF_MOTOR_CURRENT_LIMIT_INDEX      =(35)
CCU_LF_MOTOR_CURRENT_LIMIT_INDEX      =(36)
CCU_RR_MOTOR_CURRENT_LIMIT_INDEX      =(37)
CCU_LR_MOTOR_CURRENT_LIMIT_INDEX      =(38)
CCU_MIN_CURRENT_LIMIT_INDEX           =(39)
END_PROPULSION_POWER_BLOCK            =(40)

START_AUX_POWER_BLOCK                 =(40)

CCU_AUX_0_SOC_INDEX                   =(40)
CCU_AUX_1_SOC_INDEX                   =(41)
CCU_AUX_0_VOLTAGE_INDEX               =(42)
CCU_AUX_1_VOLTAGE_INDEX               =(43)
CCU_AUX_0_CURRENT_INDEX               =(44)
CCU_AUX_1_CURRENT_INDEX               =(45)
CCU_AUX_0_TEMP_INDEX                  =(46)
CCU_AUX_1_TEMP_INDEX                  =(47)
CCU_AUX_0_SYS_STATUS_INDEX            =(48)
CCU_AUX_1_SYS_STATUS_INDEX            =(49)
CCU_AUX_0_BCU_STATUS_INDEX            =(50)
CCU_AUX_1_BCU_STATUS_INDEX            =(51)
CCU_AUX_0_BCU_FAULTS_INDEX            =(52)
CCU_AUX_1_BCU_FAULTS_INDEX            =(53)
CCU_7P2V_BATTERY_VOLTAGE_INDEX        =(54)
END_AUX_POWER_BLOCK                   =(55)

START_IMU_BLOCK                       =(55)

CCU_INERTIAL_X_ACC_INDEX              =(55)
CCU_INERTIAL_Y_ACC_INDEX              =(56)
CCU_INERTIAL_X_RATE_INDEX             =(57)
CCU_INERTIAL_Y_RATE_INDEX             =(58)
CCU_INERTIAL_Z_RATE_INDEX             =(59)
CCU_PSE_PITCH_INDEX                   =(60)
CCU_PSE_PITCH_RATE_INDEX              =(61)
CCU_PSE_ROLL_INDEX                    =(62)
CCU_PSE_ROLL_RATE_INDEX               =(63)
CCU_PSE_YAW_RATE_INDEX                =(64)
CCU_PSE_DATA_VALID_INDEX              =(65)
CCU_AHRS_X_ACCEL_INDEX                =(66)
CCU_AHRS_Y_ACCEL_INDEX                =(67)
CCU_AHRS_Z_ACCEL_INDEX                =(68)
CCU_AHRS_X_GYRO_INDEX                 =(69)
CCU_AHRS_Y_GYRO_INDEX                 =(70)
CCU_AHRS_Z_GYRO_INDEX                 =(71)
CCU_AHRS_X_MAG_INDEX                  =(72)
CCU_AHRS_Y_MAG_INDEX                  =(73)
CCU_AHRS_Z_MAG_INDEX                  =(74)
CCU_AHRS_ROLL_INDEX                   =(75)
CCU_AHRS_PITCH_INDEX                  =(76)
CCU_AHRS_YAW_INDEX                    =(77)
CCU_AHRS_TIMESTAMP_INDEX              =(78)
CCU_GPS_LATITUDE_HWORD_INDEX          =(79)
CCU_GPS_LATITUDE_LWORD_INDEX          =(80)
CCU_GPS_LONGTITUDE_HWORD_INDEX        =(81)
CCU_GPS_LONGTITUDE_LWORD_INDEX        =(82)
CCU_GPS_EL_HEIGHT_HWORD_INDEX         =(83)
CCU_GPS_EL_HEIGHT_LWORD_INDEX         =(84)
CCU_GPS_MSL_HEIGHT_HWORD_INDEX        =(85)
CCU_GPS_MSL_HEIGHT_LWORD_INDEX        =(86)
CCU_GPS_HORIZONTAL_STDEV_INDEX        =(87)
CCU_GPS_VERTICAL_STDEV_INDEX          =(88)
CCU_GPS_VALID_LLH_FLAG_INDEX          =(89)
CCU_GPS_PACKED_FIX_INFO_INDEX         =(90)
CCU_GPS_PACKED_HW_STATUS_INDEX        =(91)
CCU_IMU_STATUS_INDEX                  =(92)
CCU_IMU_ERROR_INDEX                   =(93)
END_IMU_BLOCK                         =(94)

START_DYNAMICS_BLOCK                  =(94)

CCU_X_VEL_TARGET_INDEX                =(94)
CCU_Y_VEL_TARGET_INDEX                =(95)
CCU_YAW_TARGET_INDEX                  =(96)
CCU_YAW_RATE_LIMIT_INDEX              =(97)
CCU_VELOCITY_LIMIT_INDEX              =(98)
CCU_RF_WHEEL_VEL_INDEX                =(99)
CCU_LF_WHEEL_VEL_INDEX                =(100)
CCU_RR_WHEEL_VEL_INDEX                =(101)
CCU_LR_WHEEL_VEL_INDEX                =(102)
CCU_RF_WHEEL_POS_INDEX                =(103)
CCU_LF_WHEEL_POS_INDEX                =(104)
CCU_RR_WHEEL_POS_INDEX                =(105)
CCU_LR_WHEEL_POS_INDEX                =(106)
CCU_ODOM_X_ACCEL_INDEX				  =(107)
CCU_ODOM_Y_ACCEL_INDEX				  =(108)
CCU_ODOM_YAW_ACCEL_INDEX    		  =(109)
CCU_ODOM_X_VELOCITY_INDEX		      =(110)
CCU_ODOM_Y_VELOCITY_INDEX		      =(111)
CCU_ODOM_YAW_VELOCITY_INDEX    		  =(112)
CCU_ODOM_X_POSITION_INDEX             =(113)
CCU_ODOM_Y_POSITION_INDEX             =(114)
CCU_ODOM_YAW_POSITION_INDEX           =(115)
END_DYNAMICS_BLOCK                    =(116)

START_CONFIG_BLOCK                    =(116)

CCU_FRAM_VEL_LIMIT_INDEX              =(116)
CCU_FRAM_ACCEL_LIMIT_INDEX            =(117)
CCU_FRAM_DECEL_LIMIT_INDEX            =(118)
CCU_FRAM_MAX_DTZ_DECEL_INDEX          =(119)
CCU_FRAM_YAW_RATE_LIMIT_INDEX         =(120)
CCU_FRAM_YAW_ACCEL_LIMIT_INDEX        =(121)
CCU_FRAM_LATERAL_ACCEL_LIMIT_INDEX    =(122)
CCU_FRAM_TIRE_DIAMETER_INDEX          =(123)
CCU_FRAM_WHEEL_BASE_LENGTH_INDEX      =(124)
CCU_FRAM_WHEEL_TRACK_WIDTH_INDEX      =(125)
CCU_FRAM_TRANSMISSION_RATIO_INDEX     =(126)
CCU_FRAM_CFG_BITMAP_INDEX             =(127)
CCU_FRAM_ETH_IP_ADDRESS_INDEX         =(128)
CCU_FRAM_ETH_PORT_NUMBER_INDEX        =(129)
CCU_FRAM_ETH_SUBNET_MASK_INDEX        =(130)
CCU_FRAM_ETH_GATEWAY_INDEX            =(131)
END_CONFIG_BLOCK                      =(132)

NUMBER_OF_CONFIG_PARAM_VARIABLES      =(END_CONFIG_BLOCK - START_CONFIG_BLOCK)
NUMBER_OF_RMP_RSP_WORDS               =(END_CONFIG_BLOCK)
NUMBER_OF_FAULTLOG_WORDS              =(311)

