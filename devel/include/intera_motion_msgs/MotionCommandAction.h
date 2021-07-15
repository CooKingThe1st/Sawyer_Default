// Generated by gencpp from file intera_motion_msgs/MotionCommandAction.msg
// DO NOT EDIT!


#ifndef INTERA_MOTION_MSGS_MESSAGE_MOTIONCOMMANDACTION_H
#define INTERA_MOTION_MSGS_MESSAGE_MOTIONCOMMANDACTION_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <intera_motion_msgs/MotionCommandActionGoal.h>
#include <intera_motion_msgs/MotionCommandActionResult.h>
#include <intera_motion_msgs/MotionCommandActionFeedback.h>

namespace intera_motion_msgs
{
template <class ContainerAllocator>
struct MotionCommandAction_
{
  typedef MotionCommandAction_<ContainerAllocator> Type;

  MotionCommandAction_()
    : action_goal()
    , action_result()
    , action_feedback()  {
    }
  MotionCommandAction_(const ContainerAllocator& _alloc)
    : action_goal(_alloc)
    , action_result(_alloc)
    , action_feedback(_alloc)  {
  (void)_alloc;
    }



   typedef  ::intera_motion_msgs::MotionCommandActionGoal_<ContainerAllocator>  _action_goal_type;
  _action_goal_type action_goal;

   typedef  ::intera_motion_msgs::MotionCommandActionResult_<ContainerAllocator>  _action_result_type;
  _action_result_type action_result;

   typedef  ::intera_motion_msgs::MotionCommandActionFeedback_<ContainerAllocator>  _action_feedback_type;
  _action_feedback_type action_feedback;





  typedef boost::shared_ptr< ::intera_motion_msgs::MotionCommandAction_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::intera_motion_msgs::MotionCommandAction_<ContainerAllocator> const> ConstPtr;

}; // struct MotionCommandAction_

typedef ::intera_motion_msgs::MotionCommandAction_<std::allocator<void> > MotionCommandAction;

typedef boost::shared_ptr< ::intera_motion_msgs::MotionCommandAction > MotionCommandActionPtr;
typedef boost::shared_ptr< ::intera_motion_msgs::MotionCommandAction const> MotionCommandActionConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::intera_motion_msgs::MotionCommandAction_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::intera_motion_msgs::MotionCommandAction_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace intera_motion_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'intera_core_msgs': ['/home/ros/ros_ws/src/intera_common/intera_core_msgs/msg', '/home/ros/ros_ws/devel/share/intera_core_msgs/msg'], 'sensor_msgs': ['/opt/ros/kinetic/share/sensor_msgs/cmake/../msg'], 'actionlib_msgs': ['/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg'], 'intera_motion_msgs': ['/home/ros/ros_ws/src/intera_common/intera_motion_msgs/msg', '/home/ros/ros_ws/devel/share/intera_motion_msgs/msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::intera_motion_msgs::MotionCommandAction_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::intera_motion_msgs::MotionCommandAction_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::intera_motion_msgs::MotionCommandAction_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::intera_motion_msgs::MotionCommandAction_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::intera_motion_msgs::MotionCommandAction_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::intera_motion_msgs::MotionCommandAction_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::intera_motion_msgs::MotionCommandAction_<ContainerAllocator> >
{
  static const char* value()
  {
    return "b4e2247aeb284db6a4a3b8946729492d";
  }

  static const char* value(const ::intera_motion_msgs::MotionCommandAction_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xb4e2247aeb284db6ULL;
  static const uint64_t static_value2 = 0xa4a3b8946729492dULL;
};

template<class ContainerAllocator>
struct DataType< ::intera_motion_msgs::MotionCommandAction_<ContainerAllocator> >
{
  static const char* value()
  {
    return "intera_motion_msgs/MotionCommandAction";
  }

  static const char* value(const ::intera_motion_msgs::MotionCommandAction_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::intera_motion_msgs::MotionCommandAction_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
\n\
MotionCommandActionGoal action_goal\n\
MotionCommandActionResult action_result\n\
MotionCommandActionFeedback action_feedback\n\
\n\
================================================================================\n\
MSG: intera_motion_msgs/MotionCommandActionGoal\n\
# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
\n\
Header header\n\
actionlib_msgs/GoalID goal_id\n\
MotionCommandGoal goal\n\
\n\
================================================================================\n\
MSG: std_msgs/Header\n\
# Standard metadata for higher-level stamped data types.\n\
# This is generally used to communicate timestamped data \n\
# in a particular coordinate frame.\n\
# \n\
# sequence ID: consecutively increasing ID \n\
uint32 seq\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n\
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
\n\
================================================================================\n\
MSG: actionlib_msgs/GoalID\n\
# The stamp should store the time at which this goal was requested.\n\
# It is used by an action server when it tries to preempt all\n\
# goals that were requested before a certain time\n\
time stamp\n\
\n\
# The id provides a way to associate feedback and\n\
# result message with specific goal requests. The id\n\
# specified must be unique.\n\
string id\n\
\n\
\n\
================================================================================\n\
MSG: intera_motion_msgs/MotionCommandGoal\n\
# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
# Engine command goal\n\
string command\n\
string MOTION_START=start\n\
string MOTION_STOP=stop\n\
string MOTION_GENERATE=generate  # Generate path, but do not run\n\
\n\
Trajectory trajectory\n\
\n\
\n\
================================================================================\n\
MSG: intera_motion_msgs/Trajectory\n\
# Representation of a trajectory used by the engine and motion controller.\n\
\n\
# optional label\n\
string label\n\
\n\
# Array of joint names that correspond to the waypoint joint_positions\n\
string[] joint_names\n\
\n\
# Array of waypoints that comprise the trajectory\n\
Waypoint[] waypoints\n\
\n\
# Trajectory level options\n\
TrajectoryOptions trajectory_options\n\
================================================================================\n\
MSG: intera_motion_msgs/Waypoint\n\
# Representation of a waypoint used by the motion controller\n\
\n\
# Desired joint positions\n\
# For Cartesian segments, the joint positions are used as nullspace biases\n\
float64[] joint_positions\n\
\n\
# Name of the endpoint that is currently active\n\
string active_endpoint\n\
\n\
# Cartesian pose\n\
# This is not used in trajectories using joint interpolation\n\
geometry_msgs/PoseStamped pose\n\
\n\
# Waypoint specific options\n\
# Default values will be used if not set\n\
# All waypoint options are applied to the segment moving to that waypoint\n\
WaypointOptions options\n\
\n\
================================================================================\n\
MSG: geometry_msgs/PoseStamped\n\
# A Pose with reference coordinate frame and timestamp\n\
Header header\n\
Pose pose\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Pose\n\
# A representation of pose in free space, composed of position and orientation. \n\
Point position\n\
Quaternion orientation\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Point\n\
# This contains the position of a point in free space\n\
float64 x\n\
float64 y\n\
float64 z\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Quaternion\n\
# This represents an orientation in free space in quaternion form.\n\
\n\
float64 x\n\
float64 y\n\
float64 z\n\
float64 w\n\
\n\
================================================================================\n\
MSG: intera_motion_msgs/WaypointOptions\n\
# Optional waypoint label\n\
string label\n\
\n\
# Ratio of max allowed joint speed : max planned joint speed (from 0.0 to 1.0)\n\
float64 max_joint_speed_ratio\n\
\n\
# Slowdown heuristic is triggered if tracking error exceeds tolerances - radians\n\
float64[] joint_tolerances\n\
\n\
# Maximum accelerations for each joint (only for joint paths) - rad/s^2.\n\
float64[] max_joint_accel\n\
\n\
\n\
###########################################################\n\
# The remaining parameters only apply to Cartesian paths\n\
\n\
# Maximum linear speed of endpoint - m/s\n\
float64 max_linear_speed\n\
\n\
# Maximum linear acceleration of endpoint - m/s^2\n\
float64 max_linear_accel\n\
\n\
# Maximum rotational speed of endpoint - rad/s\n\
float64 max_rotational_speed\n\
\n\
# Maximum rotational acceleration of endpoint - rad/s^2\n\
float64 max_rotational_accel\n\
\n\
# Used for smoothing corners for continuous motion - m\n\
# The distance from the waypoint to where the curve starts while blending from\n\
# one straight line segment to the next.\n\
# Larger distance:  trajectory passes farther from the waypoint at a higher speed\n\
# Smaller distance:  trajectory passes closer to the waypoint at a lower speed\n\
# Zero distance:  trajectory passes through the waypoint at zero speed\n\
float64 corner_distance\n\
\n\
================================================================================\n\
MSG: intera_motion_msgs/TrajectoryOptions\n\
# Trajectory interpolation type\n\
string CARTESIAN=CARTESIAN\n\
string JOINT=JOINT\n\
string interpolation_type\n\
\n\
# True if the trajectory uses interaction control, false for position control.\n\
bool interaction_control\n\
\n\
# Interaction control parameters\n\
intera_core_msgs/InteractionControlCommand interaction_params\n\
\n\
# Allow small joint adjustments at the beginning of Cartesian trajectories.\n\
# Set to false for 'small' motions.\n\
bool nso_start_offset_allowed\n\
\n\
# Check the offset at the end of a Cartesian trajectory from the final waypoint nullspace goal.\n\
bool nso_check_end_offset\n\
\n\
# Options for the tracking controller:\n\
TrackingOptions tracking_options\n\
\n\
# Desired trajectory end time, ROS timestamp\n\
time end_time\n\
\n\
# The rate in seconds that the path is interpolated and returned back to the user\n\
# No interpolation will happen if set to zero\n\
float64 path_interpolation_step\n\
\n\
================================================================================\n\
MSG: intera_core_msgs/InteractionControlCommand\n\
# Message sets the interaction (impedance/force) control on or off\n\
# It also contains desired cartesian stiffness K, damping D, and force values\n\
\n\
Header header\n\
bool      interaction_control_active\n\
\n\
## Cartesian Impedance Control Parameters\n\
# Stiffness units are (N/m) for first 3 and (Nm/rad) for second 3 values\n\
float64[] K_impedance\n\
# Force certain directions to have maximum possible impedance for a given pose\n\
bool[] max_impedance\n\
# Damping units are (Ns/m) for first 3 and (Nms/rad) for the second 3 values\n\
float64[] D_impedance\n\
# Joint Nullspace stiffness units are in (Nm/rad) (length == number of joints)\n\
float64[] K_nullspace\n\
\n\
## Parameters for force control or impedance control with force limit\n\
# If in force mode, this is the vector of desired forces/torques\n\
# to be regulated in (N) and (Nm)\n\
# If in impedance with force limit mode, this vector specifies the\n\
# magnitude of forces/torques (N and Nm) that the command will not exceed.\n\
float64[] force_command\n\
\n\
## Desired frame\n\
geometry_msgs/Pose interaction_frame\n\
string endpoint_name\n\
# True if impedance and force commands are defined in endpoint frame\n\
bool in_endpoint_frame\n\
\n\
# Set to true to disable damping during force control. Damping is used\n\
# to slow down robot motion during force control in free space.\n\
# Option included for SDK users to disable damping in force control\n\
bool disable_damping_in_force_control\n\
\n\
# Set to true to disable reference resetting. Reference resetting is\n\
# used when interaction parameters change, in order to avoid jumps/jerks.\n\
# Option included for SDK users to disable reference resetting if the\n\
# intention is to change interaction parameters.\n\
bool disable_reference_resetting\n\
\n\
## Mode Selection Parameters\n\
# The possible interaction control modes are:\n\
# Impedance mode: implements desired endpoint stiffness and damping.\n\
uint8 IMPEDANCE_MODE=1\n\
# Force mode: applies force/torque in the specified dimensions.\n\
uint8 FORCE_MODE=2\n\
# Impedance with force limit: impedance control while ensuring the commanded\n\
# forces/torques do not exceed force_command.\n\
uint8 IMPEDANCE_WITH_FORCE_LIMIT_MODE=3\n\
# Force with motion bounds: force control while ensuring the current\n\
# pose/velocities do not exceed forceMotionThreshold (currenetly defined in yaml)\n\
uint8 FORCE_WITH_MOTION_LIMIT_MODE=4\n\
\n\
# Specifies the interaction control mode for each Cartesian dimension (6)\n\
uint8[] interaction_control_mode\n\
\n\
# All 6 values in force and impedance parameter vectors have to be filled,\n\
# If a control mode is not used in a Cartesian dimension,\n\
# the corresponding parameters will be ignored.\n\
\n\
## Parameters for Constrained Zero-G Behaviors\n\
# Allow for arbitrary rotational displacements from the current orientation\n\
# for constrained zero-G. Setting 'rotations_for_constrained_zeroG = True'\n\
# will disable the rotational stiffness field which limits rotational\n\
# displacements to +/- 82.5 degree.\n\
# NOTE: it will be only enabled for a stationary reference orientation\n\
bool rotations_for_constrained_zeroG\n\
\n\
================================================================================\n\
MSG: intera_motion_msgs/TrackingOptions\n\
# Minimum trajectory tracking time rate:  (default = less than one)\n\
bool     use_min_time_rate\n\
float64  min_time_rate\n\
\n\
# Maximum trajectory tracking time rate:  (1.0 = real-time = default)\n\
bool     use_max_time_rate\n\
float64  max_time_rate\n\
\n\
# Angular error tolerance at final point on trajectory (rad)\n\
float64[] goal_joint_tolerance\n\
\n\
# Time for the controller to settle within joint tolerances at the goal (sec)\n\
bool     use_settling_time_at_goal\n\
float64  settling_time_at_goal\n\
\n\
================================================================================\n\
MSG: intera_motion_msgs/MotionCommandActionResult\n\
# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
\n\
Header header\n\
actionlib_msgs/GoalStatus status\n\
MotionCommandResult result\n\
\n\
================================================================================\n\
MSG: actionlib_msgs/GoalStatus\n\
GoalID goal_id\n\
uint8 status\n\
uint8 PENDING         = 0   # The goal has yet to be processed by the action server\n\
uint8 ACTIVE          = 1   # The goal is currently being processed by the action server\n\
uint8 PREEMPTED       = 2   # The goal received a cancel request after it started executing\n\
                            #   and has since completed its execution (Terminal State)\n\
uint8 SUCCEEDED       = 3   # The goal was achieved successfully by the action server (Terminal State)\n\
uint8 ABORTED         = 4   # The goal was aborted during execution by the action server due\n\
                            #    to some failure (Terminal State)\n\
uint8 REJECTED        = 5   # The goal was rejected by the action server without being processed,\n\
                            #    because the goal was unattainable or invalid (Terminal State)\n\
uint8 PREEMPTING      = 6   # The goal received a cancel request after it started executing\n\
                            #    and has not yet completed execution\n\
uint8 RECALLING       = 7   # The goal received a cancel request before it started executing,\n\
                            #    but the action server has not yet confirmed that the goal is canceled\n\
uint8 RECALLED        = 8   # The goal received a cancel request before it started executing\n\
                            #    and was successfully cancelled (Terminal State)\n\
uint8 LOST            = 9   # An action client can determine that a goal is LOST. This should not be\n\
                            #    sent over the wire by an action server\n\
\n\
#Allow for the user to associate a string with GoalStatus for debugging\n\
string text\n\
\n\
\n\
================================================================================\n\
MSG: intera_motion_msgs/MotionCommandResult\n\
# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
# result\n\
bool result\n\
\n\
string errorId\n\
string FAILED_TO_PARAMETERIZE=FAILED_TO_PARAMETERIZE\n\
string PLANNED_MOTION_COLLISION=PLANNED_MOTION_COLLISION\n\
string INVALID_TRAJECTORY_MESSAGE=INVALID_TRAJECTORY_MESSAGE\n\
string ENDPOINT_DOES_NOT_EXIST=ENDPOINT_DOES_NOT_EXIST\n\
string CARTESIAN_INTERPOLATION_FAILED=CARTESIAN_INTERPOLATION_FAILED\n\
string FINAL_POSE_NOT_WITHIN_TOLERANCE=FINAL_POSE_NOT_WITHIN_TOLERANCE\n\
string CONTROLLER_NOT_FOLLOWING=CONTROLLER_NOT_FOLLOWING\n\
string ZERO_G_ACTIVATED_DURING_TRAJECTORY=ZERO_G_ACTIVATED_DURING_TRAJECTORY\n\
string PLANNED_JOINT_ACCEL_LIMIT=PLANNED_JOINT_ACCEL_LIMIT\n\
\n\
TrajectoryAnalysis trajectory_analysis\n\
\n\
int32 last_successful_waypoint\n\
int32 HAVE_NOT_REACHED_FIRST_WAYPOINT=-1\n\
int32 GENERAL_TRAJECTORY_FAILURE=-2\n\
\n\
\n\
================================================================================\n\
MSG: intera_motion_msgs/TrajectoryAnalysis\n\
# The duration of the reference trajectory, as originally planned\n\
float64 planned_duration\n\
\n\
# The measured duration of the trajectory, as executed\n\
float64 measured_duration\n\
\n\
# Minimum commanded angle during trajectory for each joint\n\
float64[] min_angle_command\n\
\n\
# Maximum commanded angle during trajectory for each joint\n\
float64[] max_angle_command\n\
\n\
# Peak speed command = max(abs(reference velocity)) for each joint\n\
float64[] peak_speed_command\n\
\n\
# Peak accel command = max(abs(reference acceleration)) for each joint\n\
float64[] peak_accel_command\n\
\n\
# Peak jerk command = max(abs(reference jerk)) for each joint\n\
float64[] peak_jerk_command\n\
\n\
# Minimum trajectory time rate observed\n\
float64 min_time_rate\n\
\n\
# Maximium trajectory time rate observed\n\
float64 max_time_rate\n\
\n\
# Max joint position error = max(abs(position error)) for each joint\n\
float64[] max_position_error\n\
\n\
# Max joint velocity error = max(abs(velocity error)) for each joint\n\
float64[] max_velocity_error\n\
\n\
================================================================================\n\
MSG: intera_motion_msgs/MotionCommandActionFeedback\n\
# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
\n\
Header header\n\
actionlib_msgs/GoalStatus status\n\
MotionCommandFeedback feedback\n\
\n\
================================================================================\n\
MSG: intera_motion_msgs/MotionCommandFeedback\n\
# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
# feedback\n\
MotionStatus status\n\
\n\
\n\
================================================================================\n\
MSG: intera_motion_msgs/MotionStatus\n\
# motion status\n\
Header header\n\
string motion_status\n\
string current_trajectory\n\
uint32 current_waypoint\n\
uint32 motion_request\n\
\n\
# motion_status enum values:\n\
string MOTION_IDLE=idle\n\
string MOTION_PENDING=pending\n\
string MOTION_RUNNING=running\n\
string MOTION_STOPPING=stopping\n\
string MOTION_DONE=done\n\
string MOTION_PREEMPTED=preempted\n\
string MOTION_ERROR=error\n\
";
  }

  static const char* value(const ::intera_motion_msgs::MotionCommandAction_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::intera_motion_msgs::MotionCommandAction_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.action_goal);
      stream.next(m.action_result);
      stream.next(m.action_feedback);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct MotionCommandAction_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::intera_motion_msgs::MotionCommandAction_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::intera_motion_msgs::MotionCommandAction_<ContainerAllocator>& v)
  {
    s << indent << "action_goal: ";
    s << std::endl;
    Printer< ::intera_motion_msgs::MotionCommandActionGoal_<ContainerAllocator> >::stream(s, indent + "  ", v.action_goal);
    s << indent << "action_result: ";
    s << std::endl;
    Printer< ::intera_motion_msgs::MotionCommandActionResult_<ContainerAllocator> >::stream(s, indent + "  ", v.action_result);
    s << indent << "action_feedback: ";
    s << std::endl;
    Printer< ::intera_motion_msgs::MotionCommandActionFeedback_<ContainerAllocator> >::stream(s, indent + "  ", v.action_feedback);
  }
};

} // namespace message_operations
} // namespace ros

#endif // INTERA_MOTION_MSGS_MESSAGE_MOTIONCOMMANDACTION_H
