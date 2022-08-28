#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_srvs/Trigger.h>

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

#include <iostream>

class UnityUR10 : public hardware_interface::RobotHW
{
public:
  UnityUR10();

  void read(const ros::Time& time, const ros::Duration& period);
  void write(const ros::Time& time, const ros::Duration& period);
  void stateCallback(const sensor_msgs::JointStateConstPtr& msg);
  void doSwitch(const std::list<hardware_interface::ControllerInfo>& start_list,
                const std::list<hardware_interface::ControllerInfo>& stop_list);
  bool checkControllerClaims(const std::set<std::string>& claimed_resources);

private:
  hardware_interface::JointStateInterface js_interface_;
  hardware_interface::PositionJointInterface pj_interface_;
  hardware_interface::VelocityJointInterface vj_interface_;
  std::vector<double> joint_position_command_;
  std::vector<double> joint_velocity_command_;
  std::vector<double> joint_positions_;
  std::vector<double> joint_velocities_;
  std::vector<double> unity_joint_positions_;
  std::vector<double> unity_joint_velocities_;
  std::vector<double> joint_efforts_;
  std::vector<std::string> joint_names_;
  bool position_controller_running_;
  bool velocity_controller_running_;

  ros::NodeHandle nh;
  ros::Publisher position_command_pub;
  ros::Publisher velocity_command_pub;
  ros::Subscriber unity_ur10_state_sub;
  std_msgs::Float64MultiArray command;
  std::string con_type;
};

UnityUR10::UnityUR10()
: joint_position_command_({ 0, 0, 0, 0, 0, 0, 0})
, joint_velocity_command_({ 0, 0, 0, 0, 0, 0, 0})
, joint_positions_{ { 0, 0, 0, 0, 0, 0, 0 } } 
, joint_velocities_{ { 0, 0, 0, 0, 0, 0, 0 } }
, joint_efforts_{ { 0, 0, 0, 0, 0, 0, 0 } }
, joint_names_( {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint", "robotiq_85_left_knuckle_joint"} ) 
, position_controller_running_(false)
, velocity_controller_running_(false)
{ 
  // Create ros_control interfaces
  for (std::size_t i = 0; i < joint_positions_.size(); ++i)
  {
    ROS_DEBUG_STREAM("Registering handles for joint " << joint_names_[i]);
    // Create joint state interface for all joints
    js_interface_.registerHandle(hardware_interface::JointStateHandle(joint_names_[i], &joint_positions_[i],
                                                                      &joint_velocities_[i], &joint_efforts_[i]));
 
    pj_interface_.registerHandle(
          hardware_interface::JointHandle(js_interface_.getHandle(joint_names_[i]), &joint_position_command_[i]));

    vj_interface_.registerHandle(
          hardware_interface::JointHandle(js_interface_.getHandle(joint_names_[i]), &joint_velocity_command_[i]));

    // // Get controller type
    // nh.getParam("controller_type", con_type);
    // ROS_INFO("%s", con_type.c_str());

    // if(!con_type.compare("pos")){
    //   // Create joint position control interface
    //   ROS_INFO("Position Interface!");
    //   pj_interface_.registerHandle(
    //       hardware_interface::JointHandle(js_interface_.getHandle(joint_names_[i]), &joint_position_command_[i]));
    // }
    // else if(!con_type.compare("vel")){
    //   // Create joint velocity control interface
    //   ROS_INFO("Velocity Interface!");
    //   vj_interface_.registerHandle(
    //       hardware_interface::JointHandle(js_interface_.getHandle(joint_names_[i]), &joint_velocity_command_[i]));
    // }
    // else{
    //   ROS_INFO("No pos or vel type controller");
    // }
  }

  registerInterface(&js_interface_);
  registerInterface(&pj_interface_);
  registerInterface(&vj_interface_);

  position_command_pub = nh.advertise<std_msgs::Float64MultiArray>("position_command", 100);
  velocity_command_pub = nh.advertise<std_msgs::Float64MultiArray>("velocity_command", 100);
  unity_ur10_state_sub = nh.subscribe<sensor_msgs::JointState>("unity_ur10_joint_states", 100, boost::bind(&UnityUR10::stateCallback, this, _1));
}

void UnityUR10::read(const ros::Time& time, const ros::Duration& period)
{
  joint_positions_ = unity_joint_positions_;
  joint_velocities_ = unity_joint_velocities_;
}

void UnityUR10::write(const ros::Time& time, const ros::Duration& period)
{
  // ROS_INFO_STREAM("pos: " << position_controller_running_ << ", vel: " << velocity_controller_running_);
  if (position_controller_running_)
  {
    command.data = joint_position_command_;
    position_command_pub.publish(command);
  }
  if (velocity_controller_running_)
  {
    command.data = joint_velocity_command_;
    velocity_command_pub.publish(command);
  }
}

void UnityUR10::stateCallback(const sensor_msgs::JointStateConstPtr& msg)
{
  unity_joint_positions_ = msg->position;
  unity_joint_velocities_ = msg->velocity;
  // ROS_INFO_STREAM("joint1: " << joint_positions_[0] << "\n");
  // ROS_INFO_STREAM("joint2: " << joint_positions_[1] << "\n");
  // ROS_INFO_STREAM("joint3: " << joint_positions_[2] << "\n");
  // ROS_INFO_STREAM("joint4: " << joint_positions_[3] << "\n");
  // ROS_INFO_STREAM("joint5: " << joint_positions_[4] << "\n");
  // ROS_INFO_STREAM("joint6: " << joint_positions_[5] << "\n");
}

void UnityUR10::doSwitch(const std::list<hardware_interface::ControllerInfo>& start_list,
              const std::list<hardware_interface::ControllerInfo>& stop_list)
{
  ROS_INFO("switch");
  for (auto& controller_it : stop_list)
  {
    for (auto& resource_it : controller_it.claimed_resources)
    {
      if (checkControllerClaims(resource_it.resources))
      {
        if (resource_it.hardware_interface == "ur_controllers::ScaledPositionJointInterface")
        {
          position_controller_running_ = false;
        }
        if (resource_it.hardware_interface == "hardware_interface::PositionJointInterface")
        {
          position_controller_running_ = false;
        }
        if (resource_it.hardware_interface == "ur_controllers::ScaledVelocityJointInterface")
        {
          velocity_controller_running_ = false;
        }
        if (resource_it.hardware_interface == "hardware_interface::VelocityJointInterface")
        {
          velocity_controller_running_ = false;
        }
      }
    }
  }
  for (auto& controller_it : start_list)
  {
    for (auto& resource_it : controller_it.claimed_resources)
    {
      if (checkControllerClaims(resource_it.resources))
      {
        if (resource_it.hardware_interface == "ur_controllers::ScaledPositionJointInterface")
        {
          position_controller_running_ = true;
        }
        if (resource_it.hardware_interface == "hardware_interface::PositionJointInterface")
        {
          position_controller_running_ = true;
        }
        if (resource_it.hardware_interface == "ur_controllers::ScaledVelocityJointInterface")
        {
          velocity_controller_running_ = true;
        }
        if (resource_it.hardware_interface == "hardware_interface::VelocityJointInterface")
        {
          velocity_controller_running_ = true;
        }
      }
    }
  }
}

bool UnityUR10::checkControllerClaims(const std::set<std::string>& claimed_resources)
{
  for (const std::string& it : joint_names_)
  {
    for (const std::string& jt : claimed_resources)
    {
      if (it == jt)
      {
        return true;
      }
    }
  }
  return false;
}