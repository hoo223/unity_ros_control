#include <../include/unity_ros_control/unity_hardware_interface.h>

int main(int argc, char** argv)
{
  // Set up ROS.
  ros::init(argc, argv, "ur_hardware_interface");
  ros::AsyncSpinner spinner(2);
  spinner.start();

  ros::NodeHandle nh;
  ros::NodeHandle nh_priv("~");

  UnityUR10 ur10;

  // Subscriber
  // ros::Subscriber unity_ur10_state_sub = nh.subscribe<sensor_msgs::JointState>("unity_ur10_joint_states", 100, boost::bind(&UnityUR10::stateCallback, &ur10, _1));

  // Service
  //ros::ServiceServer service = nh.advertiseService("/unity/change_to_vel_controller", changeToVelController);

  // Set up timers
  ros::Time timestamp, pre_timestamp;
  ros::Duration period;
  auto stopwatch_last = std::chrono::steady_clock::now();
  auto stopwatch_now = stopwatch_last;

  ROS_DEBUG_STREAM("initialized hw interface");
  controller_manager::ControllerManager cm(&ur10, nh);

  // Get current time and elapsed time since last read
  timestamp = ros::Time::now();
  stopwatch_now = std::chrono::steady_clock::now();
  period.fromSec(std::chrono::duration_cast<std::chrono::duration<double>>(stopwatch_now - stopwatch_last).count());
  stopwatch_last = stopwatch_now;

  double expected_cycle_time = 1.0 / 125.0;

  ros::Rate rate(100);

  while (ros::ok())
  {
    ur10.read(timestamp, period);

    // Get current time and elapsed time since last read
    pre_timestamp = timestamp;
    timestamp = ros::Time::now();
    stopwatch_now = std::chrono::steady_clock::now();
    period.fromSec(std::chrono::duration_cast<std::chrono::duration<double>>(stopwatch_now - stopwatch_last).count());
    stopwatch_last = stopwatch_now;

    cm.update(timestamp, period);

    ur10.write(timestamp, period);
     
    // if (period.toSec() > expected_cycle_time)
    // {
    //   ROS_WARN_STREAM("Could not keep cycle rate of " << expected_cycle_time * 1000 << "ms");
    //   ROS_WARN_STREAM("Actual cycle time:" << period.toNSec() / 1000000.0 << "ms");
    // }

    //ROS_INFO_STREAM(timestamp.toNSec() - pre_timestamp.toNSec());
    rate.sleep();
  }

  spinner.stop();
  ROS_INFO_STREAM_NAMED("hardware_interface", "Shutting down.");
  return 0;
}

// bool changeToVelController(std_srvs::Trigger::Request &req,
//                            std_srvs::Trigger::Response &res)
// {
//   std::vector< std::string > start_controllers;
//   std::vector< std::string > stop_controllers;
//   start_controllers.push_back("joint_group_velocity_controller");
//   stop_controllers.push_back("arm_controller");
//   cm.switchController(start_controllers, stop_controllers, 1);
//   res.success = true;
//   return true;
// }