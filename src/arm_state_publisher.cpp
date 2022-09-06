// ROS
#include <ros/ros.h>
// ROS Messages
#include <geometry_msgs/Pose.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
// Etc
#include <time.h> 
#include <eigen_conversions/eigen_msg.h>
// Custom
#include <move_group_interface.h> // Move_Group_Interface class


int main(int argc, char** argv)
{
  if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
  {
    ros::console::notifyLoggerLevelsChanged();
  }
  ros::init(argc, argv, "arm_state_publisher", ros::init_options::NoRosout);
  ros::NodeHandle n;

  std::string prefix = "/"; // / 꼭 넣기 
  if (argc > 1)
    prefix += argv[1];
  ROS_INFO("%s", prefix.c_str());

  ros::AsyncSpinner spinner(1);
  spinner.start();

  Move_Group_Interface mgi("arm", prefix);
  double timeout = 0.1;

  
  

  // Loop
  clock_t start, end;
  double result, manipulability;
  
  ros::Rate loop_rate(1000);
  start = clock(); // 측정 시작
  while (ros::ok()){
    
    mgi.kinematic_state->setJointGroupPositions(mgi.joint_model_group, mgi.current_joint_position);
    mgi.kinematic_state->enforceBounds(); // Make sure all state variables are within bounds and normalized.

    // check self collision
    mgi.publish_self_collision();

    // publish manipulabilty index
    mgi.publish_m_index();

    // publish manipulabilty ellipsoid
    mgi.publish_e_values();

    // publish task velocity
    mgi.publish_task_velocity();

    // publish current pose
    mgi.publish_current_pose_rpy();

    // // calculate manipulabilty
    // move_group_interface.kinematic_state->update(); // https://github.com/ros-planning/moveit/pull/188
    // move_group_interface.metrics.getManipulability(*move_group_interface.kinematic_state, move_group_interface.joint_model_group, manipulability, true);
    // std_msgs::Float64 m_index_msg;
    // m_index_msg.data = manipulability;
    // move_group_interface.m_index_pub.publish(m_index_msg);

    loop_rate.sleep();
    // end = clock(); // 측정 끝
    // result = (double)(end - start);
    // printf("time: %lf\n", result); //결과 출력
    // start = end;
  }

  ros::shutdown();
  return 0;
}