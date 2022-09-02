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

  double m_index;
  Eigen::MatrixXcd eigen_values;
  Eigen::MatrixXcd eigen_vectors;

  // Loop
  clock_t start, end;
  double result, manipulability;
  std_msgs::Float64MultiArray ik_result_msg, e_values_msg;
  ros::Rate loop_rate(250);
  start = clock(); // 측정 시작
  while (ros::ok()){
    
    mgi.kinematic_state->setJointGroupPositions(mgi.joint_model_group, mgi.current_joint_values);
    mgi.kinematic_state->enforceBounds(); // Make sure all state variables are within bounds and normalized.

    // check self collision
    std_msgs::Bool self_collision_msg;
    self_collision_msg.data = mgi.check_self_collision();
    mgi.self_collision_pub.publish(self_collision_msg);

    // calculate manipulabilty index
    mgi.kinematic_state->update(); // https://github.com/ros-planning/moveit/pull/188
    mgi.metrics.getManipulabilityIndex(*mgi.kinematic_state, mgi.joint_model_group, m_index);
    std_msgs::Float64 m_index_msg;
    m_index_msg.data = m_index;
    mgi.m_index_pub.publish(m_index_msg);

    // // calculate manipulabilty
    // move_group_interface.kinematic_state->update(); // https://github.com/ros-planning/moveit/pull/188
    // move_group_interface.metrics.getManipulability(*move_group_interface.kinematic_state, move_group_interface.joint_model_group, manipulability, true);
    // std_msgs::Float64 m_index_msg;
    // m_index_msg.data = manipulability;
    // move_group_interface.m_index_pub.publish(m_index_msg);

    // calculate manipulabilty ellipsoid
    mgi.kinematic_state->update(); // https://github.com/ros-planning/moveit/pull/188
    mgi.metrics.getManipulabilityEllipsoid(*mgi.kinematic_state, mgi.joint_model_group, eigen_values, eigen_vectors);
    e_values_msg.data.clear();
    e_values_msg.data.push_back(eigen_values(0).real());
    e_values_msg.data.push_back(eigen_values(1).real());
    e_values_msg.data.push_back(eigen_values(2).real());
    mgi.eigen_value_pub.publish(e_values_msg);
    // ROS_INFO_STREAM("e_value1: \n" << eigen_values(0) << "\n");  


    loop_rate.sleep();
    // end = clock(); // 측정 끝
    // result = (double)(end - start);
    // printf("time: %lf\n", result); //결과 출력
    // start = end;
  }

  ros::shutdown();
  return 0;
}