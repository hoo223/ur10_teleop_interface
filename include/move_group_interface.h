/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: */

// Standard
#include <time.h> 
// ROS
#include <ros/ros.h>
#include <tf/tf.h> // tf::Quaternion, tf::Matrix3x3
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h> // pointTFToMsg, quaternionTFToMsg // http://docs.ros.org/en/api/tf/html/c++/transform__datatypes_8h_source.html
// MoveIt
#include <moveit/move_group_interface/move_group_interface.h> // moveit::planning_interface::MoveGroup  // http://docs.ros.org/en/melodic/api/moveit_ros_planning_interface/html/classmoveit_1_1planning__interface_1_1MoveGroupInterface.html
#include <moveit/robot_model_loader/robot_model_loader.h> // robot_model_loader::RobotModelLoader
#include <moveit/robot_model/robot_model.h> // moveit::core::RobotModelPtr
#include <moveit/robot_state/robot_state.h> // robot_state::JointModelGroup
#include <moveit/kinematics_metrics/kinematics_metrics.h> // kinematics_metrics::KinematicsMetrics
#include <moveit/planning_scene/planning_scene.h> // planning_scene::PlanningScene
// Messages
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>
// Services
#include <ur10_teleop_interface/SolveIk.h>
// Etc
#include <eigen_conversions/eigen_msg.h>

class Move_Group_Interface
{
public:
  Move_Group_Interface(std::string group_name, std::string prefix)
  : PLANNING_GROUP(group_name),
    prefix(prefix),
    move_group(group_name),
    robot_model_loader(robot_model_loader::RobotModelLoader(prefix+"/robot_description")),
    kinematic_model(robot_model_loader.getModel()),
    planning_scene(kinematic_model),
    model(robot_model_loader.getModel()),
    state(*new moveit::core::RobotState(model)),
    metrics(model)
  {
    ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());
    kinematic_state = moveit::core::RobotStatePtr(new moveit::core::RobotState(kinematic_model));
    kinematic_state->setToDefaultValues();
    joint_model_group = kinematic_model->getJointModelGroup(PLANNING_GROUP);
    
    joint_names = joint_model_group->getVariableNames();
    current_joint_position.resize(6);
    current_joint_velocity.resize(6);
  
    // Subscriber
    arm_state_sub = n.subscribe<sensor_msgs::JointState>(prefix+"/joint_states", 10, boost::bind(&Move_Group_Interface::jointStateCallback, this, _1));
    target_pose_sub = n.subscribe<geometry_msgs::PoseStamped>(prefix+"/target_pose", 10, boost::bind(&Move_Group_Interface::targetPoseCallback, this, _1));

    // Publisher
    current_task_velocity_pub = n.advertise<std_msgs::Float64MultiArray>(prefix+"/task_velocity", 10); 
    current_pose_rpy_pub =  n.advertise<std_msgs::Float64MultiArray>(prefix+"/current_pose_rpy", 10); 
    current_pose_stamped_pub =  n.advertise<std_msgs::Float64MultiArray>(prefix+"/current_pose_stamped", 10); 
    m_index_pub = n.advertise<std_msgs::Float64>(prefix+"/m_index", 10); // manipulability index publisher
    eigen_value_pub = n.advertise<std_msgs::Float64MultiArray>(prefix+"/eigen_value", 10); // 
    self_collision_pub = n.advertise<std_msgs::Bool>(prefix+"/self_collision", 10);

    n.getParam(prefix+"/init_joint_states", pre_ik_result.data);
    continuity_threshold = 0.3;
  }

  // Method
  bool check_self_collision(void);
  bool solve_ik(geometry_msgs::Pose end_pose);
  bool check_solution_continuity(void);
  Eigen::MatrixXd getCurrentJacobian(void);
  Eigen::VectorXd getCurrentTaskVelocity(void);
  std_msgs::Float64MultiArray getCurrentPoseRPY(void);
  geometry_msgs::PoseStamped getCurrentPoseStamped(void);
  tf::StampedTransform listenTransform(void);
  void publish_m_index(void);
  void publish_e_values(void);
  void publish_self_collision(void);
  void publish_current_pose_rpy(void);
  void publish_current_pose_stamped(void);
  void publish_task_velocity(void);
  
  // Member
  std::string PLANNING_GROUP;
  std::string prefix;
  collision_detection::CollisionRequest collision_request;
  collision_detection::CollisionResult collision_result;
  std::vector<std::string> joint_names;
  std::vector<double> current_joint_position, current_joint_velocity;
  geometry_msgs::Pose target_pose, valid_target_pose;
  Eigen::Isometry3d pose_in;
  std::vector<double> joint_values;
  bool with_gripper;
  std_msgs::Float64MultiArray ik_result, pre_ik_result;
  double continuity_threshold;

  // MoveIt related
  moveit::planning_interface::MoveGroupInterface move_group;
  robot_state::JointModelGroup* joint_model_group; 
  robot_model_loader::RobotModelLoader robot_model_loader;
  moveit::core::RobotModelPtr kinematic_model;
  moveit::core::RobotStatePtr kinematic_state;
  planning_scene::PlanningScene planning_scene;
  const moveit::core::RobotModelConstPtr& model;
  const moveit::core::RobotState& state;
  kinematics_metrics::KinematicsMetrics metrics;

  // Handler
  ros::NodeHandle n;

  // Publisher & Subscriber
  ros::Subscriber arm_state_sub;
  ros::Subscriber target_pose_sub;  
  ros::Publisher current_task_velocity_pub;
  ros::Publisher current_pose_rpy_pub;
  ros::Publisher current_pose_stamped_pub;
  ros::Publisher m_index_pub;
  ros::Publisher eigen_value_pub;
  ros::Publisher self_collision_pub;

  // TF Listener
  tf::TransformListener listener;

  // Topic Callback
  void jointStateCallback(const sensor_msgs::JointStateConstPtr& joint_state);
  void targetPoseCallback(const geometry_msgs::PoseStampedConstPtr& target_pose);

  // Service Callback
  bool solve_ik_srv(ur10_teleop_interface::SolveIk::Request &req, ur10_teleop_interface::SolveIk::Response &res);
  
private:

};

bool Move_Group_Interface::solve_ik_srv(ur10_teleop_interface::SolveIk::Request &req, 
              ur10_teleop_interface::SolveIk::Response &res)
{
  // find ik
  kinematic_state->setJointGroupPositions(joint_model_group, current_joint_position);
  kinematic_state->enforceBounds();
  tf::poseMsgToEigen(req.end_pose, pose_in);
  bool found_ik = kinematic_state->setFromIK(joint_model_group, pose_in, 0.1);
  kinematic_state->update(); // https://github.com/ros-planning/moveit/pull/188
  
  // Now, we can print out the IK solution (if found):
  if (found_ik)
  {
    kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
    ROS_INFO("found IK solution");
    // publish result
    res.ik_result.data.clear();
    res.ik_result.data.push_back(joint_values[0]);
    res.ik_result.data.push_back(joint_values[1]);
    res.ik_result.data.push_back(joint_values[2]);
    res.ik_result.data.push_back(joint_values[3]);
    res.ik_result.data.push_back(joint_values[4]);
    res.ik_result.data.push_back(joint_values[5]);
    res.success = true;
  }
  else
  {
    res.success = false; 
    ROS_INFO("Did not find IK solution");
  }
  return true;
}

bool Move_Group_Interface::solve_ik(geometry_msgs::Pose end_pose)
{
  // find ik
  kinematic_state->setJointGroupPositions(joint_model_group, current_joint_position);
  kinematic_state->enforceBounds();
  tf::poseMsgToEigen(end_pose, pose_in);
  bool found_ik = kinematic_state->setFromIK(joint_model_group, pose_in, 0.1);
  kinematic_state->update(); // https://github.com/ros-planning/moveit/pull/188
  
  // Now, we can print out the IK solution (if found):
  if (found_ik)
  {
    kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
    //ROS_INFO("found IK solution");
    // publish result
    ik_result.data.clear();
    ik_result.data.push_back(joint_values[0]);
    ik_result.data.push_back(joint_values[1]);
    ik_result.data.push_back(joint_values[2]);
    ik_result.data.push_back(joint_values[3]);
    ik_result.data.push_back(joint_values[4]);
    ik_result.data.push_back(joint_values[5]);
    
    // Check solution continuity
    bool continuity = check_solution_continuity();
    if(!continuity){
      found_ik = false;
      ROS_INFO("IK solution is not continuous");
    }
  }
  else
  {
    ROS_INFO("Did not find IK solution");
  }
  return found_ik;
}

bool Move_Group_Interface::check_solution_continuity(void){
  bool solution_continuity = true;
  for(int i=0; i<joint_names.size(); i++){
    if (abs(ik_result.data[i]-pre_ik_result.data[i]) > continuity_threshold)
      solution_continuity = false;
  }
  return solution_continuity;
}

void Move_Group_Interface::jointStateCallback(const sensor_msgs::JointStateConstPtr& joint_state)
{
  // position
  current_joint_position[0] = joint_state->position[3]; // shoulder_pan_joint
	current_joint_position[1] = joint_state->position[2]; // shoulder_lift_joint
	current_joint_position[2] = joint_state->position[0]; // elbow joint
	current_joint_position[3] = joint_state->position[4]; // wrist_1_joint
	current_joint_position[4] = joint_state->position[5]; // wrist_2_joint
	current_joint_position[5] = joint_state->position[6]; // wrist_3_joint
  // velocity
  current_joint_velocity[0] = joint_state->velocity[3];
  current_joint_velocity[1] = joint_state->velocity[2];
  current_joint_velocity[2] = joint_state->velocity[0];
  current_joint_velocity[3] = joint_state->velocity[4];
  current_joint_velocity[4] = joint_state->velocity[5];
  current_joint_velocity[5] = joint_state->velocity[6];

  // ROS_INFO_STREAM("joint1: " << current_joint_position[0] << "\n"); 
  // ROS_INFO_STREAM("joint2: " << current_joint_position[1] << "\n"); 
  // ROS_INFO_STREAM("joint3: " << current_joint_position[2] << "\n"); 
  // ROS_INFO_STREAM("joint4: " << current_joint_position[3] << "\n"); 
  // ROS_INFO_STREAM("joint5: " << current_joint_position[4] << "\n"); 
  // ROS_INFO_STREAM("joint6: " << current_joint_position[5] << "\n"); 
}

void Move_Group_Interface::targetPoseCallback(const geometry_msgs::PoseStampedConstPtr& target_pose)
{
  //printf("%.3f, %.3f, %.3f", target_pose->position.x, target_pose->position.y, target_pose->position.z);
  //tf::poseMsgToEigen(*target_pose, pose_in);
  this->target_pose = target_pose->pose;
}

// https://github.com/ros-planning/moveit_tutorials/blob/master/doc/planning_scene/src/planning_scene_tutorial.cpp
bool Move_Group_Interface::check_self_collision(void)
{
  robot_state::RobotState& current_state = planning_scene.getCurrentStateNonConst();
  current_state.setJointGroupPositions(joint_model_group, current_joint_position);
  collision_result.clear();
  planning_scene.checkSelfCollision(collision_request, collision_result);
  // ROS_INFO_STREAM("Test 1: Current state is " << (collision_result.collision ? "in" : "not in") << " self collision");
  return collision_result.collision;
}

Eigen::MatrixXd Move_Group_Interface::getCurrentJacobian(void)
{
  Eigen::MatrixXd jacobian;
  jacobian = kinematic_state->getJacobian(joint_model_group);
  // ROS_INFO("Jacobian Matrix");
  // ROS_INFO("%f %f %f %f %f %f", jacobian(0, 0), jacobian(0, 1), jacobian(0, 2), jacobian(0, 3), jacobian(0, 4), jacobian(0, 5));
  // ROS_INFO("%f %f %f %f %f %f", jacobian(1, 0), jacobian(1, 1), jacobian(1, 2), jacobian(1, 3), jacobian(1, 4), jacobian(1, 5));
  // ROS_INFO("%f %f %f %f %f %f", jacobian(2, 0), jacobian(2, 1), jacobian(2, 2), jacobian(2, 3), jacobian(2, 4), jacobian(2, 5));
  // ROS_INFO("%f %f %f %f %f %f", jacobian(3, 0), jacobian(3, 1), jacobian(3, 2), jacobian(3, 3), jacobian(3, 4), jacobian(3, 5));
  // ROS_INFO("%f %f %f %f %f %f", jacobian(4, 0), jacobian(4, 1), jacobian(4, 2), jacobian(4, 3), jacobian(4, 4), jacobian(4, 5));
  // ROS_INFO("%f %f %f %f %f %f", jacobian(5, 0), jacobian(5, 1), jacobian(5, 2), jacobian(5, 3), jacobian(5, 4), jacobian(5, 5));
  return jacobian;
}

Eigen::VectorXd Move_Group_Interface::getCurrentTaskVelocity(void)
{
  Eigen::MatrixXd jacobian = getCurrentJacobian();
  Eigen::VectorXd joint_velocities(6);
  Eigen::VectorXd task_velocities(6);
  joint_velocities[0] = current_joint_velocity[0];
  joint_velocities[1] = current_joint_velocity[1];
  joint_velocities[2] = current_joint_velocity[2];
  joint_velocities[3] = current_joint_velocity[3];
  joint_velocities[4] = current_joint_velocity[4];
  joint_velocities[5] = current_joint_velocity[5];
  task_velocities = jacobian * joint_velocities;
  // ROS_INFO("%f %f %f %f %f %f", task_velocities(0), task_velocities(1), task_velocities(2), task_velocities(3), task_velocities(4), task_velocities(5));
  return task_velocities;
}

tf::StampedTransform Move_Group_Interface::listenTransform(void)
{
  tf::StampedTransform transform; // http://docs.ros.org/en/indigo/api/tf/html/c++/classtf_1_1Transform.html
  try{
    listener.lookupTransform(prefix+"/base_link", prefix+"/tool_gripper", ros::Time(0), transform);
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
  }
  return transform;
}

std_msgs::Float64MultiArray Move_Group_Interface::getCurrentPoseRPY(void)
{
  tf::StampedTransform transform = listenTransform();

  std_msgs::Float64MultiArray current_pose;
  double x, y, z, roll, pitch, yaw;
  tf::Vector3 position = transform.getOrigin();
  x = position.getX();
  y = position.getY();
  z = position.getZ();
  // https://gist.github.com/marcoarruda/f931232fe3490b7fa20dbb38da1195ac
  tf::Matrix3x3 m(transform.getRotation());
  m.getRPY(roll, pitch, yaw);
  // ROS_INFO("%f %f %f", roll, pitch, yaw);
  current_pose.data.push_back(x);
  current_pose.data.push_back(y);
  current_pose.data.push_back(z);
  current_pose.data.push_back(roll);
  current_pose.data.push_back(pitch);
  current_pose.data.push_back(yaw);
  return current_pose;
}

geometry_msgs::PoseStamped Move_Group_Interface::getCurrentPoseStamped(void)
{
  tf::StampedTransform transform = listenTransform();

  // PoseStamped version
  geometry_msgs::PoseStamped current_pose;
  geometry_msgs::Point p;
  geometry_msgs::Quaternion q;
  pointTFToMsg(transform.getOrigin(), p);
  quaternionTFToMsg(transform.getRotation(), q);
  current_pose.pose.position = p;
  current_pose.pose.orientation = q;
  return current_pose;
}

void Move_Group_Interface::publish_current_pose_rpy(void)
{
  current_pose_rpy_pub.publish(getCurrentPoseRPY());
}

void Move_Group_Interface::publish_current_pose_stamped(void)
{
  current_pose_stamped_pub.publish(getCurrentPoseStamped());
}

void Move_Group_Interface::publish_task_velocity(void)
{
  Eigen::VectorXd task_velocities = getCurrentTaskVelocity();
  std_msgs::Float64MultiArray task_velocity;
  task_velocity.data.push_back(task_velocities[0]);
  task_velocity.data.push_back(task_velocities[1]);
  task_velocity.data.push_back(task_velocities[2]);
  task_velocity.data.push_back(task_velocities[3]);
  task_velocity.data.push_back(task_velocities[4]);
  task_velocity.data.push_back(task_velocities[5]);
  current_task_velocity_pub.publish(task_velocity);
}

void Move_Group_Interface::publish_m_index(void)
{
  kinematic_state->update(); // https://github.com/ros-planning/moveit/pull/188
  double m_index;
  metrics.getManipulabilityIndex(*kinematic_state, joint_model_group, m_index);
  std_msgs::Float64 m_index_msg;
  m_index_msg.data = m_index;
  m_index_pub.publish(m_index_msg);
}

void Move_Group_Interface::publish_e_values(void)
{
  kinematic_state->update(); // https://github.com/ros-planning/moveit/pull/188
  Eigen::MatrixXcd eigen_values, eigen_vectors;
  std_msgs::Float64MultiArray ik_result_msg, e_values_msg;
  metrics.getManipulabilityEllipsoid(*kinematic_state, joint_model_group, eigen_values, eigen_vectors);
  e_values_msg.data.clear();
  e_values_msg.data.push_back(eigen_values(0).real());
  e_values_msg.data.push_back(eigen_values(1).real());
  e_values_msg.data.push_back(eigen_values(2).real());
  eigen_value_pub.publish(e_values_msg);
  // ROS_INFO_STREAM("e_value1: \n" << eigen_values(0) << "\n");  
}

void Move_Group_Interface::publish_self_collision(void)
{
  std_msgs::Bool self_collision_msg;
  self_collision_msg.data = check_self_collision();
  self_collision_pub.publish(self_collision_msg);
}