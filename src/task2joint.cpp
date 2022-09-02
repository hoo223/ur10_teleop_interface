// ROS
#include <ros/ros.h>
// ROS Messages
#include <geometry_msgs/Pose.h>
// Etc
#include <time.h> 
#include <eigen_conversions/eigen_msg.h>
// Custom
#include <move_group_interface.h> // Move_Group_Interface class

// Mode
const int INIT = 0;
const int TELEOP = 1;
const int TASK_CONTROL = 2;
const int JOINT_CONTROL = 3;
const int RL = 4;
const int MOVEIT = 5;
const int IDLE = 6;


int main(int argc, char** argv)
{
  if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
  {
    ros::console::notifyLoggerLevelsChanged();
  }
  ros::init(argc, argv, "task2joint", ros::init_options::NoRosout);
  ros::NodeHandle n;
  int mode;
  

  std::string prefix = "/"; // / 꼭 넣기 
  if (argc > 1)
    prefix += argv[1];

  n.getParam(prefix+"/mode", mode);
  ROS_INFO("%d", mode);

  ros::AsyncSpinner spinner(1);
  spinner.start();

  // move_group interface 
  Move_Group_Interface move_group_interface("arm", prefix);

  // IK service
  ros::ServiceServer ik_service = n.advertiseService("solve_ik", &Move_Group_Interface::solve_ik_srv, &move_group_interface);

  // ik result publisher
  ros::Publisher ik_result_pub;
  ik_result_pub = n.advertise<std_msgs::Float64MultiArray>(prefix+"/ik_result", 10);

  // Loop
  // clock_t start, end;
  // double result, manipulability;
  bool success = false;
  
  ros::Rate loop_rate(250);
  // start = clock(); // 측정 시작
  while (ros::ok()){
    n.getParam(prefix+"/mode", mode);
    if((mode == TELEOP) || (mode == TASK_CONTROL)){
      // ROS_INFO("mode %d", mode);
      success = move_group_interface.solve_ik(move_group_interface.target_pose);
      if(success)
        ik_result_pub.publish(move_group_interface.ik_result);
    }
    
    loop_rate.sleep();

    // end = clock(); // 측정 끝
    // result = (double)(end - start);
    // printf("time: %lf\n", result); //결과 출력
    // start = end;
  }

  ros::shutdown();
  return 0;
}