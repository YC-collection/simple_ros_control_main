#include "sp_hardware_interface.h"
#include <iostream>
#include <string>
#include <vector>

int main(int argc, char **argv)
{
  // initialize ros
  ros::init(argc, argv, "simple_ros_control");
  ros::NodeHandle nh;


  int n_dof_, update_freq_;
  std::vector<std::string> jnt_names_;
  std::vector<float> gear_ratios_;

  if (!nh.getParam("/robot/n_dof", n_dof_))
  {
	ROS_ERROR("Please specify the number of the degree of freedom (n_dof).");
	return 1;
  }

  if (!nh.getParam("/robot/joints", jnt_names_))
  {
	ROS_ERROR("Please specify joint names (joints).");
	return 1;
  }

  if (!nh.getParam("/robot/gear_ratios", gear_ratios_))
  {
	ROS_ERROR("Please specify gear ratios (gear_ratios).");
	return 1;
  }

  if (!nh.getParam("/robot/update_freq", update_freq_))
  {
	ROS_ERROR("Please specify update frequency (update_freq).");
	return 1;
  }

  SpHwInterface robot(n_dof_, update_freq_, jnt_names_, gear_ratios_);

#if 1
  controller_manager::ControllerManager cm(&robot, nh);

  // start loop
  //ros::Rate rate(1.0 / robot.getPeriod().toSec());
  ros::Rate rate(update_freq_);
  ros::AsyncSpinner spinner(1);
  spinner.start();
#endif

#if 1
  while (ros::ok())
  {
	 robot.update();
     cm.update(robot.getTime(), robot.getPeriod());
	 
	 rate.sleep();
  }
  spinner.stop();
#endif

  return 0;
}
