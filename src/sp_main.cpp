#include <simple_ros_control_main/simple_hardware_interface.h>
#include <iostream>

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
  else
	std::cout << "n_dof_ = " << n_dof_ << std::endl;

  if (!nh.getParam("/robot/joints", jnt_names_))
  {
	ROS_ERROR("Please specify joint names (joints).");
	return 1;
  }
  else
  {
	std::cout << "jnt names: ";
	for(int i = 0; i < jnt_names_.size(); i++)
	  std::cout << jnt_names_[i] << " ";
	std::cout << std::endl;
  }

  if (!nh.getParam("/robot/gear_ratios", gear_ratios_))
  {
	ROS_ERROR("Please specify gear ratios (gear_ratios).");
	return 1;
  }
  else
  {
	std::cout << "gear ratios: ";
	for(int i = 0; i < gear_ratios_.size(); i++)
	  std::cout << gear_ratios_[i] << " ";
	std::cout << std::endl;

  }

  if (!nh.getParam("/robot/update_freq", update_freq_))
  {
	ROS_ERROR("Please specify update frequency (update_freq).");
	return 1;
  }
  else
	std::cout << "update frequency = " << update_freq_ << std::endl;


  SpHwInterface robot(n_dof_, update_freq_, jnt_names_, gear_ratios_);
  controller_manager::ControllerManager cm(&robot, nh);

  // start loop
  ros::Rate rate(1.0 / robot.getPeriod().toSec());
  ros::AsyncSpinner spinner(1);
  spinner.start();

  while (ros::ok())
  {
     //robot.read();
	 robot.update();
     cm.update(robot.getTime(), robot.getPeriod());
     //robot.write();
	 
	 rate.sleep();
  }
  spinner.stop();

  return 0;
}
