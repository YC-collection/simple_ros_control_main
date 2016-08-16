#include "simple_ros_control_main/simple_hardware_interface.h"

void SpHwInterface::print_write_data_pos()
{
  std::cout << "write at " << std::setprecision(13) << ros::Time::now().toSec() << " s : " << std::endl;
  for(size_t i = 0; i < n_dof_; i++)
	std::cout << jnt_names_[i] << ": joint_pos = "<< jnt_cmd_pos_[i]
							   << "; actuator_pos = " << act_cmd_pos_[i] << std::endl;
  std::cout << std::endl;
}


void SpHwInterface::print_write_data_vel()
{
  std::cout << "write at " << std::setprecision(13) << ros::Time::now().toSec() << " s : " << std::endl;
  for(size_t i = 0; i < n_dof_; i++)
	std::cout << jnt_names_[i] << ": joint_vel = "<< jnt_cmd_vel_[i]
							   << "; actuator_vel = " << act_cmd_vel_[i] << std::endl;
  std::cout << std::endl;
}


void SpHwInterface::print_read_data_pos()
{}


void SpHwInterface::print_read_data_vel()
{
  std::cout << "read at " << std::setprecision(13) << ros::Time::now().toSec() << " s : " << std::endl;
  for(size_t i = 0; i < n_dof_; i++)
	std::cout << jnt_names_[i] << ": joint_vel = "<< jnt_curr_vel_[i]
							   << "; actuator_vel = " << act_curr_vel_[i] << std::endl;
  std::cout << std::endl;
}


ros::Time SpHwInterface::getTime() const 
{
    return ros::Time::now();
}


ros::Duration SpHwInterface::getPeriod() const 
{
    return ros::Duration(0.001);
}
