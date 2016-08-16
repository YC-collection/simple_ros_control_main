#include "simple_ros_control_main/simple_hardware_interface.h"

void SpHwInterface::jnt_act_initialize()
{
  // Joint Cleanup
  jnt_curr_pos_.clear();
  jnt_curr_vel_.clear();
  jnt_curr_eff_.clear();
  jnt_cmd_pos_.clear();
  jnt_cmd_vel_.clear();
  jnt_cmd_eff_.clear();

  jnt_state_data_.clear();
  jnt_cmd_data_.clear();

  // Actuator Cleanup
  act_curr_pos_.clear();
  act_curr_vel_.clear();
  act_curr_eff_.clear();
  act_cmd_pos_.clear();
  act_cmd_vel_.clear();
  act_cmd_eff_.clear();

  act_state_data_.clear();
  act_cmd_data_.clear();

  // Joint Resize 
  jnt_curr_pos_.resize(n_dof_);
  jnt_curr_vel_.resize(n_dof_);
  jnt_curr_eff_.resize(n_dof_);
  jnt_cmd_pos_.resize(n_dof_);
  jnt_cmd_vel_.resize(n_dof_);
  jnt_cmd_eff_.resize(n_dof_);

  jnt_state_data_.resize(n_dof_);
  jnt_cmd_data_.resize(n_dof_);

  // Actuator Resize 
  act_curr_pos_.resize(n_dof_);
  act_curr_vel_.resize(n_dof_);
  act_curr_eff_.resize(n_dof_);
  act_cmd_pos_.resize(n_dof_);
  act_cmd_vel_.resize(n_dof_);
  act_cmd_eff_.resize(n_dof_);

  act_state_data_.resize(n_dof_);
  act_cmd_data_.resize(n_dof_);
}

void SpHwInterface::hardware_interface_initialize()
{
  // Hardware Interface
  for(size_t i = 0; i < n_dof_; i++)
  {
    jnt_state_interface_.registerHandle(
        hardware_interface::JointStateHandle(jnt_names_[i], &jnt_curr_pos_[i], &jnt_curr_vel_[i], &jnt_curr_eff_[i]));

    jnt_pos_interface_.registerHandle(
        hardware_interface::JointHandle(jnt_state_interface_.getHandle(jnt_names_[i]), &jnt_cmd_pos_[i]));

    jnt_vel_interface_.registerHandle(
        hardware_interface::JointHandle(jnt_state_interface_.getHandle(jnt_names_[i]), &jnt_cmd_vel_[i]));

    //ROS_DEBUG_STREAM("Registered joint '" << jnt_names_[i] << "' in the PositionJointInterface.");
  }
  registerInterface(&jnt_state_interface_);
  registerInterface(&jnt_pos_interface_);
  registerInterface(&jnt_vel_interface_);
}

void SpHwInterface::transmission_interface_initialize()
{
  // Transmission Interface
  for(size_t i = 0; i < n_dof_; i++)
  {
	std::stringstream ss_temp("");
	ss_temp << "sim_trans" << i;

	act_to_jnt_state_.registerHandle(
		transmission_interface::ActuatorToJointStateHandle(ss_temp.str(), &sim_trans_[i], act_state_data_[i], jnt_state_data_[i]));

	jnt_to_act_state_.registerHandle(
		transmission_interface::JointToActuatorStateHandle(ss_temp.str(), &sim_trans_[i], act_cmd_data_[i], jnt_cmd_data_[i]));
  }
}

void SpHwInterface::jnt_act_data_wrap()
{
  // Joint data and actuator data
  for(size_t i = 0; i < n_dof_; i++)
  {
	jnt_state_data_[i].position.push_back(&jnt_curr_pos_[i]);
	jnt_state_data_[i].velocity.push_back(&jnt_curr_vel_[i]);
	jnt_state_data_[i].effort.push_back(&jnt_curr_eff_[i]);

	jnt_cmd_data_[i].position.push_back(&jnt_cmd_pos_[i]);
	jnt_cmd_data_[i].velocity.push_back(&jnt_cmd_vel_[i]);
	jnt_cmd_data_[i].effort.push_back(&jnt_cmd_eff_[i]);


	act_state_data_[i].position.push_back(&act_curr_pos_[i]);
	act_state_data_[i].velocity.push_back(&act_curr_vel_[i]);
	act_state_data_[i].effort.push_back(&act_curr_eff_[i]);
	
	act_cmd_data_[i].position.push_back(&act_cmd_pos_[i]);
	act_cmd_data_[i].velocity.push_back(&act_cmd_vel_[i]);
	act_cmd_data_[i].effort.push_back(&act_cmd_eff_[i]);
  }
}
