#include <sstream>
#include <simple_ros_control_main/simple_hardware_interface.h>
#include <communication_interface/communication_interface.h>

int count = 0;

SpHwInterface::SpHwInterface(
	unsigned int m_n_dof_, unsigned int m_update_freq_, 
	std::vector<std::string> m_jnt_names_, std::vector<double> m_gear_ratios_)
{
  // Initialize private members
  n_dof_ = m_n_dof_;
  update_freq_ = m_update_freq_ ;
  jnt_names_= m_jnt_names_;
  gear_ratios_= m_gear_ratios_; 

  // Initialize Gear Rations
  for(size_t i = 0; i < n_dof_; i++)
	sim_trans_.emplace_back(gear_ratios_[i], 0.0);

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

  // Hardware Interface
  for(size_t i = 0; i < n_dof_; i++)
  {
    jnt_state_interface_.registerHandle(
        hardware_interface::JointStateHandle(jnt_names_[i], &jnt_curr_pos_[i], &jnt_curr_vel_[i], &jnt_curr_eff_[i]));

    jnt_pos_interface_.registerHandle(
        hardware_interface::JointHandle(jnt_state_interface_.getHandle(jnt_names_[i]), &jnt_cmd_pos_[i]));

    //ROS_DEBUG_STREAM("Registered joint '" << jnt_names_[i] << "' in the PositionJointInterface.");
  }
  registerInterface(&jnt_state_interface_);
  registerInterface(&jnt_pos_interface_);


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

#if 0
  jnt_home_pos_ = communication_interface::get_curr_pos();
  jnt_curr_pos_ = communication_interface::get_curr_pos();
#endif
}

SpHwInterface::~SpHwInterface()
{
}

void SpHwInterface::update()
{
  // Fake reading
  for(size_t i = 0; i< n_dof_; i++)
	act_curr_pos_[i] = act_cmd_pos_[i];

  act_to_jnt_state_.propagate();
#if 0
  // Handle current position (for reading) 
  for(size_t i = 0; i < n_dof_; i++)
    jnt_curr_pos_[i] = jnt_cmd_pos_[i];
#endif

  jnt_to_act_state_.propagate();
#if 1
  if(count % 100 ==0)
  {
	  std::cout << "write at " << std::setprecision(13) << ros::Time::now().toSec() << " s : " << std::endl;
	  for(size_t i = 0; i < n_dof_; i++)
		std::cout << jnt_names_[i] << ": joint = "<< jnt_cmd_pos_[i]
								   << "; actuator = " << act_cmd_pos_[i] << std::endl;
	  std::cout << std::endl;
  }
  count ++;
#endif
}

ros::Time SpHwInterface::getTime() const 
{
    return ros::Time::now();
}

ros::Duration SpHwInterface::getPeriod() const 
{
    return ros::Duration(0.001);
}
