#include <sstream>
#include "sp_hardware_interface.h"

int count = 0;

SpHwInterface::SpHwInterface(
	unsigned int m_n_dof_, unsigned int m_update_freq_, 
	std::vector<std::string> m_jnt_names_, std::vector<float> m_gear_ratios_)
{
  // Initialize private members
  n_dof_ = m_n_dof_;
  update_freq_ = m_update_freq_ ;
  jnt_names_= m_jnt_names_;
  gear_ratios_= m_gear_ratios_; 

  // Cleanup
  jnt_curr_pos_.clear();
  jnt_curr_vel_.clear();
  jnt_curr_eff_.clear();
  jnt_cmd_pos_.clear();

  // Raw Data
  jnt_curr_pos_.resize(n_dof_);
  jnt_curr_vel_.resize(n_dof_);
  jnt_curr_eff_.resize(n_dof_);
  jnt_cmd_pos_.resize(n_dof_);

  // Hardware Interface
  for (size_t i = 0; i < n_dof_; ++i)
  {
    jnt_state_interface_.registerHandle(
        hardware_interface::JointStateHandle(jnt_names_[i], &jnt_curr_pos_[i], &jnt_curr_vel_[i], &jnt_curr_eff_[i]));

    jnt_pos_interface_.registerHandle(
        hardware_interface::JointHandle(jnt_state_interface_.getHandle(jnt_names_[i]), &jnt_cmd_pos_[i]));

    ROS_DEBUG_STREAM("Registered joint '" << jnt_names_[i] << "' in the PositionJointInterface.");
  }
  registerInterface(&jnt_state_interface_);
  registerInterface(&jnt_pos_interface_);

}

SpHwInterface::~SpHwInterface()
{
}

void SpHwInterface::update()
{

  for(size_t i = 0; i < n_dof_; i++)
    jnt_curr_pos_[i] = jnt_cmd_pos_[i];

	//std::cout << "update at " << std::setprecision(13) << ros::Time::now().toSec() << " s : " << std::endl;
  if(count % 100 ==0)
  {
	  std::cout << "write at " << std::setprecision(13) << ros::Time::now().toSec() << " s : " << std::endl;
	  for(size_t i = 0; i < n_dof_; i++)
		std::cout << jnt_names_[i] << ": "<< jnt_cmd_pos_[i] << std::endl;

	  std::cout << std::endl;
  }
  count ++;
}

ros::Time SpHwInterface::getTime() const 
{
    return ros::Time::now();
}

ros::Duration SpHwInterface::getPeriod() const 
{
    return ros::Duration(1 / update_freq_);
}

