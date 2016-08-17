#include <sstream>
#include "simple_ros_control_main/simple_hardware_interface.h"
#include "communication_interface/communication_interface.h"

long int count = 0;

SpHwInterface::SpHwInterface(
	unsigned int m_n_dof_, unsigned int m_update_freq_, std::string m_comm_type_, 
	std::vector<std::string> m_jnt_names_, std::vector<double> m_gear_ratios_)
{
  // Initialize private members
  n_dof_ = m_n_dof_;
  update_freq_ = m_update_freq_;
  comm_type_ = m_comm_type_;
  jnt_names_= m_jnt_names_;
  gear_ratios_= m_gear_ratios_; 

  // Initialize Gear Rations
  for(size_t i = 0; i < n_dof_; i++)
	sim_trans_.emplace_back(gear_ratios_[i], 0.0);

  // Initialize joint and actuator
  jnt_act_initialize();

  // Initialize hardware interface
  hardware_interface_initialize();

  // Wrap data
  jnt_act_data_wrap();

  // Initialize transmission interface
  transmission_interface_initialize();

  // Initialize communication interface
  communication_interface::init();

  if(comm_type_ == "ethercat")
  {
    act_home_pos_ = communication_interface::get_home_pos();
    act_curr_pos_ = communication_interface::get_home_pos();
	for(int i = 0; i < act_curr_pos_.size(); i++)
		std::cout << act_curr_pos_[i] << std::endl;
  }
}

SpHwInterface::~SpHwInterface()
{
}

void SpHwInterface::update()
{
  // Fake updating, just for demo.
  if(comm_type_ == "fake")
	fake_update();

  // Use ethercat
  if(comm_type_ == "ethercat")
	ethercat_update();

  // Use uart 
  if(comm_type_ == "uart")
	uart_update();

  count ++;
}


