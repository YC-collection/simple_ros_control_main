#include <sstream>
#include "simple_ros_control_main/simple_hardware_interface.h"
#include "communication_interface/communication_interface.h"

long int count = 0;

SpHwInterface::SpHwInterface(
	unsigned int m_n_dof_, unsigned int m_update_freq_,  
	std::string m_comm_type_, std::string m_control_type_,
	std::vector<std::string> m_jnt_names_, std::vector<double> m_gear_ratios_)
{
  // Initialize private members
  n_dof_ = m_n_dof_;
  update_freq_ = m_update_freq_;
  comm_type_ = m_comm_type_;
  control_type_ = m_control_type_;
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
  if(control_type_ != "fake")
  {
    communication_interface::init(comm_type_, n_dof_);

    if(comm_type_ == "ethercat")
    {
      act_home_pos_ = communication_interface::get_home_pos();
      act_curr_pos_ = communication_interface::get_home_pos();
	  for(int i = 0; i < act_curr_pos_.size(); i++)
		  std::cout << act_curr_pos_[i] << std::endl;
    }
  }
}

SpHwInterface::~SpHwInterface()
{}

void SpHwInterface::update_pp()
{

	// Substract home pos, so the act_curr_pos_ will be initialized as zero. 
	// This makes the ros control manager think the robot joints are at 0 degree.
	for(size_t i = 0; i < act_home_pos_.size(); i++)
		act_curr_pos_[i] -= act_home_pos_[i];
#if 1
	if(count % 100 ==0)
	  print_read_data_pos();
#endif

	// Transform actuator space to joint space to let ros controller knows the robot state, 
	// and then transform the new joint command back to actuator space command.
	act_to_jnt_state_.propagate();  // The ros control manager will know the joint space value
									// and calculate new commands (joint space) after this step.
	jnt_to_act_state_.propagate();

	// Update the actuator
	act_curr_pos_ = communication_interface::update_pp(act_cmd_pos_);

#if 1
	if(count % 100 ==0)
	  print_write_data_pos();
#endif

    count ++;
}

void SpHwInterface::update_pv()
{}

void SpHwInterface::update_vp()
{}

void SpHwInterface::update_vv()
{
#if 1
	if(count % 100 ==0)
	  print_read_data_vel();
#endif

	act_to_jnt_state_.propagate();  
	jnt_to_act_state_.propagate();

	// Update the actuator
	act_curr_vel_ = communication_interface::update_vv(act_cmd_vel_);

#if 1
	if(count % 100 ==0)
	  print_write_data_vel();
#endif

    count ++;
}

void SpHwInterface::update_fake()
{
	// Fake reading
	for(size_t i = 0; i< n_dof_; i++)
	act_curr_pos_[i] = act_cmd_pos_[i];

	act_to_jnt_state_.propagate();
	jnt_to_act_state_.propagate();

#if 1
	if(count % 100 ==0)
	  print_write_data_pos();
#endif

    count ++;
}

