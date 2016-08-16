#include "simple_ros_control_main/simple_hardware_interface.h"
#include "communication_interface/communication_interface.h"

void SpHwInterface::fake_update()
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

}

void SpHwInterface::ethercat_update()
{
	// Substract home pos, so the act_curr_pos_ will be initialized as zero. 
	// This makes the ros control manager think the robot joints are at 0 degree.
	for(size_t i = 0; i < act_home_pos_.size(); i++)
		act_curr_pos_[i] -= act_home_pos_[i];

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
}

void SpHwInterface::uart_update()
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
}
