#ifndef SIMPLE_HARDWARE_INTERFACE_H 
#define SIMPLE_HARDWARE_INTERFACE_H 

#include <vector>
#include <iostream>
#include <ros/ros.h>
#include <ros/time.h>
#include <ros/duration.h>
#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <transmission_interface/simple_transmission.h>
#include <transmission_interface/transmission_interface.h>

#define PI 3.1415926

extern long int count;

class SpHwInterface : public hardware_interface::RobotHW
{
public:
  SpHwInterface(unsigned int, unsigned int, std::string, std::vector<std::string>, std::vector<double>);
  ~SpHwInterface();

  void jnt_act_initialize(); 
  void hardware_interface_initialize();
  void transmission_interface_initialize();
  void jnt_act_data_wrap();
  void print_write_data_pos();
  void print_write_data_vel();
  void print_read_data_pos();
  void print_read_data_vel();
  void update();
  void fake_update();
  void ethercat_update();
  void uart_update();
  ros::Time getTime() const;
  ros::Duration getPeriod() const;

private:
  unsigned int n_dof_;
  unsigned int update_freq_;
  std::string comm_type_;

  std::vector<std::string> jnt_names_;
  std::vector<double> gear_ratios_;
  std::vector<double> act_home_pos_;

  // Hardware Interface
  hardware_interface::JointStateInterface    jnt_state_interface_;
  hardware_interface::PositionJointInterface jnt_pos_interface_;
  hardware_interface::VelocityJointInterface jnt_vel_interface_;

  // Transmission Interface
  transmission_interface::ActuatorToJointStateInterface act_to_jnt_state_;
  transmission_interface::JointToActuatorStateInterface jnt_to_act_state_;
  transmission_interface::JointToActuatorVelocityInterface jnt_to_act_vel_;

  // Transmissions
  std::vector<transmission_interface::SimpleTransmission> sim_trans_;

  // Actuator and joint space variables: wrappers around raw data
  std::vector<transmission_interface::JointData> jnt_state_data_;
  std::vector<transmission_interface::JointData> jnt_cmd_data_;

  std::vector<transmission_interface::ActuatorData> act_state_data_;
  std::vector<transmission_interface::ActuatorData> act_cmd_data_;

  // State Data
  std::vector<double> jnt_curr_pos_;
  std::vector<double> jnt_curr_vel_;
  std::vector<double> jnt_curr_eff_;
  std::vector<double> jnt_cmd_pos_;
  std::vector<double> jnt_cmd_vel_;
  std::vector<double> jnt_cmd_eff_;

  std::vector<double> act_curr_pos_; 
  std::vector<double> act_curr_vel_; 
  std::vector<double> act_curr_eff_; 
  std::vector<double> act_cmd_pos_; 
  std::vector<double> act_cmd_vel_; 
  std::vector<double> act_cmd_eff_; 
};

#endif 
