#include <rtt/TaskContext.hpp>
#include <rtt_rosparam/rosparam.h>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>
#include <rtt_rosclock/rtt_rosclock.h>
#include <boost/thread.hpp>

#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <simple_ros_control_main/simple_hardware_interface.h>

using namespace RTT;

class SpRosControl : public RTT::TaskContext{

  private:
    int n_dof_, update_freq_;
    std::string comm_type_, control_type_;
    std::vector<std::string> jnt_names_;
    std::vector<double> gear_ratios_;

    boost::thread non_rt_ros_queue_thread_;
    boost::shared_ptr<ros::NodeHandle> non_rt_ros_nh_;
    boost::shared_ptr<SpHwInterface> sp_hw_interface_;
    boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;
    ros::CallbackQueue non_rt_ros_queue_;
    ros::Time last_update_time_;

  public:
    SpRosControl(const std::string& name):TaskContext(name)
    {
      this->addProperty("robot/n_dof", n_dof_);
      this->addProperty("robot/joints", jnt_names_);
      this->addProperty("robot/gear_ratios", gear_ratios_);
      this->addProperty("robot/update_freq", update_freq_);
      this->addProperty("robot/control_type", control_type_);
      this->addProperty("robot/comm_type", comm_type_);
    }

    ~SpRosControl()
    {}

  private:
    bool configureHook()
    {
      bool all_params_found = true;

      non_rt_ros_nh_.reset(new ros::NodeHandle(""));
      non_rt_ros_nh_->setCallbackQueue(&non_rt_ros_queue_);
      this->non_rt_ros_queue_thread_ = boost::thread( boost::bind(&SpRosControl::serviceNonRtRosQueue,this ) );


      // Get the rosparam service requester
      boost::shared_ptr<rtt_rosparam::ROSParam> rosparam =
          this->getProvider<rtt_rosparam::ROSParam>("rosparam");

      if(rosparam) 
      {
        if (!rosparam->getAbsolute("robot/n_dof"))
        {
          ROS_ERROR("Please specify the number of the degree of freedom (n_dof).");
          all_params_found = false;
        }
        else
          std::cout << "n_dof_ = " << n_dof_ << std::endl;

        if (!rosparam->getAbsolute("robot/joints"))
        {
          ROS_ERROR("Please specify joint names (joints).");
          all_params_found = false;
        }
        else
        {
          std::cout << "jnt names: ";
          for(int i = 0; i < jnt_names_.size(); i++)
            std::cout << jnt_names_[i] << " ";
          std::cout << std::endl;
        }

        if (!rosparam->getAbsolute("robot/gear_ratios"))
        {
          ROS_ERROR("Please specify gear ratios (gear_ratios).");
          all_params_found = false;
        }
        else
        {
          std::cout << "gear ratios: ";
          for(int i = 0; i < gear_ratios_.size(); i++)
            std::cout << gear_ratios_[i] << " ";
          std::cout << std::endl;
        }

        if (!rosparam->getAbsolute("robot/update_freq"))
        {
          ROS_ERROR("Please specify update frequency (update_freq).");
          all_params_found = false;
        }
        else
          std::cout << "update frequency = " << update_freq_ << std::endl;


        if (!rosparam->getAbsolute("robot/control_type"))
        {
          ROS_ERROR("Please specify the control type (control_type).");
          all_params_found = false;
        }
        else
          std::cout << "control_type_ = " << control_type_ << std::endl;


        if (!rosparam->getAbsolute("robot/comm_type"))
        {
          ROS_ERROR("Please specify the communication type (comm_type).");
          all_params_found = false;
        }
        else
          std::cout << "comm_type_ = " << comm_type_ << std::endl;
      }
      else
      {
        ROS_ERROR("Cannot find parameter profider.");
        return false;
      }

      sp_hw_interface_.reset(new SpHwInterface(n_dof_, update_freq_, comm_type_, control_type_, jnt_names_, gear_ratios_));
      controller_manager_.reset(new controller_manager::ControllerManager(sp_hw_interface_.get(), *non_rt_ros_nh_));

      last_update_time_ = rtt_rosclock::rtt_now();
      return all_params_found;
    }

    bool startHook()
    {
    }

    void updateHook()
    {

      // Get current system time (for timestamps of ROS messages)
      ros::Time now(rtt_rosclock::host_now());

      // Get guaranteed monotonic time for period computation
      ros::Time now_monotonic(rtt_rosclock::rtt_now());

      ros::Duration period(now_monotonic - last_update_time_);
      last_update_time_ = now_monotonic;

      if(control_type_ == "pp")
        sp_hw_interface_->update_pp();
      else if(control_type_ == "pv")
        sp_hw_interface_->update_pv();
      else if(control_type_ == "vp")
        sp_hw_interface_->update_vp();
      else if(control_type_ == "vv")
        sp_hw_interface_->update_vv();
      else if(control_type_ == "fake")
        sp_hw_interface_->update_fake();
      else
        ROS_ERROR("Invalid control type. Please chose one of [pp, pv, vp, vv].");

      controller_manager_->update(now, period);
      log(Info) << "SpRosControl update!" << endlog();
    }

    void stopHook()
    {
    }

    void cleanupHook()
    {
      non_rt_ros_nh_->shutdown();
      non_rt_ros_queue_thread_.join();
    }

    void serviceNonRtRosQueue()
    {
      static const double timeout = 0.001;
      while (this->non_rt_ros_nh_->ok())
        this->non_rt_ros_queue_.callAvailable(ros::WallDuration(timeout));
    }
};
ORO_CREATE_COMPONENT(SpRosControl)
