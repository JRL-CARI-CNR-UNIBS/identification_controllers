#pragma once

#include <hardware_interface/joint_command_interface.h>
#include <thread>
#include <mutex>
#include <boost/graph/graph_concepts.hpp>
#include <ros/time.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <pluginlib/class_list_macros.h>

#include <subscription_notifier/subscription_notifier.h>
#include <cnr_controller_interface/cnr_joint_command_controller_interface.h>

namespace cnr
{
namespace control
{

/**
 * @brief The MultisineIdentificationController class
 */
class MultisineIdentificationController :
  public cnr::control::JointCommandController<hardware_interface::PosVelEffJointHandle,
                                                            hardware_interface::PosVelEffJointInterface>
{
public:
  bool doInit();
  bool doUpdate(const ros::Time& time, const ros::Duration& period);
  bool doStarting(const ros::Time& time);
  bool doStopping(const ros::Time& time);

protected:


  double m_carrier_frequency;
  double m_carrier_amplitude;
  double m_carrier_periods;
  double m_test_time;
  double m_warmup_time=5.0;
  std::map<double,std::pair<double,double>> m_harmonic_numbers;

  std::string m_active_joint;
  int m_active_joint_idx;
  double m_warming_time;
  double m_periods;
  ros::Time m_t0;
  rosdyn::VectorXd m_joint_center_position;

};


}  //  end namespace identification_controllers
}
