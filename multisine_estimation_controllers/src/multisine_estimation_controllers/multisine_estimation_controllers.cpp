#include <multisine_estimation_controllers/multisine_estimation_controllers.h>

#include <ros/ros.h>
#include <ros/time.h>
#include <eigen_matrix_utils/overloads.h>
#include <cnr_controller_interface/utils/utils.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(cnr::control::MultisineIdentificationController, controller_interface::ControllerBase)

namespace cnr
{
  namespace control
{

//!
//! \brief MultisineIdentificationController::doInit
//! \return
//!
bool MultisineIdentificationController::doInit()
{
  CNR_TRACE_START(m_logger);
  if(!getControllerNh().getParam("active_joint", m_active_joint))
  {
    CNR_RETURN_FALSE(m_logger,"The param '"+getControllerNamespace()+"/active_joint' does not exist");
  }

  std::vector<std::string> js=jointNames();
  std::vector<std::string>::iterator it = std::find(js.begin(),js.end(),m_active_joint);
  if (it==js.end())
  {
    CNR_RETURN_FALSE(m_logger,"active_joint "+m_active_joint+" is not in the list of joints");
  }
  m_active_joint_idx=it-js.begin();

  if(!getControllerNh().getParam("carrier_frequency", m_carrier_frequency))
  {
    CNR_RETURN_FALSE(m_logger,"The param '"+getControllerNamespace()+"/carrier_frequency' does not exist");
  }
  if(!getControllerNh().getParam("carrier_amplitude", m_carrier_amplitude))
  {
    CNR_RETURN_FALSE(m_logger,"The param '"+getControllerNamespace()+"/carrier_amplitude' does not exist");
  }
  if(!getControllerNh().getParam("carrier_periods", m_carrier_periods))
  {
    CNR_RETURN_FALSE(m_logger,"The param '"+getControllerNamespace()+"/carrier_periods' does not exist");
  }
  m_carrier_periods=std::ceil(m_carrier_periods);
  if (m_carrier_periods<1.0)
  {
    CNR_RETURN_FALSE(m_logger,"The param '"+getControllerNamespace()+"/carrier_periods' should be greater than 1.0");
  }
  m_test_time=m_carrier_periods*2.0*M_PI/m_carrier_frequency;

  if(!getControllerNh().getParam("warmup_time", m_warmup_time))
  {
    CNR_RETURN_FALSE(m_logger,"The param '"+getControllerNamespace()+"/warmup_time' does not exist");
  }


  std::vector<double> frequency;
  if(!getControllerNh().getParam("frequency", frequency))
  {
    CNR_RETURN_FALSE(m_logger,"The param '"+getControllerNamespace()+"/frequency' does not exist");
  }

  std::vector<double> amplitude_cos;
  if(!getControllerNh().getParam("amplitude_cos", amplitude_cos))
  {
    CNR_RETURN_FALSE(m_logger,"The param '"+getControllerNamespace()+"/amplitude_cos' does not exist");
  }
  if (amplitude_cos.size()!=frequency.size())
  {
    CNR_RETURN_FALSE(m_logger,"amplitude_con and frequency have wrong dimensions");
  }

  std::vector<double> amplitude_sin;
  if(!getControllerNh().getParam("amplitude_sin", amplitude_sin))
  {
    CNR_RETURN_FALSE(m_logger,"The param '"+getControllerNamespace()+"/amplitude_sin' does not exist");
  }
  if (amplitude_sin.size()!=frequency.size())
  {
    CNR_RETURN_FALSE(m_logger,"amplitude_sin and frequency have wrong dimensions");
  }

  m_harmonic_numbers.clear();
  for (size_t idx=0;idx<frequency.size();idx++)
  {
    double freq=std::round(frequency.at(idx)/m_carrier_frequency);
    std::pair<double,double> p(amplitude_cos.at(idx),amplitude_sin.at(idx));
    std::pair<double,std::pair<double,double>> freq_pair(freq,p);
    m_harmonic_numbers.insert(freq_pair);
  }



  setPriority(Q_PRIORITY);

  m_joint_center_position.resize(getPosition().size());

  CNR_DEBUG(m_logger, "Controller ' "+getControllerNamespace()+"' controls the following joint: "
                     + cnr::control::to_string(jointNames()));
  CNR_RETURN_TRUE(m_logger);
}



//!
//! \brief MultisineIdentificationController::doStarting
//! \return
//!
bool MultisineIdentificationController::doStarting(const ros::Time& /*time*/)
{
  CNR_TRACE_START(m_logger);



  std::vector<double> frequency;
  if(!getControllerNh().getParam("frequency", frequency))
  {
    CNR_RETURN_FALSE(m_logger,"The param '"+getControllerNamespace()+"/frequency' does not exist");
  }

  std::vector<double> amplitude_cos;
  if(!getControllerNh().getParam("amplitude_cos", amplitude_cos))
  {
    CNR_RETURN_FALSE(m_logger,"The param '"+getControllerNamespace()+"/amplitude_cos' does not exist");
  }
  if (amplitude_cos.size()!=frequency.size())
  {
    CNR_RETURN_FALSE(m_logger,"amplitude_con and frequency have wrong dimensions");
  }

  std::vector<double> amplitude_sin;
  if(!getControllerNh().getParam("amplitude_sin", amplitude_sin))
  {
    CNR_RETURN_FALSE(m_logger,"The param '"+getControllerNamespace()+"/amplitude_sin' does not exist");
  }
  if (amplitude_sin.size()!=frequency.size())
  {
    CNR_RETURN_FALSE(m_logger,"amplitude_sin and frequency have wrong dimensions");
  }

  m_harmonic_numbers.clear();
  for (size_t idx=0;idx<frequency.size();idx++)
  {
    double freq=std::round(frequency.at(idx)/m_carrier_frequency);
    std::pair<double,double> p(amplitude_cos.at(idx),amplitude_sin.at(idx));
    std::pair<double,std::pair<double,double>> freq_pair(freq,p);
    m_harmonic_numbers.insert(freq_pair);
  }



  m_joint_center_position=getPosition();
  m_t0=ros::Time::now();
  CNR_RETURN_TRUE(m_logger);
}

//!
//! \brief MultisineIdentificationController::doStopping
//! \return
//!
bool MultisineIdentificationController::doStopping(const ros::Time& /*time*/)
{
  CNR_TRACE_START(m_logger);
  CNR_RETURN_TRUE(m_logger);
}

//!
//! \brief MultisineIdentificationController::doUpdate
//! \return
//!
bool MultisineIdentificationController::doUpdate(const ros::Time& /*time*/, const ros::Duration& /*period*/)
{
  CNR_TRACE_START_THROTTLE_DEFAULT(m_logger);
  double t=(ros::Time::now()-m_t0).toSec();

  double y=std::sin(m_carrier_frequency*t)*m_carrier_amplitude;
  double dy=std::cos(m_carrier_frequency*t)*m_carrier_amplitude*m_carrier_frequency;

  for (const std::pair<double,std::pair<double,double>>& freq_pair: m_harmonic_numbers)
  {
    double freq=freq_pair.first;
    double ampl_cos=freq_pair.second.first;
    double ampl_sin=freq_pair.second.second;
    y+= ampl_cos*std::cos(freq*t)+ampl_sin*std::sin(freq*t);
    dy+= (-ampl_cos*std::sin(freq*t)+ampl_sin*std::cos(freq*t))*freq;
  }

  if (t<m_warmup_time)
  {
    double x=(t/m_warmup_time);
    double scaling=3.0*std::pow(x,2.0)-2.0*std::pow(x,3.0);
    y  *=scaling;
    dy *=scaling;
  }
  if (t>(m_warmup_time+m_test_time))
  {
    double t2=t-(m_warmup_time+m_test_time);
    if (t2<m_warmup_time)
    {
      double x =(m_warmup_time-t2)/m_warmup_time;
      double scaling=3.0*std::pow(x,2.0)-2.0*std::pow(x,3.0);
      y  *=scaling;
      dy *=scaling;
    }
    else
    {
      y=0.0;
      dy=0.0;
    }
  }
  rosdyn::VectorXd joint_target_position=m_joint_center_position;
  rosdyn::VectorXd joint_target_velocity=0.0*joint_target_position;

  joint_target_position(m_active_joint_idx)+=y;
  joint_target_velocity(m_active_joint_idx)+=dy;
  setCommandPosition(joint_target_position);
  setCommandVelocity(joint_target_velocity);
  CNR_RETURN_TRUE_THROTTLE_DEFAULT(m_logger);
}



}  //  end namespace identification_controllers
}
