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
  CNR_TRACE(m_logger,"initialize multisine estimator");
  m_mse=std::make_shared<identification::MultiSineEstimator>(getControllerNh(),m_logger);

  CNR_TRACE(m_logger,"selecting active joint");
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
  m_mse->initTest(m_sampling_period);

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

  double y=getPosition()(m_active_joint_idx);
  double x,dx,ddx;

  m_mse->execute(m_sampling_period,y,x,dx,ddx);

  rosdyn::VectorXd joint_target_position=m_joint_center_position;
  rosdyn::VectorXd joint_target_velocity=0.0*joint_target_position;

  joint_target_position(m_active_joint_idx)+=x;
  joint_target_velocity(m_active_joint_idx)+=dx;
  setCommandPosition(joint_target_position);
  setCommandVelocity(joint_target_velocity);
  CNR_RETURN_TRUE_THROTTLE_DEFAULT(m_logger);
}



}  //  end namespace identification_controllers
}
