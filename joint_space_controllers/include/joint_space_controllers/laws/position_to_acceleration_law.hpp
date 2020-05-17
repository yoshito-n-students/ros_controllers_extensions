#ifndef JOINT_SPACE_CONTROLLERS_LAWS_POSITION_TO_ACCELERATION_LAW_HPP
#define JOINT_SPACE_CONTROLLERS_LAWS_POSITION_TO_ACCELERATION_LAW_HPP

#include <limits>
#include <map>
#include <string>
#include <vector>

#include <control_toolbox/pid.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <joint_space_controllers/laws/model_based_law.hpp>
#include <joint_space_controllers/namespace_aliases.hpp>
#include <ros/console.h>
#include <ros/duration.h>
#include <ros/names.h>
#include <ros/node_handle.h>
#include <urdf/model.h>

#include <Eigen/Core>

#include <boost/foreach.hpp>
#include <boost/optional.hpp>

namespace joint_space_controllers {
namespace laws {

// =========================================================================
// a control law that converts position setpoints to acceleration setpoints
// by PID controllers
template < typename BaseT = ModelBasedLaw<> > class PositionToAccelerationLaw : public BaseT {
private:
  typedef BaseT Base;

public:
  PositionToAccelerationLaw() {}

  virtual ~PositionToAccelerationLaw() {}

  // ==============================================
  // name-based interface for controller frontends
  // ==============================================

  // ======================================================
  // initialize the law by params in the given namespace
  virtual bool init(const ros::NodeHandle &param_nh) override {
    // init the base law first
    if (!Base::init(param_nh)) {
      return false;
    }

    pids_.resize(Base::model_->getNumDofs());
    for (std::size_t i = 0; i < Base::model_->getNumDofs(); ++i) {
      const std::string &name(Base::jointIdToName(i));

      // PID controller to generate effort command based on position error
      const std::string pid_ns(param_nh.resolveName(ros::names::append("joints", name)));
      if (!pids_[i].initParam(pid_ns)) {
        ROS_ERROR_STREAM("PositionToAccelerationLaw::init(): Failed to init a PID by param '"
                         << pid_ns << "'");
        return false;
      }
    }

    return true;
  }

  // ===============================================
  // reset the model states (not destroy the model)
  virtual void starting() override {
    Base::starting();

    BOOST_FOREACH (ct::Pid &pid, pids_) { pid.reset(); }
  }

protected:
  // =======================================
  // index-based interface for derived laws
  // =======================================

  virtual Eigen::VectorXd computeCommandsEigen(const Eigen::VectorXd &qd,
                                               const ros::Duration &dt) override {
    // convert position setpoints to acceleration setpoints by applying PID controllers
    Eigen::VectorXd u(Base::model_->getNumDofs());
    const Eigen::VectorXd q(Base::getPositionsEigen());
    for (std::size_t i = 0; i < Base::model_->getNumDofs(); ++i) {
      u[i] = pids_[i].computeCommand(qd[i] - q[i], dt);
    }

    return Base::computeCommandsEigen(u, dt);
  }

private:
  std::vector< ct::Pid > pids_;
};

} // namespace laws
} // namespace joint_space_controllers

#endif