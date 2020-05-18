#ifndef TASK_SPACE_CONTROLLERS_LAWS_POSE_TO_ACCELERATION_LAW_HPP
#define TASK_SPACE_CONTROLLERS_LAWS_POSE_TO_ACCELERATION_LAW_HPP

#include <limits>
#include <map>
#include <string>

#include <control_toolbox/pid.h>
#include <ros/console.h>
#include <ros/duration.h>
#include <ros/names.h>
#include <ros/node_handle.h>
#include <task_space_controllers/laws/model_data_law.hpp>
#include <task_space_controllers/namespace_aliases.hpp>

#include <boost/foreach.hpp>

#include <Eigen/Core>

namespace task_space_controllers {
namespace laws {

// =====================================================================
// a control law that converts pose setpoints to acceleration setpoints
// using PID controllers
template < typename BaseT = ModelDataLaw<> > class PoseToAccelerationLaw : public BaseT {
private:
  typedef BaseT Base;

public:
  PoseToAccelerationLaw() {}

  virtual ~PoseToAccelerationLaw() {}

  // ==============================================
  // name-based interface for controller frontends
  // ==============================================

  virtual bool init(const ros::NodeHandle &param_nh) override {
    namespace rn = ros::names;

    // init the base law first
    if (!Base::init(param_nh)) {
      return false;
    }

    // load PID controllers from param for angular control.
    // if the first DoF is successfully initialized, then other two just copy gains from the first.
    const ros::NodeHandle angular_nh(param_nh, rn::append("task_space", "angular"));
    if (!pids_[0].init(angular_nh)) {
      return false;
    }
    pids_[1] = pids_[0];
    pids_[2] = pids_[0];
    // load PID controllers from param for linear control
    const ros::NodeHandle linear_nh(param_nh, rn::append("task_space", "linear"));
    if (!pids_[3].init(linear_nh)) {
      return false;
    }
    pids_[4] = pids_[3];
    pids_[5] = pids_[3];

    return true;
  }

  // ==================================================
  // reset internal states to restart this control law
  virtual void starting() override {
    Base::starting();

    BOOST_FOREACH (ct::Pid &pid, pids_) { pid.reset(); }
  }

protected:
  // =======================================
  // index-based interface for derived laws
  // =======================================

  virtual Eigen::VectorXd computeCommandsEigen(const Eigen::VectorXd &xd,
                                               const ros::Duration &dt) override {
    // pose error vector
    Eigen::Vector6d ex;
    const Eigen::Vector6d x(Base::getPoseEigen());
    ex = xd - x; // TODO: fix angular error calculation

    // acc setpoints based on pose errors
    Eigen::Vector6d ux;
    for (std::size_t i = 0; i < 6; ++i) {
      ux[i] = pids_[i].computeCommand(ex[i], dt);
    }

    return Base::computeCommandsEigen(ux, dt);
  }

private:
  ct::Pid pids_[6];
};

} // namespace laws
} // namespace task_space_controllers

#endif