#ifndef TASK_SPACE_CONTROLLERS_LAWS_POSE_TO_TWIST_LAW_HPP
#define TASK_SPACE_CONTROLLERS_LAWS_POSE_TO_TWIST_LAW_HPP

#include <ros/duration.h>
#include <task_space_controllers/laws/model_data_law.hpp>

#include <Eigen/Core>

namespace task_space_controllers {
namespace laws {

// ==============================================================
// a control law that converts pose setpoints to twist setpoints
// by simple differential operation
template < typename BaseT = ModelDataLaw<> > class PoseToTwistLaw : public BaseT {
private:
  typedef BaseT Base;

public:
  PoseToTwistLaw() {}

  virtual ~PoseToTwistLaw() {}

protected:
  // =======================================
  // index-based interface for derived laws
  // =======================================

  virtual Eigen::VectorXd computeCommandsEigen(const Eigen::VectorXd &xd,
                                               const ros::Duration &dt) override {
    Eigen::Vector6d dxd;
    const Eigen::Vector6d x(Base::getPoseEigen());
    dxd = (xd - x) / dt.toSec(); // TODO: fix angular difference calculation

    return Base::computeCommandsEigen(dxd, dt);
  }
};

} // namespace laws
} // namespace task_space_controllers

#endif