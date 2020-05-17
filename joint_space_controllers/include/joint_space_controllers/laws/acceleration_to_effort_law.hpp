#ifndef JOINT_SPACE_CONTROLLERS_LAWS_ACCELERATION_TO_EFFORT_LAW_HPP
#define JOINT_SPACE_CONTROLLERS_LAWS_ACCELERATION_TO_EFFORT_LAW_HPP

#include <joint_space_controllers/laws/model_based_law.hpp>
#include <ros/duration.h>

#include <Eigen/Core>

namespace joint_space_controllers {
namespace laws {

// =======================================================================
// a control law that converts acceleration setpoints to effort setpoints
// based on a dynamics model
template < typename BaseT = ModelBasedLaw<> > class AccelerationToEffortLaw : public BaseT {
private:
  typedef BaseT Base;

public:
  AccelerationToEffortLaw() {}

  virtual ~AccelerationToEffortLaw() {}

protected:
  // =======================================
  // index-based interface for derived laws
  // =======================================

  virtual Eigen::VectorXd computeCommandsEigen(const Eigen::VectorXd &u,
                                               const ros::Duration &dt) override {
    const Eigen::VectorXd t(Base::model_->getMassMatrix() * u +
                            Base::model_->getCoriolisAndGravityForces());
    return Base::computeCommandsEigen(t, dt);
  }
};

} // namespace laws
} // namespace joint_space_controllers

#endif