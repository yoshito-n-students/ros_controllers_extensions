#ifndef TASK_SPACE_CONTROLLERS_LAWS_VELOCITY_INTEGRATION_LAW_HPP
#define TASK_SPACE_CONTROLLERS_LAWS_VELOCITY_INTEGRATION_LAW_HPP

#include <ros/duration.h>
#include <task_space_controllers/laws/model_based_law.hpp>
#include <task_space_controllers/namespace_aliases.hpp>

#include <dart/math/MathTypes.hpp> // for dm::Jacobian

#include <Eigen/Core>

namespace task_space_controllers {
namespace laws {

// ======================================================================================
// a control law that converts velocity setpoints from the task space to the joint space
template < typename BaseT = ModelBasedLaw<> > class VelocityIntegrationLaw : public BaseT {
private:
  typedef BaseT Base;

public:
  VelocityIntegrationLaw() {}

  virtual ~VelocityIntegrationLaw() {}

protected:
  // =======================================
  // index-based interface for derived laws
  // =======================================

  virtual Eigen::VectorXd computeCommandsEigen(const Eigen::VectorXd &dxd,
                                               const ros::Duration &dt) override {
    // convert vel setpoint vector from the task space to the joint space
    const dm::Jacobian J(Base::model_end_link_->getWorldJacobian(Base::end_link_offset_));
    const Eigen::MatrixXd pinv_J(
        J.transpose() * (J * J.transpose() + 0.0025 * Eigen::Matrix6d::Identity()).inverse());
    const Eigen::VectorXd dqd(pinv_J * dxd);

    return Base::computeCommandsEigen(dqd, dt);
  }
};

} // namespace laws
} // namespace task_space_controllers

#endif