#ifndef TASK_SPACE_CONTROLLERS_LAWS_ACCELERATION_INTEGRATION_LAW_HPP
#define TASK_SPACE_CONTROLLERS_LAWS_ACCELERATION_INTEGRATION_LAW_HPP

#include <ros/duration.h>
#include <task_space_controllers/namespace_aliases.hpp>
#include <task_space_controllers/laws/model_data_law.hpp>

#include <dart/math/MathTypes.hpp> // for dm::Jacobian

#include <Eigen/Core>

namespace task_space_controllers {
namespace laws {

// ==========================================================================================
// a control law that converts acceleration setpoints from the task space to the joint space
template < typename BaseT = ModelDataLaw<> > class AccelerationIntegrationLaw : public BaseT {
private:
  typedef BaseT Base;

public:
  AccelerationIntegrationLaw() {}

  virtual ~AccelerationIntegrationLaw() {}

protected:
  // =======================================
  // index-based interface for derived laws
  // =======================================

  virtual Eigen::VectorXd computeCommandsEigen(const Eigen::VectorXd &ux,
                                               const ros::Duration &dt) override {
    // convert acc setpoint vector from the task space to the joint space
    const dm::Jacobian J(Base::model_end_link_->getWorldJacobian(Base::end_link_offset_));
    const Eigen::MatrixXd pinv_J(
        J.transpose() * (J * J.transpose() + 0.0025 * Eigen::Matrix6d::Identity()).inverse());
    const dm::Jacobian dJ(Base::model_end_link_->getJacobianClassicDeriv(Base::end_link_offset_));
    const Eigen::VectorXd dq(Base::model_->getVelocities());
    const Eigen::VectorXd uq(pinv_J * (ux - dJ * dq));

    return Base::computeCommandsEigen(uq, dt);
  }
};

} // namespace laws
} // namespace task_space_controllers

#endif