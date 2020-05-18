#ifndef JOINT_SPACE_CONTROLLERS_LAWS_VELOCITY_TO_POSITION_LAW_HPP
#define JOINT_SPACE_CONTROLLERS_LAWS_VELOCITY_TO_POSITION_LAW_HPP

#include <joint_space_controllers/laws/model_data_law.hpp>
#include <ros/duration.h>

#include <Eigen/Core>

#include <boost/math/special_functions/fpclassify.hpp> // for boost::math::isnan()

namespace joint_space_controllers {
namespace laws {

// =====================================================================
// a control law that converts velocity setpoints to position setpoints
// by simple accumulation
template < typename BaseT = ModelDataLaw<> > class VelocityToPositionLaw : public BaseT {
private:
  typedef BaseT Base;

public:
  VelocityToPositionLaw() {}

  virtual ~VelocityToPositionLaw() {}

protected:
  // =======================================
  // index-based interface for derived laws
  // =======================================

  virtual Eigen::VectorXd computeCommandsEigen(const Eigen::VectorXd &dqd,
                                               const ros::Duration &dt) override {
    // calc position setpoints by appending previous position setpoints to velocity setpoints.
    // if previous position setpoints are NaNs (e.g. the first control step),
    // use latest positions instead of position setpoints.
    Eigen::VectorXd qd(Base::model_->getNumDofs());
    const Eigen::VectorXd prev_qd(Base::getPositionSetpointsEigen()), q(Base::getPositionsEigen());
    for (std::size_t i = 0; i < Base::model_->getNumDofs(); ++i) {
      qd[i] = (boost::math::isnan(prev_qd[i]) ? q[i] : prev_qd[i]) + dqd[i] * dt.toSec();
    }

    // calc commands based on the position setpoints
    return Base::computeCommandsEigen(qd, dt);
  }
};

} // namespace laws
} // namespace joint_space_controllers

#endif