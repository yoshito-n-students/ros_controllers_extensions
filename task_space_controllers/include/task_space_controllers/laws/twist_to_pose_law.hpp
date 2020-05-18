#ifndef TASK_SPACE_CONTROLLERS_LAWS_TWIST_TO_POSE_LAW_HPP
#define TASK_SPACE_CONTROLLERS_LAWS_TWIST_TO_POSE_LAW_HPP

#include <ros/duration.h>
#include <task_space_controllers/laws/model_data_law.hpp>

#include <boost/math/special_functions/fpclassify.hpp> // for boost::math::isnan()

namespace task_space_controllers {
namespace laws {

// ====================================================================================
// a control law that converts twist setpoints to pose setpoints by simple integration
template < typename BaseT = ModelDataLaw<> > class TwistToPoseLaw : public BaseT {
private:
  typedef BaseT Base;

public:
  TwistToPoseLaw() {}

  virtual ~TwistToPoseLaw() {}

protected:
  // =======================================
  // index-based interface for derived laws
  // =======================================

  virtual Eigen::VectorXd computeCommandsEigen(const Eigen::VectorXd &dxd,
                                               const ros::Duration &dt) override {
    // calc pose setpoints by appending previous pose setpoints to twist setpoints.
    // previous pose setpoints may be NaNs on the first control step.
    Eigen::Vector6d xd;
    const Eigen::Vector6d prev_xd(Base::getPoseSetpointsEigen()), x(Base::getPoseEigen());
    for (std::size_t i = 0; i < 6; ++i) {
      xd[i] = (boost::math::isnan(prev_xd[i]) ? x[i] : prev_xd[i]) + dxd[i] * dt.toSec();
      // TODO: fix angular accumuration
    }

    return Base::computeCommandsEigen(xd, dt);
  }
};

} // namespace laws
} // namespace task_space_controllers

#endif