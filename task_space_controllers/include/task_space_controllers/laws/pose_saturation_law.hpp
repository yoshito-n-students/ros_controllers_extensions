#ifndef TASK_SPACE_CONTROLLERS_LAWS_POSE_SATURATION_LAW_HPP
#define TASK_SPACE_CONTROLLERS_LAWS_POSE_SATURATION_LAW_HPP

#include <limits>
#include <map>
#include <string>

#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <ros/console.h>
#include <ros/duration.h>
#include <ros/node_handle.h>
#include <task_space_controllers/laws/model_based_law.hpp>
#include <task_space_controllers/namespace_aliases.hpp>

#include <boost/assign/list_of.hpp>
#include <boost/foreach.hpp>
#include <boost/optional.hpp>

namespace task_space_controllers {
namespace laws {

// ==================================================================================
// a control law that saturates pose setpoints based on joint limits from ros-params
template < typename BaseT = ModelBasedLaw<> > class PoseSaturationLaw : public BaseT {
private:
  typedef BaseT Base;

public:
  PoseSaturationLaw() {}

  virtual ~PoseSaturationLaw() {}

  // ==============================================
  // name-based interface for controller frontends
  // ==============================================

  virtual bool init(const ros::NodeHandle &param_nh) override {
    namespace ba = boost::assign;

    // init the base model first
    if (!Base::init(param_nh)) {
      return false;
    }

    // load joint limits for each DoF if given
    ros::NodeHandle limits_nh(param_nh.getNamespace(),
                              /* remappings = */ ba::map_list_of< std::string, std::string >(
                                  "joint_limits/angular", "task_space/angular")(
                                  "joint_limits/linear", "task_space/linear"));
    // try loading limits from 'task_space/angular', which is an alias of 'joint_limits/angular'
    jli::JointLimits angular_limits;
    if (jli::getJointLimits("angular", limits_nh, angular_limits)) {
      if (!initDofLimits(&dof_infos_[0], angular_limits) ||
          !initDofLimits(&dof_infos_[1], angular_limits) ||
          !initDofLimits(&dof_infos_[2], angular_limits)) {
        return false;
      }
    }
    // try loading limits from 'task_space/linear', which is an alias of 'joint_limits/linear'
    jli::JointLimits linear_limits;
    if (jli::getJointLimits("linear", limits_nh, linear_limits)) {
      if (!initDofLimits(&dof_infos_[3], linear_limits) ||
          !initDofLimits(&dof_infos_[4], linear_limits) ||
          !initDofLimits(&dof_infos_[5], linear_limits)) {
        return false;
      }
    }

    return true;
  }

  // =============================================
  // reset saturation handles to restart this law
  virtual void starting() override {
    Base::starting();

    BOOST_FOREACH (DofInfo &dof, dof_infos_) {
      dof.pos_sp = std::numeric_limits< double >::quiet_NaN();
      if (dof.pos_sp_sat_handle) {
        dof.pos_sp_sat_handle->reset();
      }
    }
  }

protected:
  // =======================================
  // index-based interface for derived laws
  // =======================================

  virtual void setPositionsEigen(const Eigen::VectorXd &q) override {
    Base::setPositionsEigen(q);

    const Eigen::Vector6d x(Base::getPoseEigen());
    for (std::size_t i = 0; i < 6; ++i) {
      dof_infos_[i].pos = x[i];
    }
  }

  virtual Eigen::VectorXd computeCommandsEigen(const Eigen::VectorXd &xd,
                                               const ros::Duration &dt) override {
    // desired pose vector
    Eigen::Vector6d xd_clamped;
    for (std::size_t i = 0; i < 6; ++i) {
      DofInfo &dof(dof_infos_[i]);
      // desired pose
      dof.pos_sp = xd[i];
      if (dof.pos_sp_sat_handle) {
        dof.pos_sp_sat_handle->enforceLimits(dt);
      }
      xd_clamped[i] = dof.pos_sp;
    }

    return Base::computeCommandsEigen(xd_clamped, dt);
  }

  Eigen::Vector6d getPoseSetpointsEigen() const {
    Eigen::Vector6d xd;
    for (std::size_t i = 0; i < 6; ++i) {
      xd[i] = dof_infos_[i].pos_sp;
    }
    return xd;
  }

private:
  struct DofInfo {
    double pos, vel, eff;
    double pos_sp;
    boost::optional< jli::PositionJointSaturationHandle > pos_sp_sat_handle;
  };

  static bool initDofLimits(DofInfo *const dof, const jli::JointLimits &limits) {
    const hi::JointStateHandle state_handle("dof", &dof->pos, &dof->vel, &dof->eff);
    dof->pos_sp_sat_handle =
        jli::PositionJointSaturationHandle(hi::JointHandle(state_handle, &dof->pos_sp), limits);
    return true;
  }

private:
  DofInfo dof_infos_[6];
};

} // namespace laws
} // namespace task_space_controllers

#endif