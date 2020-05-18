#ifndef JOINT_SPACE_CONTROLLERS_LAWS_POSITION_SATURATION_LAW_HPP
#define JOINT_SPACE_CONTROLLERS_LAWS_POSITION_SATURATION_LAW_HPP

#include <limits>
#include <map>
#include <string>
#include <vector>

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <joint_space_controllers/namespace_aliases.hpp>
#include <joint_space_controllers/laws/model_data_law.hpp>
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

// ======================================================================================
// a control law that saturates position setpoints based on joint limits from ros-params
template < typename BaseT = ModelDataLaw<> > class PositionSaturationLaw : public BaseT {
private:
  typedef BaseT Base;

public:
  PositionSaturationLaw() {}

  virtual ~PositionSaturationLaw() {}

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

    // parse robot description to extract joint names & limits
    urdf::Model robot_desc;
    if (!robot_desc.initString(getRobotDescription(param_nh))) {
      ROS_ERROR(
          "PositionSaturationLaw::init(): Failed to parse robot description as an URDF");
      return false;
    }

    // compose joint info
    joint_infos_.resize(Base::model_->getNumDofs());
    for (std::size_t i = 0; i < Base::model_->getNumDofs(); ++i) {
      const std::string &name(Base::jointIdToName(i));
      JointInfo &info(joint_infos_[i]);

      // load joint limits from robot description if limits exist
      jli::JointLimits limits;
      if (jli::getJointLimits(robot_desc.getJoint(name), limits)) {
        const hi::JointStateHandle state_handle(name, &info.pos, &info.vel, &info.eff);
        const hi::JointHandle pos_sp_handle(state_handle, &info.pos_sp);
        info.pos_sp_sat_handle = jli::PositionJointSaturationHandle(pos_sp_handle, limits);
      }
    }

    return true;
  }

  // ===============================================
  // reset the model states (not destroy the model)
  virtual void starting() override {
    Base::starting();

    BOOST_FOREACH (JointInfo &info, joint_infos_) {
      info.pos_sp = std::numeric_limits< double >::quiet_NaN();
      if (info.pos_sp_sat_handle) {
        info.pos_sp_sat_handle->reset();
      }
    }
  }

protected:
  // =======================================
  // index-based interface for derived laws
  // =======================================

  Eigen::VectorXd getPositionSetpointsEigen() const {
    Eigen::VectorXd qd(joint_infos_.size());
    for (std::size_t i = 0; i < joint_infos_.size(); ++i) {
      qd[i] = joint_infos_[i].pos_sp;
    }
    return qd;
  }

  virtual void setPositionsEigen(const Eigen::VectorXd &q) override {
    Base::setPositionsEigen(q);

    for (std::size_t i = 0; i < Base::model_->getNumDofs(); ++i) {
      joint_infos_[i].pos = q[i];
    }
  }

  virtual Eigen::VectorXd computeCommandsEigen(const Eigen::VectorXd &qd,
                                               const ros::Duration &dt) override {
    // convert position setpoints to acceleration setpoints
    // by saturating position setpoints according to the joint limits
    Eigen::VectorXd qd_clamped(Base::model_->getNumDofs());
    for (std::size_t i = 0; i < Base::model_->getNumDofs(); ++i) {
      JointInfo &info(joint_infos_[i]);
      info.pos_sp = qd[i];
      if (info.pos_sp_sat_handle) {
        info.pos_sp_sat_handle->enforceLimits(dt);
      }
      qd_clamped[i] = info.pos_sp;
    }

    return Base::computeCommandsEigen(qd_clamped, dt);
  }

private:
  struct JointInfo {
    double pos, vel, eff;
    double pos_sp;
    boost::optional< jli::PositionJointSaturationHandle > pos_sp_sat_handle;
  };
  std::vector< JointInfo > joint_infos_;
};

} // namespace laws
} // namespace joint_space_controllers

#endif