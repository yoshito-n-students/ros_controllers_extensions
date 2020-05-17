#ifndef JOINT_SPACE_CONTROLLERS_LAWS_MODEL_BASED_LAW_HPP
#define JOINT_SPACE_CONTROLLERS_LAWS_MODEL_BASED_LAW_HPP

#include <map>
#include <memory>
#include <string>

#include <joint_space_controllers/laws/forward_command_law.hpp>
#include <joint_space_controllers/laws/ros_package_resource_retriever.hpp>
#include <joint_space_controllers/namespace_aliases.hpp>
#include <joint_space_controllers/utils.hpp>
#include <ros/assert.h>
#include <ros/console.h>
#include <ros/duration.h>
#include <ros/node_handle.h>

#include <dart/dynamics/DegreeOfFreedom.hpp>
#include <dart/dynamics/Joint.hpp>
#include <dart/dynamics/Skeleton.hpp>
#include <dart/utils/urdf/DartLoader.hpp>

#include <Eigen/Core>

#include <boost/foreach.hpp>

namespace joint_space_controllers {
namespace laws {

// =================================================
// a base control law for dynamics model based laws
template < typename BaseT = ForwardCommandLaw > class ModelBasedLaw : public BaseT {
private:
  typedef BaseT Base;

public:
  ModelBasedLaw() {}

  virtual ~ModelBasedLaw() {}

  // ==============================================
  // name-based interface for controller frontends
  // ==============================================

  virtual bool init(const ros::NodeHandle &param_nh) override {
    if (!Base::init(param_nh)) {
      return false;
    }

    model_ = du::DartLoader().parseSkeletonString(
        getRobotDescription(param_nh),
        // base URI to resolve relative URIs in the URDF.
        // this won't be used because almost all URIs are absolute.
        dc::Uri("file:/"), std::make_shared< ROSPackageResourceRetriever >());
    if (!model_) {
      ROS_ERROR("ModelBasedLaw::init(): Failed to build a dynamics model from "
                "robot description");
      return false;
    }

    // make sure the root joint between the world and model is fixed
    // & all other joints are fixed or single-DoF.
    // this ensures all DoFs in the model belong to different joints
    // so that we can convert DoF indices & joint names.
    if (model_->getRootJoint()->getNumDofs() > 0) {
      ROS_ERROR("ModelBasedLaw::init(): Non-fixed root joint");
      return false;
    }
    BOOST_FOREACH (const dd::Joint *const joint, model_->getJoints()) {
      if (joint->getNumDofs() > 1) {
        ROS_ERROR_STREAM("ModelBasedLaw::init(): multi-DoF joint '" << joint->getName() << "'");
        return false;
      }
    }

    return true;
  }

  // ==================================================
  // update model state based on given joint positions
  virtual void setPositions(const std::map< std::string, double > &positions) override {
    Base::setPositions(positions);
    setPositionsEigen(jointMapToEigen(positions));
  }

  // ===================================================
  // update model state based on given joint velocities
  virtual void setVelocities(const std::map< std::string, double > &velocities) override {
    Base::setVelocities(velocities);
    setVelocitiesEigen(jointMapToEigen(velocities));
  }

  // =====================================================================
  // compute joint commands to realize given setpoints based on the model
  virtual std::map< std::string, double >
  computeCommands(const std::map< std::string, double > &setpoints,
                  const ros::Duration &dt) override {
    const std::map< std::string, double > modified_setpoints(
        eigenToJointMap(computeCommandsEigen(jointMapToEigen(setpoints), dt)));
    return Base::computeCommands(modified_setpoints, dt);
  }

protected:
  // =======================================
  // index-based interface for derived laws
  // =======================================

  virtual void setPositionsEigen(const Eigen::VectorXd &q) {
    for (std::size_t i = 0; i < model_->getNumDofs(); ++i) {
      model_->getDof(i)->setPosition(q[i]);
      // use joint state forwarded by one time step for better control stability
      // (recommended in https://dartsim.github.io/tutorials_manipulator.html)
      // model_->getDof(i)->setPosition(q[i] + dq[i] * dt.toSec());
    }
  }

  virtual void setVelocitiesEigen(const Eigen::VectorXd &dq) {
    for (std::size_t i = 0; i < model_->getNumDofs(); ++i) {
      model_->getDof(i)->setVelocity(dq[i]);
    }
  }

  virtual Eigen::VectorXd computeCommandsEigen(const Eigen::VectorXd &vd, const ros::Duration &dt) {
    // just forward given setpoints as commands. this should be overrided in child laws
    return vd;
  }

  Eigen::VectorXd getPositionsEigen() const {
    Eigen::VectorXd q(model_->getNumDofs());
    for (std::size_t i = 0; i < model_->getNumDofs(); ++i) {
      q[i] = model_->getDof(i)->getPosition();
    }
    return q;
  }

  // ================================
  // conversion between name & index
  // ================================

  const std::string &jointIdToName(const std::size_t id) const {
    return model_->getDof(id)->getJoint()->getName();
  }

  Eigen::VectorXd jointMapToEigen(const std::map< std::string, double > &m) const {
    Eigen::VectorXd e(model_->getNumDofs());
    for (std::size_t i = 0; i < model_->getNumDofs(); ++i) {
      e[i] = findValue(m, jointIdToName(i));
    }
    return e;
  }

  std::map< std::string, double > eigenToJointMap(const Eigen::VectorXd &e) const {
    ROS_ASSERT_MSG(e.size() == model_->getNumDofs(),
                   "ModelBasedLaw::eigenToJointMap(): Invalid vector size");

    std::map< std::string, double > m;
    for (std::size_t i = 0; i < model_->getNumDofs(); ++i) {
      m[jointIdToName(i)] = e[i];
    }
    return m;
  }

protected:
  dd::SkeletonPtr model_;
};

} // namespace laws
} // namespace joint_space_controllers

#endif