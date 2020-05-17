#ifndef TASK_SPACE_CONTROLLERS_LAWS_MODEL_BASED_LAW_HPP
#define TASK_SPACE_CONTROLLERS_LAWS_MODEL_BASED_LAW_HPP

#include <map>
#include <string>

#include <joint_space_controllers/laws/model_based_law.hpp>
#include <ros/assert.h>
#include <ros/console.h>
#include <ros/node_handle.h>
#include <task_space_controllers/namespace_aliases.hpp>
#include <task_space_controllers/laws/forward_command_law.hpp>
#include <task_space_controllers/utils.hpp>

#include <dart/dynamics/BodyNode.hpp>

#include <Eigen/Core>

namespace task_space_controllers {
namespace laws {

// =================================================
// a base control law for dynamics model based laws
template < typename BaseT = jscl::ModelBasedLaw< ForwardCommandLaw > >
class ModelBasedLaw : public BaseT {
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

    // TODO: end effector info from param
    model_end_link_ = Base::model_->getBodyNode(Base::model_->getNumBodyNodes() - 1);
    end_link_offset_ = Eigen::Vector3d::Zero();

    return true;
  }

  virtual std::map< std::string, double > getPose() const override {
    return eigenToDofMap(getPoseEigen());
  }

protected:
  // =======================================
  // index-based interface for derived laws
  // =======================================

  Eigen::Vector6d getPoseEigen() const {
    const Eigen::Isometry3d T(model_end_link_->getWorldTransform());
    const Eigen::AngleAxisd aa(T.linear());
    Eigen::Vector6d x;
    x.head< 3 >() = aa.angle() * aa.axis(); // angular pos
    x.tail< 3 >() = T * end_link_offset_;   // linear pos
    return x;
  }

  // ================================
  // conversion between name & index
  // ================================

  static std::string dofIdToName(const std::size_t id) {
    ROS_ASSERT_MSG(id >= 0 && id < 6, "ModelBasedLaw::dofIdToName(): Invalid id (%d)",
                   static_cast< int >(id));

    switch (id) {
    case 0:
      return "angular_x";
    case 1:
      return "angular_y";
    case 2:
      return "angular_z";
    case 3:
      return "linear_x";
    case 4:
      return "linear_y";
    case 5:
      return "linear_z";
    }
  }

  static Eigen::Vector6d dofMapToEigen(const std::map< std::string, double > &m) {
    Eigen::Vector6d e;
    for (std::size_t i = 0; i < 6; ++i) {
      e[i] = findValue(m, dofIdToName(i));
    }
    return e;
  }

  static std::map< std::string, double > eigenToDofMap(const Eigen::Vector6d &e) {
    std::map< std::string, double > m;
    for (std::size_t i = 0; i < 6; ++i) {
      m[dofIdToName(i)] = e[i];
    }
    return m;
  }

protected:
  dd::BodyNodePtr model_end_link_;
  Eigen::Vector3d end_link_offset_;
};

} // namespace laws
} // namespace task_space_controllers

#endif