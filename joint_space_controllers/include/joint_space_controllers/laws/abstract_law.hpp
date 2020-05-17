#ifndef JOINT_SPACE_CONTROLLERS_LAWS_ABSTRACT_LAW_HPP
#define JOINT_SPACE_CONTROLLERS_LAWS_ABSTRACT_LAW_HPP

#include <map>
#include <string>

#include <ros/duration.h>
#include <ros/node_handle.h>

namespace joint_space_controllers {
namespace laws {

// ===================================
// An abstract class for control laws
class AbstractLaw {
public:
  AbstractLaw() {}

  virtual ~AbstractLaw() {}

  // ===================
  // initialize the law
  virtual bool init(const ros::NodeHandle &param_nh) = 0;

  // =================================
  // reset internal states of the law
  virtual void starting() = 0;

  // ========================================================
  // update the joint positions of internal model in the law
  virtual void setPositions(const std::map< std::string, double > &positions) = 0;

  // ========================================================
  // update the joint velocities of internal model in the law
  virtual void setVelocities(const std::map< std::string, double > &velocities) = 0;

  // ============================================================
  // compute commands to the hardware based on control setpoints
  virtual std::map< std::string, double >
  computeCommands(const std::map< std::string, double > &setpoints, const ros::Duration &dt) = 0;
};

} // namespace laws
} // namespace joint_space_controllers

#endif