#ifndef JOINT_SPACE_CONTROLLERS_LAWS_FORWARD_COMMAND_LAW_HPP
#define JOINT_SPACE_CONTROLLERS_LAWS_FORWARD_COMMAND_LAW_HPP

#include <map>
#include <string>

#include <joint_space_controllers/laws/abstract_law.hpp>
#include <ros/duration.h>
#include <ros/node_handle.h>

namespace joint_space_controllers {
namespace laws {

// =====================================================================
// simplest control law that forwards commands without any modification
template < typename AbstractT = AbstractLaw > class ForwardCommandLaw : public AbstractT {
public:
  ForwardCommandLaw() {}

  virtual ~ForwardCommandLaw() {}

  virtual bool init(const ros::NodeHandle &param_nh) override { return true; }

  virtual void starting() override {}

  virtual void setPositions(const std::map< std::string, double > &positions) override {}

  virtual void setVelocities(const std::map< std::string, double > &velocities) override {}

  virtual std::map< std::string, double >
  computeCommands(const std::map< std::string, double > &setpoints,
                  const ros::Duration &dt) override {
    return setpoints;
  }
};

} // namespace laws
} // namespace joint_space_controllers

#endif