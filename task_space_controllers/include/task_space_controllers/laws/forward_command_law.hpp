#ifndef TASK_SPACE_CONTROLLERS_LAWS_FORWARD_COMMAND_LAW_HPP
#define TASK_SPACE_CONTROLLERS_LAWS_FORWARD_COMMAND_LAW_HPP

#include <map>
#include <string>

#include <ros/duration.h>
#include <ros/node_handle.h>
#include <task_space_controllers/laws/abstract_law.hpp>

namespace task_space_controllers {
namespace laws {

// ==========================================================================
// the simplest control law that forwards commands without any modification
class ForwardCommandLaw : public AbstractLaw {
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

  virtual std::map< std::string, double > getPose() const override {
    std::map< std::string, double > zero_pose;
    zero_pose["angular_x"] = 0.;
    zero_pose["angular_y"] = 0.;
    zero_pose["angular_z"] = 0.;
    zero_pose["linear_x"] = 0.;
    zero_pose["linear_y"] = 0.;
    zero_pose["linear_z"] = 0.;
    return zero_pose;
  }
};

} // namespace laws
} // namespace task_space_controllers

#endif