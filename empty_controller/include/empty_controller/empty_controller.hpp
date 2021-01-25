#ifndef EMPTY_CONTROLLER_EMPTY_CONTROLLER_HPP
#define EMPTY_CONTROLLER_EMPTY_CONTROLLER_HPP

#include <controller_interface/controller_base.h>
#include <hardware_interface/robot_hw.h>
#include <ros/common.h>
#include <ros/duration.h>
#include <ros/node_handle.h>
#include <ros/time.h>

namespace empty_controller {

class EmptyController : public controller_interface::ControllerBase {
public:
  virtual void update(const ros::Time & /*time*/, const ros::Duration & /*period*/) {}

  virtual bool initRequest(hardware_interface::RobotHW * /*robot_hw*/,
                           ros::NodeHandle & /*root_nh*/, ros::NodeHandle & /*controller_nh*/,
                           ClaimedResources &claimed_resources) {
    claimed_resources.clear(); // claims no hw resources because this controller does nothing
#if ROS_VERSION >= ROS_VERSION_COMBINED(1, 15, 0)
    // noetic
    state_ = ControllerState::INITIALIZED;
#else
    // melodic or earlier
    state_ = INITIALIZED;
#endif
    return true;
  }
};
} // namespace empty_controller
#endif