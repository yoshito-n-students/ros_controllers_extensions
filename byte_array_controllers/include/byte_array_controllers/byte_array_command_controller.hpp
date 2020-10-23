#ifndef BYTE_ARRAY_CONTROLLERS_BYTE_ARRAY_COMMAND_CONTROLLER_HPP
#define BYTE_ARRAY_CONTROLLERS_BYTE_ARRAY_COMMAND_CONTROLLER_HPP

#include <string>

#include <byte_array_controllers/common_namespaces.hpp>
#include <controller_interface/controller.h>
#include <hardware_interface/hardware_interface.h> // for HardwareInterfaceException
#include <hardware_interface_extensions/byte_array_interface.hpp>
#include <realtime_tools/realtime_buffer.h>
#include <ros/console.h>
#include <ros/duration.h>
#include <ros/node_handle.h>
#include <ros/subscriber.h>
#include <ros/time.h>
#include <std_msgs/ByteMultiArray.h>

namespace byte_array_controllers {

class ByteArrayCommandController
    : public controller_interface::Controller< hie::ByteArrayCommandInterface > {
public:
  ByteArrayCommandController() {}
  virtual ~ByteArrayCommandController() { cmd_sub_.shutdown(); }

  virtual bool init(hie::ByteArrayCommandInterface *hw, ros::NodeHandle &nh) {
    std::string handle_name;
    if (!nh.getParam("handle", handle_name)) {
      ROS_ERROR_STREAM("Failed to get required param " << nh.resolveName("handle"));
      return false;
    }

    try {
      cmd_handle_ = hw->getHandle(handle_name);
    } catch (const hi::HardwareInterfaceException &ex) {
      ROS_ERROR_STREAM("Failed to get hardware handle: " << ex.what());
      return false;
    }

    cmd_sub_ = nh.subscribe< std_msgs::ByteMultiArray >("command", 1,
                                                        &ByteArrayCommandController::commandCB, this);

    return true;
  }

  virtual void starting(const ros::Time & /*time*/) { cmd_buf_.writeFromNonRT(hie::ByteArray()); }

  virtual void update(const ros::Time & /*time*/, const ros::Duration & /*period*/) {
    const hie::ByteArray *const cmd(cmd_buf_.readFromRT());
    cmd_handle_.setData(*cmd);
  }

private:
  void commandCB(const std_msgs::ByteMultiArrayConstPtr &msg) {
    cmd_buf_.writeFromNonRT(hie::ByteArray::fromMsg(*msg));
  }

private:
  hie::ByteArrayHandle cmd_handle_;
  realtime_tools::RealtimeBuffer< hie::ByteArray > cmd_buf_;
  ros::Subscriber cmd_sub_;
};
} // namespace byte_array_controllers

#endif