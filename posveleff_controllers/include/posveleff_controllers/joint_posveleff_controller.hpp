#ifndef POSVEL_CONTROLLERS_JOINT_POSVEL_CONTROLLER_HPP
#define POSVEL_CONTROLLERS_JOINT_POSVEL_CONTROLLER_HPP

#include <string>

#include <controller_interface/controller.h>
#include <hardware_interface/hardware_interface.h> // for HardwareInterfaceException
#include <hardware_interface_extensions/posveleff_command_interface.hpp>
#include <realtime_tools/realtime_buffer.h>
#include <ros/console.h>
#include <ros/duration.h>
#include <ros/node_handle.h>
#include <ros/subscriber.h>
#include <ros/time.h>
#include <std_msgs/Float64MultiArray.h>

#include <boost/array.hpp>

namespace posveleff_controllers {

class JointPosVelEffController : public controller_interface::Controller<
                                     hardware_interface_extensions::PosVelEffJointInterface > {
public:
  JointPosVelEffController() {}
  virtual ~JointPosVelEffController() { cmd_sub_.shutdown(); }

  virtual bool init(hardware_interface_extensions::PosVelEffJointInterface *hw,
                    ros::NodeHandle &nh) {
    std::string joint_name;
    if (!nh.getParam("joint", joint_name)) {
      ROS_ERROR_STREAM("Failed to get required param " << nh.resolveName("joint"));
      return false;
    }

    try {
      joint_ = hw->getHandle(joint_name);
    } catch (const hardware_interface::HardwareInterfaceException &ex) {
      ROS_ERROR_STREAM("Failed to get hardware handle: " << ex.what());
      return false;
    }

    cmd_sub_ = nh.subscribe< std_msgs::Float64MultiArray >(
        "command", 1, &JointPosVelEffController::commandCB, this);

    return true;
  }

  virtual void starting() {
    const boost::array< double, 3 > command = {joint_.getPosition(), 0., 0.};
    command_buffer_.writeFromNonRT(command);
  }

  virtual void update(const ros::Time & /*time*/, const ros::Duration & /*period*/) {
    const boost::array< double, 3 > *const command(command_buffer_.readFromRT());
    joint_.setCommandPosition((*command)[0]);
    joint_.setCommandVelocity((*command)[1]);
    joint_.setCommandEffort((*command)[2]);
  }

private:
  void commandCB(const std_msgs::Float64MultiArrayConstPtr &msg) {
    if (msg->data.size() == 3) {
      const boost::array< double, 3 > command = {msg->data[0], msg->data[1], msg->data[2]};
      command_buffer_.writeFromNonRT(command);
    } else {
      ROS_ERROR_STREAM("Data size must be 3 (received data size: " << msg->data.size() << ")");
    }
  }

private:
  hardware_interface_extensions::PosVelEffJointHandle joint_;
  realtime_tools::RealtimeBuffer< boost::array< double, 3 > > command_buffer_;
  ros::Subscriber cmd_sub_;
};
} // namespace posveleff_controllers

#endif