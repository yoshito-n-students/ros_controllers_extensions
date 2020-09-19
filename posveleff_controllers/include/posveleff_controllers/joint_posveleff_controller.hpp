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
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>

#include <boost/array.hpp>

namespace posveleff_controllers {

class JointPosVelEffController : public controller_interface::Controller<
                                     hardware_interface_extensions::PosVelEffJointInterface > {
public:
  JointPosVelEffController() {}
  virtual ~JointPosVelEffController() {
    cmd_subs_[0].shutdown();
    cmd_subs_[1].shutdown();
    cmd_subs_[2].shutdown();
  }

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

    if (nh.param("separate_command", false)) {
      cmd_subs_[0] = nh.subscribe< std_msgs::Float64 >(
          "pos_command", 1, &JointPosVelEffController::separatedCommandCB< 0 >, this);
      cmd_subs_[1] = nh.subscribe< std_msgs::Float64 >(
          "vel_command", 1, &JointPosVelEffController::separatedCommandCB< 1 >, this);
      cmd_subs_[2] = nh.subscribe< std_msgs::Float64 >(
          "eff_command", 1, &JointPosVelEffController::separatedCommandCB< 2 >, this);
    } else {
      cmd_subs_[0] = nh.subscribe< std_msgs::Float64MultiArray >(
          "command", 1, &JointPosVelEffController::commandCB, this);
    }

    return true;
  }

  virtual void starting(const ros::Time & /*time*/) {
    const boost::array< double, 3 > cmd = {joint_.getPosition(), 0., 0.};
    cmd_buf_.writeFromNonRT(cmd);
  }

  virtual void update(const ros::Time & /*time*/, const ros::Duration & /*period*/) {
    const boost::array< double, 3 > *const cmd(cmd_buf_.readFromRT());
    joint_.setCommandPosition((*cmd)[0]);
    joint_.setCommandVelocity((*cmd)[1]);
    joint_.setCommandEffort((*cmd)[2]);
  }

private:
  void commandCB(const std_msgs::Float64MultiArrayConstPtr &msg) {
    if (msg->data.size() == 3) {
      const boost::array< double, 3 > cmd = {msg->data[0], msg->data[1], msg->data[2]};
      cmd_buf_.writeFromNonRT(cmd);
    } else {
      ROS_ERROR_STREAM("Data size must be 3 (received data size: " << msg->data.size() << ")");
    }
  }

  template < std::size_t I > void separatedCommandCB(const std_msgs::Float64ConstPtr &msg) {
    boost::array< double, 3 > cmd(*cmd_buf_.readFromNonRT());
    cmd[I] = msg->data;
    cmd_buf_.writeFromNonRT(cmd);
  }

private:
  hardware_interface_extensions::PosVelEffJointHandle joint_;
  realtime_tools::RealtimeBuffer< boost::array< double, 3 > > cmd_buf_;
  ros::Subscriber cmd_subs_[3];
};
} // namespace posveleff_controllers

#endif