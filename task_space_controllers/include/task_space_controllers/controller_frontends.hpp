#ifndef TASK_SPACE_CONTROLLERS_CONTROLLER_FRONTENDS_HPP
#define TASK_SPACE_CONTROLLERS_CONTROLLER_FRONTENDS_HPP

#include <map>
#include <string>

#include <controller_interface/controller.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <joint_space_controllers/hardware/abstract_hardware.hpp>
#include <realtime_tools/realtime_buffer.h>
#include <ros/console.h>
#include <ros/duration.h>
#include <ros/node_handle.h>
#include <ros/subscriber.h>
#include <ros/time.h>
#include <task_space_controllers/laws/abstract_law.hpp>
#include <task_space_controllers/namespace_aliases.hpp>

#include <Eigen/Core>

#include <boost/scoped_ptr.hpp>

namespace task_space_controllers {

// ==============================================================================
// controller frontend subscribing task space commands & passing them to backend
template < typename MessageT, typename LawT, typename HardwareT >
class ControllerFrontend : public ci::Controller< typename HardwareT::HardwareInterface > {
public:
  typedef MessageT Message;
  typedef LawT Law;
  typedef HardwareT Hardware;
  typedef typename Hardware::HardwareInterface HardwareInterface;

public:
  ControllerFrontend() {}

  virtual ~ControllerFrontend() {}

  // required interfaces as a Controller

  virtual bool init(HardwareInterface *hw_iface, ros::NodeHandle &controller_nh) override {
    // init the controller backend
    jnt_hw_.reset(new Hardware());
    if (!jnt_hw_->init(hw_iface, controller_nh)) {
      ROS_ERROR("ControllerFrontend::init(): Failed to init the joint hardware");
      return false;
    }
    law_.reset(new Law());
    if (!law_->init(controller_nh)) {
      ROS_ERROR("ControllerFrontend::init(): Failed to init the control law");
      return false;
    }

    // subscribe command for task space dofs
    cmd_sub_ = controller_nh.subscribe("command", 1, &ControllerFrontend::commandCB, this);

    return true;
  }

  virtual void starting(const ros::Time &time) override {
    // should be overrided in a child class
  }

  virtual void update(const ros::Time &time, const ros::Duration &period) override {
    // update the control law with the present states of the joint hardware
    law_->setPositions(jnt_hw_->getPositions());
    law_->setVelocities(jnt_hw_->getVelocities());

    // compute commands based on subscribed setpoints
    const std::map< std::string, double > commands(
        law_->computeCommands(*cmd_buf_.readFromRT(), period));

    // set commands to the joint hardware
    jnt_hw_->setCommands(commands);
  }

  virtual void stopping(const ros::Time &time) override {}

protected:
  virtual void commandCB(const typename Message::ConstPtr &msg) {
    // should be overrided in a child classes
  }

protected:
  boost::scoped_ptr< jsch::AbstractHardware< HardwareInterface > > jnt_hw_;
  boost::scoped_ptr< laws::AbstractLaw > law_;
  rt::RealtimeBuffer< std::map< std::string, double > > cmd_buf_;
  ros::Subscriber cmd_sub_;
};

// =================================================================
// controller frontend specialized for geometry_msgs::Pose messages
template < typename LawT, typename HardwareT >
class PoseControllerFrontend : public ControllerFrontend< geometry_msgs::Pose, LawT, HardwareT > {
private:
  typedef ControllerFrontend< geometry_msgs::Pose, LawT, HardwareT > Base;

public:
  virtual void starting(const ros::Time &time) override {
    Base::law_->starting();

    // reset pose setpoint by the present pose of end effector
    Base::law_->setPositions(Base::jnt_hw_->getPositions());
    Base::law_->setVelocities(Base::jnt_hw_->getVelocities());
    Base::cmd_buf_.writeFromNonRT(Base::law_->getPose());
  }

protected:
  virtual void commandCB(const typename Base::Message::ConstPtr &msg) override {
    // format subscribed pose message
    std::map< std::string, double > cmd;
    const Eigen::AngleAxisd aa(Eigen::Quaterniond(msg->orientation.w, msg->orientation.x,
                                                  msg->orientation.y, msg->orientation.z));
    const Eigen::Vector3d angular(aa.angle() * aa.axis());
    cmd["linear_x"] = msg->position.x;
    cmd["linear_y"] = msg->position.y;
    cmd["linear_z"] = msg->position.z;
    cmd["angular_x"] = angular[0];
    cmd["angular_y"] = angular[1];
    cmd["angular_z"] = angular[2];

    // store formatted setpoint
    Base::cmd_buf_.writeFromNonRT(cmd);
  }
};

// ==================================================================
// controller frontend specialized for geometry_msgs::Twist messages
template < typename LawT, typename HardwareT >
class TwistControllerFrontend : public ControllerFrontend< geometry_msgs::Twist, LawT, HardwareT > {
private:
  typedef ControllerFrontend< geometry_msgs::Twist, LawT, HardwareT > Base;

public:
  virtual void starting(const ros::Time &time) override {
    Base::law_->starting();

    // reset twist command
    std::map< std::string, double > cmd;
    cmd["linear_x"] = 0.;
    cmd["linear_y"] = 0.;
    cmd["linear_z"] = 0.;
    cmd["angular_x"] = 0.;
    cmd["angular_y"] = 0.;
    cmd["angular_z"] = 0.;
    Base::cmd_buf_.writeFromNonRT(cmd);
  }

protected:
  virtual void commandCB(const typename Base::Message::ConstPtr &msg) override {
    // format subscribed twist message
    std::map< std::string, double > cmd;
    cmd["linear_x"] = msg->linear.x;
    cmd["linear_y"] = msg->linear.y;
    cmd["linear_z"] = msg->linear.z;
    cmd["angular_x"] = msg->angular.x;
    cmd["angular_y"] = msg->angular.y;
    cmd["angular_z"] = msg->angular.z;

    // store formatted setpoint
    Base::cmd_buf_.writeFromNonRT(cmd);
  }
};

} // namespace task_space_controllers

#endif