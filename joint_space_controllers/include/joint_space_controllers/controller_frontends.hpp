#ifndef JOINT_SPACE_CONTROLLERS_CONTROLLER_FRONTENDS_HPP
#define JOINT_SPACE_CONTROLLERS_CONTROLLER_FRONTENDS_HPP

#include <map>
#include <string>

#include <controller_interface/controller.h>
#include <joint_space_controllers/hardware/abstract_hardware.hpp>
#include <joint_space_controllers/laws/abstract_law.hpp>
#include <joint_space_controllers/namespace_aliases.hpp>
#include <joint_space_controllers/utils.hpp>
#include <realtime_tools/realtime_buffer.h>
#include <ros/console.h>
#include <ros/duration.h>
#include <ros/names.h>
#include <ros/node_handle.h>
#include <ros/subscriber.h>
#include <ros/time.h>
#include <std_msgs/Float64.h>

#include <boost/bind.hpp>
#include <boost/foreach.hpp>
#include <boost/optional.hpp>
#include <boost/scoped_ptr.hpp>

namespace joint_space_controllers {

// ==========================================================================
// controller frontend subscribing joint setpoints & passing them to backend
template < typename MessageT, typename LawT, typename HardwareT >
class ControllerFrontend : public ci::Controller< typename HardwareT::HardwareInterface > {
public:
  typedef MessageT Message;
  typedef LawT Law;
  typedef HardwareT Hardware;
  typedef typename Hardware::HardwareInterface HardwareInterface;

protected:
  struct RealtimeSubscriber {
    // in kinetic, rt::RealtimeBuffer<> does not have a const copy constructor
    //   --> RealtimeSubscriber cannot have a const copy constructor
    //   --> BOOST_FOREACH(NameToSubscriber::value_type &kv, name_to_sub_) does not compile
    // boost::optional<> is workaround to enable a const copy constructor.
    // actually, buf is always available.
    boost::optional< rt::RealtimeBuffer< double > > buf;
    ros::Subscriber sub;
  };
  typedef std::map< std::string, RealtimeSubscriber > NameToSubscriber;

public:
  ControllerFrontend() {}

  virtual ~ControllerFrontend() {}

  // required interfaces as a Controller

  virtual bool init(HardwareInterface *hw_iface, ros::NodeHandle &controller_nh) override {
    // init the controller backends
    hw_.reset(new Hardware());
    if (!hw_->init(hw_iface, controller_nh)) {
      ROS_ERROR("Float64Controller::init(): Failed to init the joint hardware");
      return false;
    }
    law_.reset(new Law());
    if (!law_->init(controller_nh)) {
      ROS_ERROR("Float64Controller::init(): Failed to init the control law");
      return false;
    }

    // subscribe commands for controlled joints
    BOOST_FOREACH (const std::string &name, hw_->getNames()) {
      RealtimeSubscriber &sub(name_to_sub_[name]);
      sub.buf.emplace();
      sub.sub = controller_nh.subscribe< std_msgs::Float64 >(
          ros::names::append(name, "command"), 1,
          boost::bind(&ControllerFrontend::commandCB, this, _1, sub.buf.get_ptr()));
    }

    return true;
  }

  virtual void starting(const ros::Time &time) override {
    // should be implemented in a child class
  }

  virtual void update(const ros::Time &time, const ros::Duration &period) override {
    // update the law based on present states of the hardware
    law_->setPositions(hw_->getPositions());
    law_->setVelocities(hw_->getVelocities());

    // populate setpoints from subscribed commands
    std::map< std::string, double > setpoints;
    BOOST_FOREACH (typename NameToSubscriber::value_type &kv, name_to_sub_) {
      const std::string &name(kv.first);
      RealtimeSubscriber &sub(kv.second);
      // copy position setpoint from the buffer
      setpoints[name] = *sub.buf->readFromRT();
    }

    // compute commands for the hardware
    const std::map< std::string, double > commands(law_->computeCommands(setpoints, period));

    // set commands to the hardware
    hw_->setCommands(commands);
  }

  virtual void stopping(const ros::Time &time) override {}

protected:
  virtual void commandCB(const typename Message::ConstPtr &msg,
                         rt::RealtimeBuffer< double > *const buf) {
    // should be implemented in child class
  }

protected:
  boost::scoped_ptr< hardware::AbstractHardware< HardwareInterface > > hw_;
  boost::scoped_ptr< laws::AbstractLaw > law_;
  NameToSubscriber name_to_sub_;
};

template < typename LawT, typename HardwareT >
class EffortControllerFrontend : public ControllerFrontend< std_msgs::Float64, LawT, HardwareT > {
private:
  typedef ControllerFrontend< std_msgs::Float64, LawT, HardwareT > Base;

public:
  virtual void starting(const ros::Time &time) override {
    BOOST_FOREACH (typename Base::NameToSubscriber::value_type &kv, Base::name_to_sub_) {
      typename Base::RealtimeSubscriber &sub(kv.second);
      // reset effort command
      sub.buf->writeFromNonRT(0.);
    }

    Base::law_->starting();
  }

protected:
  virtual void commandCB(const typename Base::Message::ConstPtr &msg,
                         rt::RealtimeBuffer< double > *const buf) override {
    buf->writeFromNonRT(msg->data);
  }
};

template < typename LawT, typename HardwareT >
class VelocityControllerFrontend : public ControllerFrontend< std_msgs::Float64, LawT, HardwareT > {
private:
  typedef ControllerFrontend< std_msgs::Float64, LawT, HardwareT > Base;

public:
  virtual void starting(const ros::Time &time) override {
    BOOST_FOREACH (typename Base::NameToSubscriber::value_type &kv, Base::name_to_sub_) {
      typename Base::RealtimeSubscriber &sub(kv.second);
      // reset velocity command
      sub.buf->writeFromNonRT(0.);
    }

    Base::law_->starting();
  }

protected:
  virtual void commandCB(const typename Base::Message::ConstPtr &msg,
                         rt::RealtimeBuffer< double > *const buf) override {
    buf->writeFromNonRT(msg->data);
  }
};

template < typename LawT, typename HardwareT >
class PositionControllerFrontend : public ControllerFrontend< std_msgs::Float64, LawT, HardwareT > {
private:
  typedef ControllerFrontend< std_msgs::Float64, LawT, HardwareT > Base;

public:
  virtual void starting(const ros::Time &time) override {
    const std::map< std::string, double > positions(Base::hw_->getPositions());

    BOOST_FOREACH (typename Base::NameToSubscriber::value_type &kv, Base::name_to_sub_) {
      const std::string &name(kv.first);
      typename Base::RealtimeSubscriber &sub(kv.second);
      // reset position command by present position
      sub.buf->writeFromNonRT(findValue(positions, name));
    }

    Base::law_->starting();
  }

protected:
  virtual void commandCB(const typename Base::Message::ConstPtr &msg,
                         rt::RealtimeBuffer< double > *const buf) override {
    buf->writeFromNonRT(msg->data);
  }
};

} // namespace joint_space_controllers

#endif