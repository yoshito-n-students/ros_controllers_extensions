#ifndef INTEGER_CONTROLLERS_INTEGER_STATE_CONTROLLERS_HPP
#define INTEGER_CONTROLLERS_INTEGER_STATE_CONTROLLERS_HPP

#include <cctype>
#include <memory>
#include <string>
#include <vector>

#include <controller_interface/controller.h>
#include <hardware_interface/internal/demangle_symbol.h>
#include <hardware_interface_extensions/integer_interface.hpp>
#include <integer_controllers/common_namespaces.hpp>
#include <realtime_tools/realtime_publisher.h>
#include <ros/console.h>
#include <ros/duration.h>
#include <ros/node_handle.h>
#include <ros/time.h>

namespace integer_controllers {

/////////////////////////////
// State controller template

template < typename InterfaceT >
class IntegerStateController : public ci::Controller< InterfaceT > {
public:
  typedef InterfaceT Interface;

private:
  typedef IntegerStateController< Interface > This;
  typedef typename Interface::Handle Handle;
  typedef typename Handle::Msg Msg;
  typedef rt::RealtimePublisher< Msg > Publisher;

public:
  IntegerStateController() {}

  virtual bool init(Interface *hw, ros::NodeHandle &root_nh,
                    ros::NodeHandle &controller_nh) override {
    // load params
    interval_ = ros::Duration(controller_nh.param("interval", 1.));

    // launch publishers for each sensor
    for (const std::string &hw_name : hw->getNames()) {
      hw_handles_.push_back(hw->getHandle(hw_name));
      const std::string topic_name(toTopicName(hw_name));
      publishers_.push_back(std::make_shared< Publisher >(root_nh, topic_name, 1));
      deadlines_.push_back(ros::Time(0)); // zero time means no deadline

      ROS_INFO_STREAM(hii::demangledTypeName< This >()
                      << "::init(): Initialzed a publisher (msg: '"
                      << hii::demangledTypeName< Msg >() << "', topic: '"
                      << root_nh.resolveName(topic_name) << "')");
    }

    // kind warning if no handles found
    if (hw_handles_.empty()) {
      ROS_WARN_STREAM(hii::demangledTypeName< This >()
                      << "::init(): No " << hii::demangledTypeName< Handle >() << " found on "
                      << hii::demangledTypeName< Interface >() << ". Will do nothing.");
    }

    return true;
  }

  virtual void starting(const ros::Time &time) override {
    // set publish deadlines as the current time
    for (ros::Time &deadline : deadlines_) {
      deadline = time;
    }
  }

  virtual void update(const ros::Time &time, const ros::Duration &period) override {
    for (std::size_t i = 0; i < hw_handles_.size(); ++i) {
      // check if deadline has come
      if (deadlines_[i].isZero() || deadlines_[i] > time) {
        continue;
      }
      // check if can publish
      if (!publishers_[i]->trylock()) {
        ROS_WARN_STREAM(hii::demangledTypeName< This >()
                        << "::update(): Failed to own the publisher associated with '"
                        << hw_handles_[i].getName() << "'. Will retry in the next cycle.");
        continue;
      }
      // publish message
      publishers_[i]->msg_ = hw_handles_[i].toMsg();
      publishers_[i]->unlockAndPublish();
      // set next deadline
      deadlines_[i] += interval_;
    }
  }

  virtual void stopping(const ros::Time &time) override {
    // unset publish deadlines
    for (ros::Time &deadline : deadlines_) {
      deadline = ros::Time(0);
    }
  }

private:
  // convert "Device/State Item 01" to "device/state_item_01",
  // which sounds valid and natural as a topic name
  static std::string toTopicName(const std::string &in) {
    std::string out;
    for (const std::string::value_type c : in) {
      if (std::isupper(c)) {
        out.push_back(std::tolower(c));
      } else if (std::islower(c) || std::isdigit(c) || c == '/') {
        out.push_back(c);
      } else {
        out.push_back('_');
      }
    }
    return out;
  }

private:
  std::vector< Handle > hw_handles_;
  std::vector< std::shared_ptr< Publisher > > publishers_;
  std::vector< ros::Time > deadlines_;
  ros::Duration interval_;
};

//////////////////////////
// State controller types

// signed 8-64
typedef IntegerStateController< hie::Int8StateInterface > Int8StateController;
typedef IntegerStateController< hie::Int16StateInterface > Int16StateController;
typedef IntegerStateController< hie::Int32StateInterface > Int32StateController;
typedef IntegerStateController< hie::Int64StateInterface > Int64StateController;
// unsigned 8-64
typedef IntegerStateController< hie::UInt8StateInterface > UInt8StateController;
typedef IntegerStateController< hie::UInt16StateInterface > UInt16StateController;
typedef IntegerStateController< hie::UInt32StateInterface > UInt32StateController;
typedef IntegerStateController< hie::UInt64StateInterface > UInt64StateController;

} // namespace integer_controllers

#endif