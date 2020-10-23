#ifndef BYTE_ARRAY_CONTROLLERS_BYTE_ARRAY_STATE_CONTROLLERS_HPP
#define BYTE_ARRAY_CONTROLLERS_BYTE_ARRAY_STATE_CONTROLLERS_HPP

#include <string>
#include <vector>

#include <byte_array_controllers/common_namespaces.hpp>
#include <controller_interface/controller.h>
#include <hardware_interface_extensions/byte_array_interface.hpp>
#include <realtime_tools/realtime_publisher.h>
#include <ros/console.h>
#include <ros/duration.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <std_msgs/ByteMultiArray.h>

#include <boost/foreach.hpp>
#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>

namespace byte_array_controllers {

class ByteArrayStateController
    : public controller_interface::Controller< hie::ByteArrayStateInterface > {
public:
  ByteArrayStateController() {}

  virtual bool init(hie::ByteArrayStateInterface *hw, ros::NodeHandle &root_nh,
                    ros::NodeHandle &controller_nh) {
    // load params
    interval_ = ros::Duration(controller_nh.param("interval", 1.));

    // launch publishers for each sensor
    BOOST_FOREACH (const std::string &name, hw->getNames()) {
      hw_handles_.push_back(hw->getHandle(name));
      publishers_.push_back(boost::make_shared< Publisher >(root_nh, name, 1));
      deadlines_.push_back(ros::Time(0)); // zero time means no deadline

      ROS_INFO_STREAM("ByteArrayStateController::init(): Initialzed a publisher (topic: '"
                      << root_nh.resolveName(name) << "')");
    }

    // kind warning if no sensors found
    if (hw_handles_.empty()) {
      ROS_WARN_STREAM("ByteArrayStateController::init(): No hw handles found on "
                      "ByteArrayStateInterface. Will do nothing.");
    }

    return true;
  }

  virtual void starting(const ros::Time &time) {
    // set publish deadlines as the current time
    BOOST_FOREACH (ros::Time &deadline, deadlines_) { deadline = time; }
  }

  virtual void update(const ros::Time &time, const ros::Duration &period) {
    for (std::size_t i = 0; i < hw_handles_.size(); ++i) {
      // check if deadline has come
      if (deadlines_[i].isZero() || deadlines_[i] > time) {
        continue;
      }
      // check if can publish
      if (!publishers_[i]->trylock()) {
        ROS_WARN_STREAM(
            "ByteArrayStateController::update(): Failed to own the publisher associated with '"
            << hw_handles_[i].getName() << "'. Will retry in the next cycle.");
        continue;
      }
      // publish message
      publishers_[i]->msg_ = *hw_handles_[i].getData().toMsg();
      publishers_[i]->unlockAndPublish();
      // set next deadline
      deadlines_[i] += interval_;
    }
  }

  virtual void stopping(const ros::Time &time) {
    // unset publish deadlines
    BOOST_FOREACH (ros::Time &deadline, deadlines_) { deadline = ros::Time(0); }
  }

private:
  typedef realtime_tools::RealtimePublisher< std_msgs::ByteMultiArray > Publisher;

  std::vector< hie::ByteArrayHandle > hw_handles_;
  std::vector< boost::shared_ptr< Publisher > > publishers_;
  std::vector< ros::Time > deadlines_;
  ros::Duration interval_;
};

} // namespace byte_array_controllers

#endif