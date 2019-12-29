#ifndef SENSOR_STATE_CONTROLLERS_SENSOR_STATE_CONTROLLERS_HPP
#define SENSOR_STATE_CONTROLLERS_SENSOR_STATE_CONTROLLERS_HPP

#include <string>
#include <vector>

#include <controller_interface/controller.h>
#include <hardware_interface/internal/demangle_symbol.h>
#include <hardware_interface_extensions/sensor_state_interface.hpp>
#include <realtime_tools/realtime_publisher.h>
#include <ros/console.h>
#include <ros/duration.h>
#include <ros/node_handle.h>
#include <ros/time.h>

#include <boost/foreach.hpp>
#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>

namespace sensor_state_controllers {

template < typename Interface >
class SensorStateController : public controller_interface::Controller< Interface > {
public:
  SensorStateController() {}

  virtual bool init(Interface *hw, ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) {
    namespace hii = hardware_interface::internal;

    // load params
    interval_ = ros::Duration(controller_nh.param("interval", 10.));

    // launch publishers for each sensor
    BOOST_FOREACH (const std::string &sensor_name, hw->getNames()) {
      sensors_.push_back(hw->getHandle(sensor_name));
      publishers_.push_back(boost::make_shared< Publisher >(root_nh, sensor_name, 1));
      deadlines_.push_back(ros::Time(0)); // zero time means no deadline

      ROS_INFO_STREAM("SensorStateController::init(): Initialzed a publisher (msg: '"
                      << hii::demangledTypeName< Data >() << "', topic: '"
                      << root_nh.resolveName(sensor_name) << "')");
    }

    // kind warning if no sensors found
    if (sensors_.empty()) {
      ROS_WARN_STREAM("SensorStateController::init(): No sensor handles found on '"
                      << hii::demangledTypeName< Interface >() << "'. Will do nothing.");
    }

    return true;
  }

  virtual void starting(const ros::Time &time) {
    // set publish deadlines as the current time
    BOOST_FOREACH (ros::Time &deadline, deadlines_) { deadline = time; }
  }

  virtual void update(const ros::Time &time, const ros::Duration &period) {
    for (std::size_t i = 0; i < sensors_.size(); ++i) {
      // check if deadline has come
      if (deadlines_[i].isZero() || deadlines_[i] > time) {
        continue;
      }
      // check if can publish
      if (!publishers_[i]->trylock()) {
        ROS_WARN_STREAM(
            "SensorStateController::update(): Failed to own the publisher associated with '"
            << sensors_[i].getName() << "'. Will retry in the next cycle.");
        continue;
      }
      // publish message
      publishers_[i]->msg_ = sensors_[i].getData();
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
  typedef typename Interface::Handle Handle;
  typedef typename Handle::Data Data;
  typedef realtime_tools::RealtimePublisher< Data > Publisher;

  std::vector< Handle > sensors_;
  std::vector< boost::shared_ptr< Publisher > > publishers_;
  std::vector< ros::Time > deadlines_;
  ros::Duration interval_;
};

typedef SensorStateController< hardware_interface_extensions::BatteryStateInterface >
    BatteryStateController;

} // namespace sensor_state_controllers

#endif