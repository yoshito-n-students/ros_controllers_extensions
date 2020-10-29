#ifndef INTEGER_CONTROLLERS_INTEGER_COMMAND_CONTROLLERS_HPP
#define INTEGER_CONTROLLERS_INTEGER_COMMAND_CONTROLLERS_HPP

#include <string>

#include <controller_interface/controller.h>
#include <hardware_interface/hardware_interface.h> // for HardwareInterfaceException
#include <hardware_interface/internal/demangle_symbol.h>
#include <hardware_interface_extensions/integer_interface.hpp>
#include <integer_controllers/common_namespaces.hpp>
#include <realtime_tools/realtime_buffer.h>
#include <ros/console.h>
#include <ros/duration.h>
#include <ros/node_handle.h>
#include <ros/subscriber.h>
#include <ros/time.h>

namespace integer_controllers {

///////////////////////////////
// Command controller template

template < typename InterfaceT >
class IntegerCommandController : public ci::Controller< InterfaceT > {
public:
  typedef InterfaceT Interface;

private:
  typedef IntegerCommandController< Interface > This;
  typedef typename Interface::Handle Handle;
  typedef typename Handle::Data Data;
  typedef typename Handle::Msg Msg;

public:
  IntegerCommandController() {}

  virtual ~IntegerCommandController() { cmd_sub_.shutdown(); }

  virtual bool init(Interface *hw, ros::NodeHandle &nh) {
    std::string handle_name;
    if (!nh.getParam("handle", handle_name)) {
      ROS_ERROR_STREAM(hii::demangledTypeName< This >()
                       << "::init(): Failed to get required param " << nh.resolveName("handle"));
      return false;
    }

    try {
      cmd_handle_ = hw->getHandle(handle_name);
    } catch (const hi::HardwareInterfaceException &ex) {
      ROS_ERROR_STREAM(hii::demangledTypeName< This >()
                       << "::init(): Failed to get hardware handle: " << ex.what());
      return false;
    }

    cmd_sub_ = nh.subscribe< Msg >("command", 1, &This::commandCB, this);

    return true;
  }

  virtual void starting(const ros::Time & /*time*/) {
    cmd_buf_.writeFromNonRT(cmd_handle_.getState());
  }

  virtual void update(const ros::Time & /*time*/, const ros::Duration & /*period*/) {
    cmd_handle_.setCommand(*cmd_buf_.readFromRT());
  }

private:
  void commandCB(const typename Msg::ConstPtr &msg) { cmd_buf_.writeFromNonRT(msg->data); }

private:
  Handle cmd_handle_;
  rt::RealtimeBuffer< Data > cmd_buf_;
  ros::Subscriber cmd_sub_;
};

////////////////////////////
// Command controller types

// signed 8-64
typedef IntegerCommandController< hie::Int8Interface > Int8Controller;
typedef IntegerCommandController< hie::Int16Interface > Int16Controller;
typedef IntegerCommandController< hie::Int32Interface > Int32Controller;
typedef IntegerCommandController< hie::Int64Interface > Int64Controller;
// unsigned 8-64
typedef IntegerCommandController< hie::UInt8Interface > UInt8Controller;
typedef IntegerCommandController< hie::UInt16Interface > UInt16Controller;
typedef IntegerCommandController< hie::UInt32Interface > UInt32Controller;
typedef IntegerCommandController< hie::UInt64Interface > UInt64Controller;

} // namespace integer_controllers

#endif