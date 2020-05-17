#ifndef JOINT_SPACE_CONTROLLERS_HARDWARE_JOINT_HARDWARE_HPP
#define JOINT_SPACE_CONTROLLERS_HARDWARE_JOINT_HARDWARE_HPP

#include <map>
#include <string>
#include <vector>

#include <hardware_interface/hardware_interface.h> // for hi::HardwareInterfaceException
#include <hardware_interface/joint_command_interface.h>
#include <joint_space_controllers/hardware/abstract_hardware.hpp>
#include <joint_space_controllers/namespace_aliases.hpp>
#include <joint_space_controllers/utils.hpp>
#include <ros/console.h>
#include <ros/node_handle.h>
#include <urdf/model.h>

#include <boost/foreach.hpp>

namespace joint_space_controllers {
namespace hardware {

// ===============================================================================================
// common base class for backend controllers that just forward commands to joint hardware handles
template < typename HardwareInterfaceT >
class JointHardware : public AbstractHardware< HardwareInterfaceT > {
public:
  typedef HardwareInterfaceT HardwareInterface;

public:
  JointHardware() {}

  virtual ~JointHardware() {}

  virtual bool init(HardwareInterface *const hw_iface, const ros::NodeHandle &param_nh) override {
    urdf::Model robot_desc;
    if (!robot_desc.initString(getRobotDescription(param_nh))) {
      ROS_ERROR("JointHardware::init(): Failed to parse robot description as an URDF");
      return false;
    }

    typedef std::map< std::string, urdf::JointSharedPtr > NameToJoint;
    BOOST_FOREACH (const NameToJoint::value_type &kv, robot_desc.joints_) {
      const std::string &name(kv.first);
      const urdf::Joint &joint(*kv.second);
      // skip a joint with no degrees of freedom
      if (joint.type == urdf::Joint::UNKNOWN || joint.type == urdf::Joint::FIXED) {
        continue;
      }
      // command handle to the hardware
      try {
        joints_.push_back(hw_iface->getHandle(name));
      } catch (const hi::HardwareInterfaceException &ex) {
        ROS_ERROR_STREAM("JointHardware::init(): Failed to get the command "
                         "handle of the joint '"
                         << name << "': " << ex.what());
        return false;
      }
    }

    return true;
  }

  virtual std::vector< std::string > getNames() const override {
    std::vector< std::string > names;
    BOOST_FOREACH (const HardwareHandle &joint, joints_) { names.push_back(joint.getName()); }
    return names;
  }

  virtual std::map< std::string, double > getPositions() const override {
    std::map< std::string, double > positions;
    BOOST_FOREACH (const HardwareHandle &joint, joints_) {
      positions[joint.getName()] = joint.getPosition();
    }
    return positions;
  }

  virtual std::map< std::string, double > getVelocities() const override {
    std::map< std::string, double > velocities;
    BOOST_FOREACH (const HardwareHandle &joint, joints_) {
      velocities[joint.getName()] = joint.getVelocity();
    }
    return velocities;
  }

  virtual void setCommands(const std::map< std::string, double > &commands) override {
    BOOST_FOREACH (HardwareHandle &joint, joints_) {
      joint.setCommand(findValue(commands, joint.getName()));
    }
  }

private:
  typedef typename HardwareInterface::ResourceHandleType HardwareHandle;
  std::vector< HardwareHandle > joints_;
};

} // namespace hardware
} // namespace joint_space_controllers

#endif