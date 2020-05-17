#ifndef JOINT_SPACE_CONTROLLERS_HARDWARE_ABSTRACT_HARDWARE_HPP
#define JOINT_SPACE_CONTROLLERS_HARDWARE_ABSTRACT_HARDWARE_HPP

#include <map>
#include <string>
#include <vector>

#include <ros/node_handle.h>

namespace joint_space_controllers {
namespace hardware {

// =========================================
// An abstract class for hardware of joints
template < typename HardwareInterfaceT > class AbstractHardware {
public:
  typedef HardwareInterfaceT HardwareInterface;

public:
  AbstractHardware() {}

  virtual ~AbstractHardware() {}

  // ======================================================================
  // initialize (ex. find joint hardware handles from the given interface)
  virtual bool init(HardwareInterface *const hw_iface, const ros::NodeHandle &param_nh) = 0;

  // =====================================================
  // get names of joint hardware managed by this instance
  virtual std::vector< std::string > getNames() const = 0;

  // ==========================================
  // get names and positions of managed joints
  virtual std::map< std::string, double > getPositions() const = 0;

  // ===========================================
  // get names and velocities of managed joints
  virtual std::map< std::string, double > getVelocities() const = 0;

  // ================================
  // set commands for managed joints
  virtual void setCommands(const std::map< std::string, double > &commands) = 0;
};

} // namespace hardware
} // namespace joint_space_controllers

#endif