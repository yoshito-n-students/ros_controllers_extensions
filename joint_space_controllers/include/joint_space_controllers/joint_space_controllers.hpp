#ifndef JOINT_SPACE_CONTROLLERS_JOINT_SPACE_CONTROLLERS_HPP
#define JOINT_SPACE_CONTROLLERS_JOINT_SPACE_CONTROLLERS_HPP

#include <joint_space_controllers/controller_frontends.hpp>
#include <joint_space_controllers/hardware/joint_hardware.hpp>
#include <joint_space_controllers/laws/acceleration_to_effort_law.hpp>
#include <joint_space_controllers/laws/forward_command_law.hpp>
#include <joint_space_controllers/laws/position_saturation_law.hpp>
#include <joint_space_controllers/laws/position_to_acceleration_law.hpp>
#include <joint_space_controllers/laws/velocity_to_position_law.hpp>
#include <joint_space_controllers/namespace_aliases.hpp>

namespace joint_space_controllers {

// control laws
namespace laws {
typedef PositionSaturationLaw< PositionToAccelerationLaw< AccelerationToEffortLaw<> > >
    PositionToEffortLaw;
typedef VelocityToPositionLaw< PositionToEffortLaw > VelocityToEffortLaw;
} // namespace laws

// joint hardware types
namespace hardware {
typedef JointHardware< hi::EffortJointInterface > EffortJointHardware;
typedef JointHardware< hi::VelocityJointInterface > VelocityJointHardware;
typedef JointHardware< hi::PositionJointInterface > PositionJointHardware;
} // namespace hardware

// effort controllers
typedef EffortControllerFrontend< laws::ForwardCommandLaw<>, hardware::EffortJointHardware >
    EffortForwardController;

// velocity controllers
typedef VelocityControllerFrontend< laws::ForwardCommandLaw<>, hardware::VelocityJointHardware >
    VelocityForwardController;
typedef VelocityControllerFrontend< laws::VelocityToEffortLaw, hardware::EffortJointHardware >
    EffortBasedVelocityController;

// position controllers
typedef PositionControllerFrontend< laws::ForwardCommandLaw<>, hardware::PositionJointHardware >
    PositionForwardController;
typedef PositionControllerFrontend< laws::PositionToEffortLaw, hardware::EffortJointHardware >
    EffortBasedPositionController;

} // namespace joint_space_controllers

#endif