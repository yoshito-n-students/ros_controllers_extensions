#ifndef TASK_SPACE_CONTROLLERS_TASK_SPACE_CONTROLLERS_HPP
#define TASK_SPACE_CONTROLLERS_TASK_SPACE_CONTROLLERS_HPP

#include <joint_space_controllers/hardware/joint_hardware.hpp>
#include <joint_space_controllers/laws/acceleration_to_effort_law.hpp>
#include <task_space_controllers/controller_frontends.hpp>
#include <task_space_controllers/laws/acceleration_integration_law.hpp>
#include <task_space_controllers/laws/model_based_law.hpp>
#include <task_space_controllers/laws/pose_saturation_law.hpp>
#include <task_space_controllers/laws/pose_to_acceleration_law.hpp>
#include <task_space_controllers/laws/pose_to_twist_law.hpp>
#include <task_space_controllers/laws/twist_to_pose_law.hpp>
#include <task_space_controllers/laws/velocity_integration_law.hpp>
#include <task_space_controllers/namespace_aliases.hpp>

namespace task_space_controllers {

// control laws
namespace laws {
typedef AccelerationIntegrationLaw< jscl::AccelerationToEffortLaw< ModelBasedLaw<> > >
    AccelerationToEffortLaw;
typedef PoseSaturationLaw< PoseToAccelerationLaw< AccelerationToEffortLaw > > PoseToEffortLaw;
typedef TwistToPoseLaw< PoseToEffortLaw > TwistToEffortLaw;
} // namespace laws

// joint hardware types
namespace hardware {
typedef jsch::JointHardware< hi::EffortJointInterface > EffortJointHardware;
typedef jsch::JointHardware< hi::VelocityJointInterface > VelocityJointHardware;
typedef jsch::JointHardware< hi::PositionJointInterface > PositionJointHardware;
} // namespace hardware

// twist controllers
typedef TwistControllerFrontend< laws::VelocityIntegrationLaw<>, hardware::VelocityJointHardware >
    VelocityBasedTwistController;
typedef TwistControllerFrontend< laws::TwistToEffortLaw, hardware::EffortJointHardware >
    EffortBasedTwistController;

// pose controllers
typedef PoseControllerFrontend< laws::PoseToTwistLaw< laws::VelocityIntegrationLaw<> >,
                                hardware::VelocityJointHardware >
    VelocityBasedPoseController;
typedef PoseControllerFrontend< laws::PoseToEffortLaw, hardware::EffortJointHardware >
    EffortBasedPoseController;

} // namespace task_space_controllers

#endif