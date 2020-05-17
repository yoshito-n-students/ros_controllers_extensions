#include <controller_interface/controller_base.h>
#include <joint_space_controllers/joint_space_controllers.hpp>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(joint_space_controllers::EffortForwardController,
                       controller_interface::ControllerBase);

PLUGINLIB_EXPORT_CLASS(joint_space_controllers::VelocityForwardController,
                       controller_interface::ControllerBase);
PLUGINLIB_EXPORT_CLASS(joint_space_controllers::EffortBasedVelocityController,
                       controller_interface::ControllerBase);

PLUGINLIB_EXPORT_CLASS(joint_space_controllers::PositionForwardController,
                       controller_interface::ControllerBase);
PLUGINLIB_EXPORT_CLASS(joint_space_controllers::EffortBasedPositionController,
                       controller_interface::ControllerBase);