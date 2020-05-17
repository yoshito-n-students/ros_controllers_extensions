#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>
#include <task_space_controllers/task_space_controllers.hpp>

PLUGINLIB_EXPORT_CLASS(task_space_controllers::VelocityBasedTwistController,
                       controller_interface::ControllerBase);
PLUGINLIB_EXPORT_CLASS(task_space_controllers::EffortBasedTwistController,
                       controller_interface::ControllerBase);

PLUGINLIB_EXPORT_CLASS(task_space_controllers::VelocityBasedPoseController,
                       controller_interface::ControllerBase);
PLUGINLIB_EXPORT_CLASS(task_space_controllers::EffortBasedPoseController,
                       controller_interface::ControllerBase);