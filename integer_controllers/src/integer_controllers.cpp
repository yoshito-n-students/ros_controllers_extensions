#include <controller_interface/controller_base.h>
#include <integer_controllers/integer_command_controllers.hpp>
#include <integer_controllers/integer_state_controllers.hpp>
#include <pluginlib/class_list_macros.h>

// State controllers
PLUGINLIB_EXPORT_CLASS(integer_controllers::Int32StateController,
                       controller_interface::ControllerBase);

// Command controllers
PLUGINLIB_EXPORT_CLASS(integer_controllers::Int32Controller, controller_interface::ControllerBase);