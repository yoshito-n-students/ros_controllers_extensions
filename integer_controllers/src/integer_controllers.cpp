#include <controller_interface/controller_base.h>
#include <integer_controllers/integer_command_controllers.hpp>
#include <integer_controllers/integer_state_controllers.hpp>
#include <pluginlib/class_list_macros.h>

/////////////////////
// State controllers

// singed 8-64
PLUGINLIB_EXPORT_CLASS(integer_controllers::Int8StateController,
                       controller_interface::ControllerBase);
PLUGINLIB_EXPORT_CLASS(integer_controllers::Int16StateController,
                       controller_interface::ControllerBase);
PLUGINLIB_EXPORT_CLASS(integer_controllers::Int32StateController,
                       controller_interface::ControllerBase);
PLUGINLIB_EXPORT_CLASS(integer_controllers::Int64StateController,
                       controller_interface::ControllerBase);
// unsinged 8-64
PLUGINLIB_EXPORT_CLASS(integer_controllers::UInt8StateController,
                       controller_interface::ControllerBase);
PLUGINLIB_EXPORT_CLASS(integer_controllers::UInt16StateController,
                       controller_interface::ControllerBase);
PLUGINLIB_EXPORT_CLASS(integer_controllers::UInt32StateController,
                       controller_interface::ControllerBase);
PLUGINLIB_EXPORT_CLASS(integer_controllers::UInt64StateController,
                       controller_interface::ControllerBase);

///////////////////////
// Command controllers

// signed 8-64
PLUGINLIB_EXPORT_CLASS(integer_controllers::Int8Controller, controller_interface::ControllerBase);
PLUGINLIB_EXPORT_CLASS(integer_controllers::Int16Controller, controller_interface::ControllerBase);
PLUGINLIB_EXPORT_CLASS(integer_controllers::Int32Controller, controller_interface::ControllerBase);
PLUGINLIB_EXPORT_CLASS(integer_controllers::Int64Controller, controller_interface::ControllerBase);
// unsigned 8-64
PLUGINLIB_EXPORT_CLASS(integer_controllers::UInt8Controller, controller_interface::ControllerBase);
PLUGINLIB_EXPORT_CLASS(integer_controllers::UInt16Controller, controller_interface::ControllerBase);
PLUGINLIB_EXPORT_CLASS(integer_controllers::UInt32Controller, controller_interface::ControllerBase);
PLUGINLIB_EXPORT_CLASS(integer_controllers::UInt64Controller, controller_interface::ControllerBase);