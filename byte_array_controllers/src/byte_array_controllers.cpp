#include <byte_array_controllers/byte_array_command_controller.hpp>
#include <byte_array_controllers/byte_array_state_controller.hpp>
#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(byte_array_controllers::ByteArrayStateController,
                       controller_interface::ControllerBase);
PLUGINLIB_EXPORT_CLASS(byte_array_controllers::ByteArrayCommandController,
                       controller_interface::ControllerBase);