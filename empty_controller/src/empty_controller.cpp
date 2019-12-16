#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>
#include <empty_controller/empty_controller.hpp>

PLUGINLIB_EXPORT_CLASS(empty_controller::EmptyController, controller_interface::ControllerBase);