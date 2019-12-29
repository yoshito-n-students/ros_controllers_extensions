#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>
#include <sensor_state_controllers/sensor_state_controllers.hpp>

PLUGINLIB_EXPORT_CLASS(sensor_state_controllers::BatteryStateController,
                       controller_interface::ControllerBase);