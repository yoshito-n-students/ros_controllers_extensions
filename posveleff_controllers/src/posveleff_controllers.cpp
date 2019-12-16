#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>
#include <posveleff_controllers/joint_posveleff_controller.hpp>

PLUGINLIB_EXPORT_CLASS(posveleff_controllers::JointPosVelEffController,
                       controller_interface::ControllerBase);