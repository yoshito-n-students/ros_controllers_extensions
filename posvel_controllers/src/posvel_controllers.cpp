#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>
#include <posvel_controllers/joint_posvel_controller.hpp>

PLUGINLIB_EXPORT_CLASS(posvel_controllers::JointPosVelController, controller_interface::ControllerBase);