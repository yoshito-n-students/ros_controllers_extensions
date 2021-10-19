# ros_controllers_extensions
extensions for ros_controllers

## [empty_controller package](empty_controller)
### empty_controller/EmptyController
* controller which does completely nothing
* useful when notifing hardware to something by just swithing to this controller

## [posvel_controllers package](posvel_controllers)
### posvel_controllers/JointPosVelController
#### <u>Subscribed topics</u>
___<controller_namespace>/command___ (std_msgs/Float64MultiArray)
* must have exactly 2 data elements
* data[0] & data[1] correspond position & velocity commands
* layout will be ignored
* subscribed when ___separete_command___ is false

___<controller_namespace>/pos_command___ (std_msgs/Float64)
* subscribed when ___separete_command___ is true

___<controller_namespace>/vel_command___ (std_msgs/Float64)
* subscribed when ___separete_command___ is true

#### <u>Parameters</u>
___<controller_namespace>/joint___ (string, required)
* joint name to be managed by the controller

___<controller_namespace>/separate_command___ (bool, default: true)
* if true, position & velocity commands are separately subscribed

## [posveleff_controllers package](posveleff_controllers)
### posveleff_controllers/JointPosVelEffController
#### <u>Subscribed topics</u>
___<controller_namespace>/command___ (std_msgs/Float64MultiArray)
* must have exactly 3 data elements
* data[0], data[1] & data[2] correspond position, velocity & effort commands
* layout will be ignored
* subscribed when ___separete_command___ is false

___<controller_namespace>/pos_command___ (std_msgs/Float64)
* subscribed when ___separete_command___ is true

___<controller_namespace>/vel_command___ (std_msgs/Float64)
* subscribed when ___separete_command___ is true

___<controller_namespace>/eff_command___ (std_msgs/Float64)
* subscribed when ___separete_command___ is true

#### <u>Parameters</u>
___<controller_namespace>/joint___ (string, required)
* joint name to be managed by the controller

___<controller_namespace>/separate_command___ (bool, default: true)
* if true, position, velocity & effort commands are separately subscribed