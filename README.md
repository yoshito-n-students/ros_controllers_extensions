# ros_controllers_extensions
extensions for ros_controllers

## [empty_controller package](empty_controller)
### empty_controller/EmptyController
* controller which does completely nothing
* useful when notifing hardware to something by just swithing to this controller

## [joint_space_controllers package](joint_space_controllers)
* dynamics-based controllers for manipulators, which takes setpoints in the joint space
* see [README](joint_space_controllers/README.md) for details

## [posvel_controllers package](posvel_controllers)
### posvel_controllers/JointPosVelController
#### <u>Subscribed topics</u>
___<controller_namespace>/command___ (std_msgs/Float64MultiArray)
* must have exactly 2 data elements
* data[0] & data[1] correspond position & velocity commands
* layout will be ignored

#### <u>Parameters</u>
___<controller_namespace>/joint___
* joint name to be managed by the controller

## [posveleff_controllers package](posveleff_controllers)
### posveleff_controllers/JointPosVelEffController
#### <u>Subscribed topics</u>
___<controller_namespace>/command___ (std_msgs/Float64MultiArray)
* must have exactly 3 data elements
* data[0], data[1] & data[2] correspond position, velocity & effort commands
* layout will be ignored

#### <u>Parameters</u>
___<controller_namespace>/joint___
* joint name to be managed by the controller

## [task_space_controllers package](task_space_controllers)
* dynamics-based controllers for manipulators, which takes setpoints in the task space
* see [README](task_space_controllers/README.md) for details