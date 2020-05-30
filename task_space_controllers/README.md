# task_sapce_controllers
Dynamices-based controllers for manipulators, which take setpoints in the task space

## Control laws
### Acceleration integration law

### Pose-to-acceleration law

### Pose saturation law

### Twist-to-pose law

### Velocity integration law

### Pose-to-twist law

## task_space_controllers/EffortBasedPoseController
* subscribes pose setpoints in the task space
* expects all joints to have hardware_interface/EffortJointInterface
* apply the pose saturation, pose-to-acceleration, acceleration integration and acceleration-to-effort laws to the subscribed pose setpoints to obtain effort commands for the joints

### <u>Subscribed topics</u>
___<controller_namespace>/command___ (geometry_msgs/Pose)
* Pose command in the task space

### <u>Parameters</u>
___<controller_namespace>/robot_description___ or ___robot_description___ (string, required)
* Robot description in URDF used in the position saturation & position-to-effort laws

___<controller_namespace>/task_space/{linear, angular}___ (struct, required)
* Limits and PID gains for degree of freedom in the task space used in the pose saturation and pose-to-acceleration laws, respectively
* Limits are optional and PID gains are required
* Limits will be loaded by [joint_limits_interface](http://wiki.ros.org/joint_limits_interface)::getJointLimits()
* PID gains will be loaded by [control_toolbox](http://wiki.ros.org/control_toolbox)::Pid::init()

## task_space_controllers/EffortBasedTwistController
* subscribes twist setpoints in the task space
* expects all joints to have hardware_interface/EffortJointInterface
* apply the twist-to-pose, pose saturation, pose-to-acceleration, acceleration integration and acceleration-to-effort laws to the subscribed twist setpoints to obtain effort commands for the joints

### <u>Subscribed topics</u>
___<controller_namespace>/command___ (geometry_msgs/Twist)
* Twist command in the task space

### <u>Parameters</u>
___<controller_namespace>/robot_description___ or ___robot_description___ (string, required)
* Robot description in URDF used in the position saturation & position-to-effort laws

___<controller_namespace>/task_space/{linear, angular}___ (struct, required)
* Limits and PID gains for degree of freedom in the task space used in the pose saturation and pose-to-acceleration laws, respectively
* Limits are optional and PID gains are required
* Limits will be loaded by [joint_limits_interface](http://wiki.ros.org/joint_limits_interface)::getJointLimits()
* PID gains will be loaded by [control_toolbox](http://wiki.ros.org/control_toolbox)::Pid::init()

## task_space_controllers/VelocityBasedTwistController
* subscribes twist setpoints in the task space
* expects all joints to have hardware_interface/VelocityJointInterface
* apply the velocity integration law to the subscribed twist setpoints to obtain velocity commands for the joints

### <u>Subscribed topics</u>
___<controller_namespace>/command___ (geometry_msgs/Twist)
* Twist command in the task space

### <u>Parameters</u>
___<controller_namespace>/robot_description___ or ___robot_description___ (string, required)
* Robot description in URDF used in the position saturation & position-to-effort laws

## task_space_controllers/VelocityBasedPoseController
* subscribes pose setpoints in the task space
* expects all joints to have hardware_interface/VelocityJointInterface
* apply the pose saturation, pose-to-twist and velocity integration laws to the subscribed pose setpoints to obtain velocity commands for the joints

### <u>Subscribed topics</u>
___<controller_namespace>/command___ (geometry_msgs/Pose)
* Pose command in the task space

### <u>Parameters</u>
___<controller_namespace>/robot_description___ or ___robot_description___ (string, required)
* Robot description in URDF used in the position saturation & position-to-effort laws

___<controller_namespace>/task_space/{linear, angular}___ (struct, optional)
* Limits for degree of freedom in the task space used in the pose saturation law
* Limits will be loaded by [joint_limits_interface](http://wiki.ros.org/joint_limits_interface)::getJointLimits()