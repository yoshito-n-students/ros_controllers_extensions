# task_sapce_controllers
Dynamices-based controllers for manipulators, which take setpoints in the task space

## Control laws
### Acceleration integration law
The acceleration integration law converts the acceleration of the end-effector in the task space to the acceleration of the joints in the joint space.

<img src="https://latex.codecogs.com/gif.latex?\ddot{q}_r=J^\dagger(\ddot{x}_r-\dot{J}\dot{q})" />

where <img src="https://latex.codecogs.com/gif.latex?q" />, <img src="https://latex.codecogs.com/gif.latex?x" />, <img src="https://latex.codecogs.com/gif.latex?J" /> are the pose vector of the end-effector, the position vector of the joints, and the Jacobian matrix. <img src="https://latex.codecogs.com/gif.latex?\bullet_r" /> and <img src="https://latex.codecogs.com/gif.latex?\bullet^\dagger" /> are reference values and pseudo-inverse (<img src="https://latex.codecogs.com/gif.latex?\bullet^\dagger=\bullet^T(\bullet\bullet^T)^{-1}" />).

### Pose-to-acceleration law
The pose-to-acceleration law converts reference pose to acceleration in the task space by using a PID controller.

<img src="https://latex.codecogs.com/gif.latex?\textup{PID}(e)=K_pe+K_i\int&space;edt+K_d\dot{e}" />

where <img src="https://latex.codecogs.com/gif.latex?e=\ddot{x}_r^\ddot{x}" /> is the pose tracking error.

### Pose saturation law
The pose saturation law generates reference pose by saturating pose setpoints based on predefined limits.

<img src="https://latex.codecogs.com/gif.latex?x_r=\textup{saturate}(x_\textup{sp})" />

where <img src="https://latex.codecogs.com/gif.latex?\textup{saturate}(\cdot)" /> is a saturation function.

### Twist-to-pose law
The twist-to-pose law just converts reference twist to pose by accumulating reference twist.

<img src="https://latex.codecogs.com/gif.latex?x_r&space;\gets&space;x_r+\dot{x}_r\Delta&space;t" />

where <img src="https://latex.codecogs.com/gif.latex?\Delta&space;t" /> is the control time step.

### Velocity integration law
The velocity integration law converts the twist of the end-effector in the task space to the velocity of the joints in the joint space.

<img src="https://latex.codecogs.com/gif.latex?\dot{q}_r=J^\dagger\dot{x}_r" />

### Pose-to-twist law
The pose-to-twist law just converts reference pose to twist by differentiating reference pose.

<img src="https://latex.codecogs.com/gif.latex?\dot{x}_r=\frac{x_r(t)-x_r(t-\Delta&space;t)}{\Delta&space;t}" />

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