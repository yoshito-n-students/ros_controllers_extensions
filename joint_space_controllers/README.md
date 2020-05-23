# joint_sapce_controllers
Dynamices-based controllers for manipulators, which take setpoints in the joint space

## Control laws
### Position-to-effort law
The position-to-effort law expects a general rigid body dynamics system, whose equations of motion are given in the form below.

<img src="https://latex.codecogs.com/gif.latex?M(q)\ddot{q}+C(q,\dot{q})+g(q)=\tau" />

where <img src="https://latex.codecogs.com/gif.latex?q" />, <img src="https://latex.codecogs.com/gif.latex?M(\cdot)" />, <img src="https://latex.codecogs.com/gif.latex?C(\cdot)" />, and <img src="https://latex.codecogs.com/gif.latex?g(\cdot)" /> are the joint position vector, inertia matrix, Colioris vector, and gravity vector.

The law computes the joint effort commands <img src="https://latex.codecogs.com/gif.latex?\tau_d" /> to track the joint position setpoints <img src="https://latex.codecogs.com/gif.latex?q_{\textup{sp}}" /> with compensation for the inertia, Colioris, and gravity forces.

<img src="https://latex.codecogs.com/gif.latex?\tau_d=M(q)(\ddot{q}+\textup{PID}(q_{\textup{sp}}-q)+C(q,\dot{q})+g(q)" />

where <img src="https://latex.codecogs.com/gif.latex?\textup{PID}(\cdot)" /> is the control input to the system based on a PID controller for the position tracking errors.

<img src="https://latex.codecogs.com/gif.latex?\textup{PID}(e)=K_pe+K_i\int&space;edt+K_d\dot{e}" />

### Position saturation law
The position saturation law saturates position setpoints based on joint limits in robot description parameter.

<img src="https://latex.codecogs.com/gif.latex?q_\textup{sp}\gets\textup{saturate}(q_\textup{sp})" />

where <img src="https://latex.codecogs.com/gif.latex?\textup{saturate}(\cdot)" /> is a saturation function.

### Velocity-to-position law
The velocity-to-position law just converts velocity setpoints to position setpoints by accumulating velocity setpoints.

<img src="https://latex.codecogs.com/gif.latex?q_{\textup{sp}}&space;\gets&space;q_{\textup{sp}}+\dot{q}_{\textup{sp}}\Delta&space;t" />

where <img src="https://latex.codecogs.com/gif.latex?\Delta&space;t" /> is the control time step.

## joint_space_controllers/EffortBasedPositionController
* subscribes position setpoints for each joint
* expects all joints to have hardware_interface/EffortJointInterface
* apply position saturation and position-to-effort laws to the subscribed position setpoints to obtain effort commands for the joints

### <u>Subscribed topics</u>
___<controller_namespace>/<joint_name>/command___ (std_msgs/Float64)
* Position command for each joint

### <u>Parameters</u>
___<controller_namespace>/robot_description___ or ___robot_description___ (string, required)
* Robot description in URDF used in the position saturation & position-to-effort laws

___<controller_namespace>/joints/<joint_name>___ (struct, required)
* PID gains for each joint used in the position-to-effort law
* [control_toolbox](http://wiki.ros.org/control_toolbox)::Pid will be initialized using these parameters

## joint_space_controllers/EffortBasedVelocityController
* subscribes velocity setpoints for each joint
* expects all joints to have hardware_interface/EffortJointInterface
* apply velocity-to-position, position saturation & position-to-effort laws to the subscribed velocity setpoints to obtain effort commands for the joints

### <u>Subscribed topics</u>
___<controller_namespace>/<joint_name>/command___ (std_msgs/Float64)
* Velocity command for each joint

### <u>Parameters</u>
___<controller_namespace>/robot_description___ or ___robot_description___ (string, required)
* Robot description in URDF used in the position saturation & position-to-effort laws

___<controller_namespace>/joints/<joint_name>___ (struct, required)
* PID gains for each joint used in the position-to-effort law
* [control_toolbox](http://wiki.ros.org/control_toolbox)::Pid will be initialized using these parameters

## joint_space_controllers/EffortForwardController
* subscribes effort setpoints and just forward them to joint hardware interfaces
* expects all joints to have hardware_interface/EffortJointInterface

### <u>Subscribed topics</u>
___<controller_namespace>/<joint_name>/command___ (std_msgs/Float64)
* Effort command for each joint

## joint_space_controllers/PositionForwardController
* subscribes position setpoints and just forward them to joint hardware interfaces
* expects all joints to have hardware_interface/PositionJointInterface

### <u>Subscribed topics</u>
___<controller_namespace>/<joint_name>/command___ (std_msgs/Float64)
* Position command for each joint

## joint_space_controllers/VelocityForwardController
* subscribes velocity setpoints and just forward them to joint hardware interfaces
* expects all joints to have hardware_interface/VelocityJointInterface

### <u>Subscribed topics</u>
___<controller_namespace>/<joint_name>/command___ (std_msgs/Float64)
* Velocity command for each joint
