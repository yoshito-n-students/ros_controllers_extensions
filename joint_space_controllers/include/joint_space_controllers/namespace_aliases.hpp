#ifndef JOINT_SPACE_CONTROLLERS_NAMESPACE_ALIASES_HPP
#define JOINT_SPACE_CONTROLLERS_NAMESPACE_ALIASES_HPP

namespace controller_interface {}

namespace control_toolbox {}

namespace dart {
namespace common {}
namespace dynamics {}
namespace math {}
namespace utils {}
} // namespace dart

namespace hardware_interface {}

namespace joint_limits_interface {}

namespace realtime_tools {}

namespace joint_space_controllers {
namespace ci = controller_interface;
namespace ct = control_toolbox;
namespace dc = dart::common;
namespace dd = dart::dynamics;
namespace dm = dart::math;
namespace du = dart::utils;
namespace hi = hardware_interface;
namespace jli = joint_limits_interface;
namespace rt = realtime_tools;
} // namespace joint_space_controllers

#endif