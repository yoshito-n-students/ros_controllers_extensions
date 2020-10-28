#ifndef INTEGER_CONTROLLERS_COMMON_NAMESPACES_HPP
#define INTEGER_CONTROLLERS_COMMON_NAMESPACES_HPP

namespace controller_interface {}

namespace hardware_interface {
namespace internal {}
} // namespace hardware_interface

namespace hardware_interface_extensions {}

namespace realtime_tools {}

namespace integer_controllers {
namespace ci = controller_interface;
namespace hi = hardware_interface;
namespace hii = hi::internal;
namespace hie = hardware_interface_extensions;
namespace rt = realtime_tools;
} // namespace integer_controllers

#endif