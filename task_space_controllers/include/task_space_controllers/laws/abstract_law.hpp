#ifndef TASK_SPACE_CONTROLLERS_LAWS_ABSTRACT_LAW_HPP
#define TASK_SPACE_CONTROLLERS_LAWS_ABSTRACT_LAW_HPP

#include <map>
#include <string>

#include <joint_space_controllers/laws/abstract_law.hpp>
#include <task_space_controllers/namespace_aliases.hpp>

namespace task_space_controllers {
namespace laws {

// ================================================
// An abstract class for control law in task space
class AbstractLaw : public jscl::AbstractLaw {
public:
  AbstractLaw() {}

  virtual ~AbstractLaw() {}

  // ==========================================
  // get the end effector's pose in task space
  virtual std::map< std::string, double > getPose() const = 0;
};

} // namespace laws
} // namespace task_space_controllers

#endif