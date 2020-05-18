#ifndef TASK_SPACE_CONTROLLERS_UTILS_HPP
#define TASK_SPACE_CONTROLLERS_UTILS_HPP

#include <map>
#include <string>

#include <ros/assert.h>

namespace task_space_controllers {

// ================================================================================
// utility functions to find a value corresponding the given key in the given map.
// returns the reference of value if found, or die.
template < typename Value >
static Value &findValue(std::map< std::string, Value > &val_map, const std::string &key) {
  const typename std::map< std::string, Value >::iterator it(val_map.find(key));
  ROS_ASSERT_MSG(it != val_map.end(), "findValue(): No value found for the key '%s'", key.c_str());
  return it->second;
}

template < typename Value >
static const Value &findValue(const std::map< std::string, Value > &val_map,
                              const std::string &key) {
  const typename std::map< std::string, Value >::const_iterator it(val_map.find(key));
  ROS_ASSERT_MSG(it != val_map.end(), "findValue(): No value found for the key '%s'", key.c_str());
  return it->second;
}

} // namespace task_space_controllers

#endif