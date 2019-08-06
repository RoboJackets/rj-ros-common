#ifndef SRC_ASSERTIONS_H
#define SRC_ASSERTIONS_H

#include <type_traits>
#include <ros/ros.h>

namespace assertions
{
template <bool B, typename T = void>
using disable_if = std::enable_if<!B, T>;

template <bool B, class T = void>
using enable_if_t = typename std::enable_if<B, T>::type;

template <typename T>
struct is_vector : public std::false_type
{
};

template <typename T, typename A>
struct is_vector<std::vector<T, A>> : public std::true_type
{
};

class Asserter
{
 public:
  /**
   * Class for performing paramter assertions.
   *
   * @param exit_on_failure If true, will call std::exit(-1) on failures.
   */
  explicit Asserter(bool exit_on_failure = false) noexcept;

  /**
   * Calls nh.param with the passed in values. Logs with ROS_WARN_STREAM if the default value is used. Returns the
   * result of nh.param
   *
   * @tparam T
   * @param nh ros::NodeHandle to use
   * @param param_name
   * @param param_val
   * @param default_val
   * @return
   */
  template <typename T>
  bool param(const ros::NodeHandle& nh, const std::string& param_name, T& param_val, const T& default_val) const;

  /**
   * Calls nh.param with the passed in values. Logs with ROS_WARN_STREAM if the default value is used. Returns the
   * result of nh.param
   *
   * @tparam T
   * @param nh ros::NodeHandle to use
   * @param param_name
   * @param default_val
   * @return
   */
  template<typename T>
  [[nodiscard]] T param(const ros::NodeHandle& nh, const std::string& param_name, const T& default_val) const;

  /**
   * Calls nh.getParam with the passed in values. If getParam returns false, calls ros::shutdown if exit_on_failure_
   * is false, otherwise does std::exit(-1).
   *
   * @tparam T
   * @param nh ros::NodeHandle to use
   * @param param_name
   * @param param_val
   */
  template <typename T>
  void getParam(const ros::NodeHandle& nh, const std::string& param_name, T& param_val) const;

 private:
  template <typename T, typename disable_if<is_vector<T>::value, T>::type* = nullptr>
  void warnDefaultWithMessage(const std::string& node_namespace, const std::string& variable_name,
                              const T& variable, const std::string& message) const;

  template <typename V,typename std::enable_if<is_vector<V>::value, V>::type* = nullptr>
  void warnDefaultWithMessage(const std::string& node_namespace, const std::string& variable_name,
                              const V& variable, const std::string& message) const;

  bool exit_on_failure_;
};
}

#endif //SRC_ASSERTIONS_H
