#ifndef SRC_ASSERTIONS_H
#define SRC_ASSERTIONS_H

#include <parameter_assertions/type_traits.h>
#include <ros/ros.h>
#include <type_traits>

namespace assertions
{
enum class NumberAssertionType
{
  POSITIVE,
  NEGATIVE,
  NON_NEGATIVE,
  NON_POSITIVE,
  LESS_THAN_EQ_ONE,
  ABS_LESS_THAN_EQ_ONE,
};

class Asserter
{
public:
  template <typename T>
  using AssertionFP = typename std::add_pointer<bool(const T&)>::type;

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
  bool param(const ros::NodeHandle& nh, const std::string& param_name, T& param_var, const T& default_val) const;

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
  template <typename T>
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
  bool getParam(const ros::NodeHandle& nh, const std::string& param_name, T& param_var) const;

  template <typename T, typename std::enable_if<type_traits::is_number<T>::value, T>::type* = nullptr>
  bool getParam(const ros::NodeHandle& nh, const std::string& param_name, T& param_var,
                std::vector<NumberAssertionType> assertions) const;

  template <typename T, typename type_traits::disable_if<type_traits::is_number<T>::value, T>::type* = nullptr>
  bool getParam(const ros::NodeHandle& nh, const std::string& param_name, T& param_var,
                std::vector<NumberAssertionType> assertions) const;

private:
  template <typename T, typename type_traits::disable_if<type_traits::is_vector<T>::value, T>::type* = nullptr>
  void warnDefaultWithMessage(const std::string& node_namespace, const std::string& variable_name, const T& variable,
                              const std::string& message) const;

  template <typename V, typename std::enable_if<type_traits::is_vector<V>::value, V>::type* = nullptr>
  void warnDefaultWithMessage(const std::string& node_namespace, const std::string& variable_name, const V& variable,
                              const std::string& message) const;

  template <typename T, typename type_traits::disable_if<type_traits::is_vector<T>::value, T>::type* = nullptr>
  bool passesAssertion(const T& variable, std::vector<NumberAssertionType> assertions) const;

  template <typename V, typename std::enable_if<type_traits::is_vector<V>::value, V>::type* = nullptr>
  bool passesAssertion(const V& variable, std::vector<NumberAssertionType> assertions) const;

  template <typename T>
  AssertionFP<T> getAssertionFunction(NumberAssertionType assertion_type) const;

  void fail() const;

  bool exit_on_failure_;
};
}  // namespace assertions

#endif  // SRC_ASSERTIONS_H
