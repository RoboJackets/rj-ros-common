#include <parameter_assertions/assertions.h>

namespace assertions
{
Asserter::Asserter(bool exit_on_failure) noexcept : exit_on_failure_{ exit_on_failure }
{
}

template <typename T, typename type_traits::disable_if<type_traits::is_vector<T>::value, T>::type*>
void Asserter::warnDefaultWithMessage(const std::string& node_namespace, const std::string& variable_name,
                                      const T& variable, const std::string& message) const
{
  ROS_WARN_STREAM("[" << node_namespace << "] " << variable_name << message << ". Continuing with default values "
                      << variable);
}

template <typename V, typename std::enable_if<type_traits::is_vector<V>::value, V>::type*>
void Asserter::warnDefaultWithMessage(const std::string& node_namespace, const std::string& variable_name,
                                      const V& variable, const std::string& message) const
{
  std::stringstream ss;

  ss << "{ ";
  if (variable.size() > 0)
  {
    ss << variable[0];

    for (size_t i = 0; i < variable.size(); i++)
    {
      ss << ", " << variable[i];
    }
  }
  ss << " }";

  warnDefaultWithMessage(node_namespace, variable_name, ss.str(), message);
}

template <typename T>
bool Asserter::param(const ros::NodeHandle& nh, const std::string& param_name, T& param_var, const T& default_val) const
{
  if (!nh.param(param_name, param_var, default_val))
  {
    warnDefaultWithMessage(nh.getNamespace(), param_name, default_val, " is not set");
    return false;
  }
  return true;
}

template <typename T>
T Asserter::param(const ros::NodeHandle& nh, const std::string& param_name, const T& default_val) const
{
  T param_var;
  param(nh, param_name, param_var, std::move(default_val));
  return param_var;
}

template <typename T>
bool Asserter::getParam(const ros::NodeHandle& nh, const std::string& param_name, T& param_var) const
{
  if (!nh.getParam(param_name, param_var))
  {
    ROS_ERROR_STREAM("[" << nh.getNamespace() << "] " << param_name << " is not set. Exiting...");
    fail();
    return false;
  }

  return true;
}

template <typename T, typename std::enable_if<type_traits::is_number<T>::value, T>::type*>
bool Asserter::getParam(const ros::NodeHandle& nh, const std::string& param_name, T& param_var,
                        const std::vector<NumberAssertionType>& assertions) const
{
  if (getParam(nh, param_name, param_var))
  {
    if (!passesAssertion(param_var, assertions))
    {
      ROS_ERROR_STREAM("[" << nh.getNamespace() << "] " << param_name << " is not set. Exiting...");
      fail();
      return false;
    }
  }

  return true;
}

template <typename T, typename type_traits::disable_if<type_traits::is_number<T>::value, T>::type*>
bool Asserter::getParam(const ros::NodeHandle& nh, const std::string& param_name, T& param_var,
                        const std::vector<NumberAssertionType>& /*assertions*/) const
{
  ROS_WARN_STREAM("[" << nh.getNamespace() << "] An assertion was set for " << param_name
                      << " but it is not a number. Continuing without assertions.");

  return getParam(nh, param_name, param_var);
}

template <typename T, typename type_traits::disable_if<type_traits::is_vector<T>::value, T>::type*>
bool Asserter::passesAssertion(const T& variable, const std::vector<NumberAssertionType>& assertions) const
{
  for (const auto& assertion : assertions)
  {
    if (!getAssertionFunction<T>(assertion)(variable))
    {
      return false;
    }
  }
  return true;
}

template <typename V, typename std::enable_if<type_traits::is_vector<V>::value, V>::type*>
bool Asserter::passesAssertion(const V& variable, std::vector<NumberAssertionType> assertions) const
{
  for (const auto& element : variable)
  {
    if (!passesAssertion(element, assertions))
    {
      return false;
    }
  }
  return true;
}

template <typename T>
Asserter::AssertionFP<T> Asserter::getAssertionFunction(NumberAssertionType assertion_type) const
{
  switch (assertion_type)
  {
    case NumberAssertionType::POSITIVE:
      return [](const T& param) { return param > 0; };
    case NumberAssertionType::NON_NEGATIVE:
      return [](const T& param) { return param >= 0; };
    case NumberAssertionType::NEGATIVE:
      return [](const T& param) { return param < 0; };
    case NumberAssertionType::NON_POSITIVE:
      return [](const T& param) { return param <= 0; };
    case NumberAssertionType::LESS_THAN_EQ_ONE:
      return [](const T& param) { return param <= 1; };
    case NumberAssertionType::ABS_LESS_THAN_EQ_ONE:
      return [](const T& param) { return std::abs(param) <= 1; };
    default:
      ROS_ERROR_STREAM("default case reached in Asserter::getAssertionFunction even though match was exhaustive");
      return [](const T& /*param*/) { return false; };
  }
}

void Asserter::fail() const
{
  if (exit_on_failure_)
  {
    std::exit(-1);
  }
  else
  {
    ros::shutdown();
  }
}

template void Asserter::warnDefaultWithMessage(const std::string& node_namespace, const std::string& variable_name,
                                               const std::string& variable, const std::string& message) const;
template void Asserter::warnDefaultWithMessage(const std::string& node_namespace, const std::string& variable_name,
                                               const double& variable, const std::string& message) const;
template void Asserter::warnDefaultWithMessage(const std::string& node_namespace, const std::string& variable_name,
                                               const float& variable, const std::string& message) const;
template void Asserter::warnDefaultWithMessage(const std::string& node_namespace, const std::string& variable_name,
                                               const int& variable, const std::string& message) const;
template void Asserter::warnDefaultWithMessage(const std::string& node_namespace, const std::string& variable_name,
                                               const bool& variable, const std::string& message) const;
template void Asserter::warnDefaultWithMessage(const std::string& node_namespace, const std::string& variable_name,
                                               const std::vector<std::string>& variable,
                                               const std::string& message) const;
template void Asserter::warnDefaultWithMessage(const std::string& node_namespace, const std::string& variable_name,
                                               const std::vector<double>& variable, const std::string& message) const;
template void Asserter::warnDefaultWithMessage(const std::string& node_namespace, const std::string& variable_name,
                                               const std::vector<float>& variable, const std::string& message) const;
template void Asserter::warnDefaultWithMessage(const std::string& node_namespace, const std::string& variable_name,
                                               const std::vector<int>& variable, const std::string& message) const;
template void Asserter::warnDefaultWithMessage(const std::string& node_namespace, const std::string& variable_name,
                                               const std::vector<bool>& variable, const std::string& message) const;

template bool Asserter::getParam(const ros::NodeHandle& nh, const std::string& param_name,
                                 std::string& param_var) const;
template bool Asserter::getParam(const ros::NodeHandle& nh, const std::string& param_name, double& param_var) const;
template bool Asserter::getParam(const ros::NodeHandle& nh, const std::string& param_name, float& param_var) const;
template bool Asserter::getParam(const ros::NodeHandle& nh, const std::string& param_name, int& param_var) const;
template bool Asserter::getParam(const ros::NodeHandle& nh, const std::string& param_name, bool& param_var) const;
template bool Asserter::getParam(const ros::NodeHandle& nh, const std::string& param_name,
                                 std::vector<std::string>& param_var) const;
template bool Asserter::getParam(const ros::NodeHandle& nh, const std::string& param_name,
                                 std::vector<double>& param_var) const;
template bool Asserter::getParam(const ros::NodeHandle& nh, const std::string& param_name,
                                 std::vector<float>& param_var) const;
template bool Asserter::getParam(const ros::NodeHandle& nh, const std::string& param_name,
                                 std::vector<int>& param_var) const;
template bool Asserter::getParam(const ros::NodeHandle& nh, const std::string& param_name,
                                 std::vector<bool>& param_var) const;

template bool Asserter::getParam(const ros::NodeHandle& nh, const std::string& param_name, std::string& param_var,
                                 const std::vector<NumberAssertionType>& assertions) const;
template bool Asserter::getParam(const ros::NodeHandle& nh, const std::string& param_name, double& param_var,
                                 const std::vector<NumberAssertionType>& assertions) const;
template bool Asserter::getParam(const ros::NodeHandle& nh, const std::string& param_name, float& param_var,
                                 const std::vector<NumberAssertionType>& assertions) const;
template bool Asserter::getParam(const ros::NodeHandle& nh, const std::string& param_name, int& param_var,
                                 const std::vector<NumberAssertionType>& assertions) const;
template bool Asserter::getParam(const ros::NodeHandle& nh, const std::string& param_name, bool& param_var,
                                 const std::vector<NumberAssertionType>& assertions) const;
template bool Asserter::getParam(const ros::NodeHandle& nh, const std::string& param_name,
                                 std::vector<std::string>& param_var,
                                 const std::vector<NumberAssertionType>& assertions) const;
template bool Asserter::getParam(const ros::NodeHandle& nh, const std::string& param_name,
                                 std::vector<double>& param_var,
                                 const std::vector<NumberAssertionType>& assertions) const;
template bool Asserter::getParam(const ros::NodeHandle& nh, const std::string& param_name,
                                 std::vector<float>& param_var,
                                 const std::vector<NumberAssertionType>& assertions) const;
template bool Asserter::getParam(const ros::NodeHandle& nh, const std::string& param_name, std::vector<int>& param_var,
                                 const std::vector<NumberAssertionType>& assertions) const;
template bool Asserter::getParam(const ros::NodeHandle& nh, const std::string& param_name, std::vector<bool>& param_var,
                                 const std::vector<NumberAssertionType>& assertions) const;

template bool Asserter::param(const ros::NodeHandle& nh, const std::string& param_name, std::string& param_var,
                              const std::string& default_val) const;
template bool Asserter::param(const ros::NodeHandle& nh, const std::string& param_name, double& param_var,
                              const double& default_val) const;
template bool Asserter::param(const ros::NodeHandle& nh, const std::string& param_name, float& param_var,
                              const float& default_val) const;
template bool Asserter::param(const ros::NodeHandle& nh, const std::string& param_name, int& param_var,
                              const int& default_val) const;
template bool Asserter::param(const ros::NodeHandle& nh, const std::string& param_name, bool& param_var,
                              const bool& default_val) const;
template bool Asserter::param(const ros::NodeHandle& nh, const std::string& param_name,
                              std::vector<std::string>& param_var, const std::vector<std::string>& default_val) const;
template bool Asserter::param(const ros::NodeHandle& nh, const std::string& param_name, std::vector<double>& param_var,
                              const std::vector<double>& default_val) const;
template bool Asserter::param(const ros::NodeHandle& nh, const std::string& param_name, std::vector<float>& param_var,
                              const std::vector<float>& default_val) const;
template bool Asserter::param(const ros::NodeHandle& nh, const std::string& param_name, std::vector<int>& param_var,
                              const std::vector<int>& default_val) const;
template bool Asserter::param(const ros::NodeHandle& nh, const std::string& param_name, std::vector<bool>& param_var,
                              const std::vector<bool>& default_val) const;

template std::string Asserter::param(const ros::NodeHandle& nh, const std::string& param_name,
                                     const std::string& default_val) const;
template double Asserter::param(const ros::NodeHandle& nh, const std::string& param_name,
                                const double& default_val) const;
template float Asserter::param(const ros::NodeHandle& nh, const std::string& param_name,
                               const float& default_val) const;
template int Asserter::param(const ros::NodeHandle& nh, const std::string& param_name, const int& default_val) const;
template bool Asserter::param(const ros::NodeHandle& nh, const std::string& param_name, const bool& default_val) const;
template std::vector<std::string> Asserter::param(const ros::NodeHandle& nh, const std::string& param_name,
                                                  const std::vector<std::string>& default_val) const;
template std::vector<double> Asserter::param(const ros::NodeHandle& nh, const std::string& param_name,
                                             const std::vector<double>& default_val) const;
template std::vector<float> Asserter::param(const ros::NodeHandle& nh, const std::string& param_name,
                                            const std::vector<float>& default_val) const;
template std::vector<int> Asserter::param(const ros::NodeHandle& nh, const std::string& param_name,
                                          const std::vector<int>& default_val) const;
template std::vector<bool> Asserter::param(const ros::NodeHandle& nh, const std::string& param_name,
                                           const std::vector<bool>& default_val) const;
}  // namespace assertions
