#include <parameter_assertions/assertions.h>

namespace assertions
{
Asserter::Asserter(bool exit_on_failure) noexcept : exit_on_failure_{ exit_on_failure }
{}

template <typename T, typename disable_if<is_vector<T>::value, T>::type*>
void Asserter::warnDefaultWithMessage(const std::string& node_namespace, const std::string& variable_name,
                                      const T& variable, const std::string& message) const
{
  ROS_WARN_STREAM("[" << node_namespace << "] " << variable_name << message << ". Continuing with default values "
                      << variable);
}

template <typename V,typename std::enable_if<is_vector<V>::value, V>::type*>
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
bool Asserter::param(const ros::NodeHandle& nh, const std::string& param_name, T& param_val, const T& default_val) const
{
  if (!nh.param(param_name, param_val, default_val))
  {
    warnDefaultWithMessage(nh.getNamespace(), param_name, default_val, " is not set");
    return false;
  }
  return true;
}

template<typename T>
T Asserter::param(const ros::NodeHandle& nh, const std::string& param_name, const T& default_val) const
{
  T param_val;
  param(nh, param_name, param_val, std::move(default_val));
  return param_val;
}

template <typename T>
void Asserter::getParam(const ros::NodeHandle& nh, const std::string& param_name, T& param_val) const
{
  if (!nh.getParam(param_name, param_val))
  {
    ROS_FATAL_STREAM("[" << nh.getNamespace() << "] Missing parameter " << param_name << ". Exiting...");

    if (exit_on_failure_)
    {
      std::exit(-1);
    }
    else
    {
      ros::shutdown();
    }
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
                                               const std::vector<std::string>& variable, const std::string& message) const;
template void Asserter::warnDefaultWithMessage(const std::string& node_namespace, const std::string& variable_name,
                                               const std::vector<double>& variable, const std::string& message) const;
template void Asserter::warnDefaultWithMessage(const std::string& node_namespace, const std::string& variable_name,
                                               const std::vector<float>& variable, const std::string& message) const;
template void Asserter::warnDefaultWithMessage(const std::string& node_namespace, const std::string& variable_name,
                                               const std::vector<int>& variable, const std::string& message) const;
template void Asserter::warnDefaultWithMessage(const std::string& node_namespace, const std::string& variable_name,
                                               const std::vector<bool>& variable, const std::string& message) const;

template void Asserter::getParam(const ros::NodeHandle& nh, const std::string& param_name, std::string& param_val) const;
template void Asserter::getParam(const ros::NodeHandle& nh, const std::string& param_name, double& param_val) const;
template void Asserter::getParam(const ros::NodeHandle& nh, const std::string& param_name, float& param_val) const;
template void Asserter::getParam(const ros::NodeHandle& nh, const std::string& param_name, int& param_val) const;
template void Asserter::getParam(const ros::NodeHandle& nh, const std::string& param_name, bool& param_val) const;
template void Asserter::getParam(const ros::NodeHandle& nh, const std::string& param_name, std::vector<std::string>& param_val) const;
template void Asserter::getParam(const ros::NodeHandle& nh, const std::string& param_name, std::vector<double>& param_val) const;
template void Asserter::getParam(const ros::NodeHandle& nh, const std::string& param_name, std::vector<float>& param_val) const;
template void Asserter::getParam(const ros::NodeHandle& nh, const std::string& param_name, std::vector<int>& param_val) const;
template void Asserter::getParam(const ros::NodeHandle& nh, const std::string& param_name, std::vector<bool>& param_val) const;

template bool Asserter::param(const ros::NodeHandle& nh, const std::string& param_name, std::string& param_val, const std::string& default_val) const;
template bool Asserter::param(const ros::NodeHandle& nh, const std::string& param_name, double& param_val, const double& default_val) const;
template bool Asserter::param(const ros::NodeHandle& nh, const std::string& param_name, float& param_val, const float& default_val) const;
template bool Asserter::param(const ros::NodeHandle& nh, const std::string& param_name, int& param_val, const int& default_val) const;
template bool Asserter::param(const ros::NodeHandle& nh, const std::string& param_name, bool& param_val, const bool& default_val) const;
template bool Asserter::param(const ros::NodeHandle& nh, const std::string& param_name, std::vector<std::string>& param_val, const std::vector<std::string>& default_val) const;
template bool Asserter::param(const ros::NodeHandle& nh, const std::string& param_name, std::vector<double>& param_val, const std::vector<double>& default_val) const;
template bool Asserter::param(const ros::NodeHandle& nh, const std::string& param_name, std::vector<float>& param_val, const std::vector<float>& default_val) const;
template bool Asserter::param(const ros::NodeHandle& nh, const std::string& param_name, std::vector<int>& param_val, const std::vector<int>& default_val) const;
template bool Asserter::param(const ros::NodeHandle& nh, const std::string& param_name, std::vector<bool>& param_val, const std::vector<bool>& default_val) const;

template std::string Asserter::param(const ros::NodeHandle& nh, const std::string& param_name, const std::string& default_val) const;
template double Asserter::param(const ros::NodeHandle& nh, const std::string& param_name, const double& default_val) const;
template float Asserter::param(const ros::NodeHandle& nh, const std::string& param_name, const float& default_val) const;
template int Asserter::param(const ros::NodeHandle& nh, const std::string& param_name, const int& default_val) const;
template bool Asserter::param(const ros::NodeHandle& nh, const std::string& param_name, const bool& default_val) const;
template std::vector<std::string> Asserter::param(const ros::NodeHandle& nh, const std::string& param_name, const std::vector<std::string>& default_val) const;
template std::vector<double> Asserter::param(const ros::NodeHandle& nh, const std::string& param_name, const std::vector<double>& default_val) const;
template std::vector<float> Asserter::param(const ros::NodeHandle& nh, const std::string& param_name, const std::vector<float>& default_val) const;
template std::vector<int> Asserter::param(const ros::NodeHandle& nh, const std::string& param_name, const std::vector<int>& default_val) const;
template std::vector<bool> Asserter::param(const ros::NodeHandle& nh, const std::string& param_name, const std::vector<bool>& default_val) const;
}

