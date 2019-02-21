
namespace ada {
namespace util {
//==============================================================================
template <class T>
T getRosParam(const std::string& paramName, const ros::NodeHandle& nh)
{
  T value;
  if (!nh.getParam(paramName, value))
  {
    throw std::runtime_error("Failed to load ros parameter " + paramName);
  }
  return value;
}
} // namespace util
} // namespace ada
