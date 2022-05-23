#include "libada/util.hpp"
#include <aikido/common/Spline.hpp>
#include <aikido/distance/DistanceMetric.hpp>
#include <aikido/distance/defaults.hpp>
#include <aikido/statespace/CartesianProduct.hpp>
#include <aikido/statespace/dart/MetaSkeletonStateSpace.hpp>
#include <aikido/trajectory/Interpolated.hpp>
#include <aikido/trajectory/util.hpp>
#include <dart/common/StlHelpers.hpp>
#include <stdlib.h>

namespace ada {
namespace util {

//==============================================================================
void waitForUser(const std::string& msg, const std::shared_ptr<Ada>& ada)
{
  ROS_INFO((msg + "\nPress [y] to continue, [n] to terminate.").c_str());
  char input;
  std::cin.clear();
  std::cin >> input;
  if (input == 'n')
  {
    ROS_INFO_STREAM("Aborting with user input " << input);
    ada->deactivateExecutor();
    exit(1);
  }
}

//==============================================================================
Eigen::Isometry3d createIsometry(
    double x, double y, double z, double roll, double pitch, double yaw)
{
  Eigen::Isometry3d isometry = Eigen::Isometry3d::Identity();
  isometry.translation() = Eigen::Vector3d(x, y, z);
  Eigen::Matrix3d rotation;
  rotation = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())
             * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
             * Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
  isometry.linear() = rotation;

  return isometry;
}

//==============================================================================
Eigen::Isometry3d createIsometry(const std::vector<double>& vec)
{
  if (vec.size() < 6)
  {
    throw std::runtime_error(
        "Expected size of vector is 6, received " + vec.size());
  }
  return createIsometry(vec[0], vec[1], vec[2], vec[3], vec[4], vec[5]);
}

//==============================================================================
Eigen::MatrixXd createBwMatrixForTSR(
    double xTolerance,
    double yTolerance,
    double zTolerance,
    double rollTolerance,
    double pitchTolerance,
    double yawTolerance)
{
  Eigen::MatrixXd bw = Eigen::Matrix<double, 6, 2>::Zero();
  bw(0, 0) = -xTolerance;
  bw(0, 1) = xTolerance;
  bw(1, 0) = -yTolerance;
  bw(1, 1) = yTolerance;
  bw(2, 0) = -zTolerance;
  bw(2, 1) = zTolerance;
  bw(3, 0) = -rollTolerance;
  bw(3, 1) = rollTolerance;
  bw(4, 0) = -pitchTolerance;
  bw(4, 1) = pitchTolerance;
  bw(5, 0) = -yawTolerance;
  bw(5, 1) = yawTolerance;

  return bw;
}

//==============================================================================
hardware_interface::JointCommandModes modeFromString(std::string str)
{
  if (str == "BEGIN")
    return hardware_interface::JointCommandModes::BEGIN;
  if (str == "POSITION" || str == "MODE_POSITION")
    return hardware_interface::JointCommandModes::MODE_POSITION;
  if (str == "VELOCITY" || str == "MODE_VELOCITY")
    return hardware_interface::JointCommandModes::MODE_VELOCITY;
  if (str == "EFFORT" || str == "MODE_EFFORT")
    return hardware_interface::JointCommandModes::MODE_EFFORT;
  if (str == "NOMODE")
    return hardware_interface::JointCommandModes::NOMODE;
  if (str == "ESTOP" || str == "EMERGENCY_STOP")
    return hardware_interface::JointCommandModes::EMERGENCY_STOP;
  if (str == "SWITCHING")
    return hardware_interface::JointCommandModes::SWITCHING;

  return hardware_interface::JointCommandModes::ERROR;
}

//=============================================================================
std::vector<ExecutorDetails> loadExecutorsDetailsFromParameter(const ros::NodeHandle &nodeHandle, const std::string &parameterName) 
{
  using XmlRpc::XmlRpcValue;

  static const std::vector<ExecutorDetails> emptyResult;

  XmlRpcValue executorsDetailsXml;
  if (!nodeHandle.getParam(parameterName, executorsDetailsXml)) {
    ROS_ERROR_STREAM("Parameter '" << nodeHandle.getNamespace()
                                   << "/executors' is required.");
  }

  if (executorsDetailsXml.getType() != XmlRpcValue::TypeArray) {
    ROS_ERROR_STREAM("Parameter '" << nodeHandle.getNamespace()
                                   << "/executors' is not an array.");
    return emptyResult;
  }

  std::vector<ExecutorDetails> output;
  for (int i = 0; i < executorsDetailsXml.size(); ++i) {
    ExecutorDetails executorDetails;
    auto &executorDetailsXml = executorsDetailsXml[i];

    if (executorDetailsXml.getType() == XmlRpcValue::TypeStruct) {
      
      auto &idXml = executorDetailsXml["id"];
      if (idXml.getType() != XmlRpcValue::TypeString) {
        ROS_ERROR_STREAM("Parameter '" << nodeHandle.getNamespace()
                                       << "/executors[" << i
                                       << "]/id' is not a string.");
        return emptyResult;
      }
      executorDetails.mId = static_cast<std::string>(idXml);
      
      auto &typeXml = executorDetailsXml["type"];
      if (typeXml.getType() != XmlRpcValue::TypeString) {
        ROS_ERROR_STREAM("Parameter '" << nodeHandle.getNamespace()
                                       << "/executors[" << i
                                       << "]/type' is not a string.");
        return emptyResult;
      }
      executorDetails.mType = static_cast<std::string>(typeXml);

      if(executorDetails.mType != "TRAJECTORY")
      {
        auto &modeXml = executorDetailsXml["mode"];
        if (modeXml.getType() != XmlRpcValue::TypeString) {
          ROS_ERROR_STREAM("Parameter '" << nodeHandle.getNamespace()
                                         << "/executors[" << i
                                         << "]/mode' is not a string.");
          return emptyResult;
        }
        executorDetails.mMode = static_cast<std::string>(modeXml);
      }

      auto &controllerXml = executorDetailsXml["controller"];
      if (controllerXml.getType() != XmlRpcValue::TypeString) {
        ROS_ERROR_STREAM("Parameter '" << nodeHandle.getNamespace()
                                       << "/executors[" << i
                                       << "]/controller' is not a string.");
        return emptyResult;
      }
      executorDetails.mController = static_cast<std::string>(controllerXml);

    } else {
      ROS_ERROR_STREAM("Parameter '" << nodeHandle.getNamespace() << "/executors["
                                     << i << "]' is not a struct.");
      return emptyResult;
    }

    output.emplace_back(executorDetails);
  }

  return output;
}

} // namespace util
} // namespace ada
