#ifndef LIBADA_UTIL_HPP_
#define LIBADA_UTIL_HPP_

#include <Eigen/Dense>
#include <aikido/trajectory/Interpolated.hpp>
#include <aikido/trajectory/Spline.hpp>
#include <aikido/trajectory/Trajectory.hpp>
#include <ros/ros.h>

namespace ada {
namespace util {

/// Displays a message and waits for the user to press the enter key.
/// \param[in] The message to display.
/// \return False if the user entered 'n'.
bool waitForUser(const std::string& msg);

/// Loads and returns a ros parameter.
/// Throws a runtime_error if the parameter is not set.
/// \param[in] paramName The name of the parameter.
/// \param[in] nodeHandle Handle of the ros node.
/// \return The value of the ros parameter.
template <class T>
T getRosParam(const std::string& paramName, const ros::NodeHandle& nh);

/// Convenience function to create an Eigen Isometry3D based on position and
/// rotation.
/// \param[in] x, y, z Position
/// \param[in] roll, pitch, yaw Rotation
/// \return The transform.
Eigen::Isometry3d createIsometry(
    double x,
    double y,
    double z,
    double roll = 0,
    double pitch = 0,
    double yaw = 0);

/// Convenience function to create an Eigen Isometry3D based on position and
/// rotation.
/// \param[in] vec Position and rotation in a vector [x,y,z,roll,pitch,yaw]
/// \return The transform.
Eigen::Isometry3d createIsometry(const std::vector<double>& vec);

/// Convenience function to create the Bw Matrix that is needed for TSRs.
/// TODO (avk): Clarify what Bw matrix is. Why only these tolerances.
/// \param[in] horizontalTolerance
/// \param[in] verticalTolerance
/// \param[in] yawMin
/// \param[in] yawMax
Eigen::MatrixXd createBwMatrixForTSR(
    double horizontalTolerance,
    double verticalTolerance,
    double yawMin,
    double yawMax);

/// TODO (avk): Docstring.
aikido::trajectory::UniqueSplinePtr posePostprocessingForSO2(
    const aikido::trajectory::Spline& input);

} // namespace util
} // namespace ada

#include "detail/util-impl.hpp"

#endif
