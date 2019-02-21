#ifndef LIBADA_UTIL_HPP_
#define LIBADA_UTIL_HPP_

#include <Eigen/Dense>
#include <aikido/trajectory/Interpolated.hpp>
#include <aikido/trajectory/Spline.hpp>
#include <aikido/trajectory/Trajectory.hpp>
#include <ros/ros.h>

#include "libada/Ada.hpp"

namespace ada {
namespace util {

/// Displays a message and waits for the user to press the enter key.
/// Terminates if 'n' is pressed.
/// \param[in] msg The message to display.
/// \param[in] ada Ada currently being active.
void waitForUser(const std::string& msg, const std::shared_ptr<Ada>& ada);

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
/// \param[in] xTolerance
/// \param[in] yTolerance
/// \param[in] zTolerance
/// \param[in] rollTolerance
/// \param[in] pitchTolerance
/// \param[in] yawTolerance
Eigen::MatrixXd createBwMatrixForTSR(
    double xTolerance,
    double yTolerance,
    double zTolerenace,
    double rollTolerance,
    double pitchTolerance,
    double yawTolerance);

} // namespace util
} // namespace ada

#include "detail/util-impl.hpp"

#endif
