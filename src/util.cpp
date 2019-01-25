#include "libada/util.hpp"

#include <aikido/common/Spline.hpp>
#include <aikido/distance/DistanceMetric.hpp>
#include <aikido/distance/defaults.hpp>
#include <aikido/statespace/CartesianProduct.hpp>
#include <aikido/statespace/dart/MetaSkeletonStateSpace.hpp>
#include <aikido/trajectory/Interpolated.hpp>
#include <aikido/trajectory/util.hpp>
#include <dart/common/StlHelpers.hpp>

namespace ada {
namespace util {

//==============================================================================
bool waitForUser(const std::string& msg, bool terminate_system)
{
  ROS_INFO((msg + "\nPress [ENTER] to continue, [n] to terminate.").c_str());
  char input = ' ';
  std::cin.get(input);
  if (terminate_system)
  {
    if (input == 'n')
    {
      std::cout << "Terminate with user input [n]" << std::endl;
      exit(0);
    }
  }
  return input != 'n';
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
    double horizontalTolerance,
    double verticalTolerance,
    double yawTolerance,
    double pitchTolerance,
    double rollTolerance)
{
  Eigen::MatrixXd bw = Eigen::Matrix<double, 6, 2>::Zero();
  bw(0, 0) = -horizontalTolerance;
  bw(0, 1) = horizontalTolerance;
  bw(1, 0) = -horizontalTolerance;
  bw(1, 1) = horizontalTolerance;
  bw(2, 0) = -verticalTolerance;
  bw(2, 1) = verticalTolerance;
  bw(3, 0) = - rollTolerance;
  bw(3, 1) = rollTolerance;
  bw(4, 0) = - pitchTolerance;
  bw(4, 1) = pitchTolerance;
  bw(5, 0) = -yawTolerance;
  bw(5, 1) = yawTolerance;

  return bw;
}
//==============================================================================
aikido::trajectory::UniqueSplinePtr posePostprocessingForSO2(
    const aikido::trajectory::Spline& spline)
{

  auto tmpState = spline.getStateSpace()->createState();
  Eigen::VectorXd tmpVec(spline.getStateSpace()->getDimension());

  std::vector<Eigen::VectorXd> waypoints;
  spline.getWaypoint(0, tmpState);
  spline.getStateSpace()->logMap(tmpState, tmpVec);
  waypoints.push_back(tmpVec);

  auto interpolator
      = std::make_shared<aikido::statespace::GeodesicInterpolator>(
          spline.getStateSpace());

  for (std::size_t i = 0; i < spline.getNumWaypoints() - 1; ++i)
  {
    auto state = spline.getStateSpace()->createState();
    spline.getStateSpace()->expMap(tmpVec, tmpState);
    spline.getWaypoint(i + 1, state);
    auto diff = interpolator->getTangentVector(tmpState, state);
    tmpVec += diff;
    waypoints.push_back(tmpVec);
  }

  // TODO (avk): this will convert everything back to the
  // original representation making this function moot.
  // since we want all the timing etc. to happen in the
  // R-space, create a new statespace and use the relevant
  // space and interpolator. (see the logic in retime/aikido).
  auto interpolated = std::make_shared<aikido::trajectory::Interpolated>(
      spline.getStateSpace(), interpolator);

  auto newState = spline.getStateSpace()->createState();
  for (size_t i = 0; i < waypoints.size(); i++)
  {
    spline.getStateSpace()->expMap(waypoints[i], newState);
    interpolated->addWaypoint(i, newState);
  }

  return aikido::trajectory::convertToSpline(*interpolated);
}

} // namespace util
} // namespace ada
