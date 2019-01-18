#ifndef FEEDING_ADAMOVER_HPP_
#define FEEDING_ADAMOVER_HPP_

#include <libada/Ada.hpp>

namespace feeding {

enum TrajectoryPostprocessType
{
  RETIME,
  SMOOTH,
  TRYOPTIMALRETIME
};

class AdaMover
{

public:
  AdaMover(
      ada::Ada& ada,
      aikido::statespace::dart::MetaSkeletonStateSpacePtr armSpace,
      aikido::constraint::dart::CollisionFreePtr collisionFreeConstraint,
      ros::NodeHandle nodeHandle);

  /// Moves the end effector to a TSR.
  /// Throws a runtime_error if no trajectory could be found.
  /// \return True if the trajectory was completed successfully.
  bool moveArmToTSR(const aikido::constraint::dart::TSR& tsr, const std::vector<double>& velocityLimits = std::vector<double>());

  /// Moves the end effector along a certain position offset.
  /// Throws a runtime_error if no trajectory could be found.
  /// \return True if the trajectory was completed successfully.
  bool moveToEndEffectorOffset(const Eigen::Vector3d& direction, double length, bool respectCollision = true);

  bool moveToEndEffectorOffset(const Eigen::Vector3d& direction, double length, const std::vector<double>& velocityLimits, bool respectCollision = true);

  bool moveWithEndEffectorTwist(const Eigen::Vector6d& twists, double durations, bool respectCollision = true);

  bool moveWithEndEffectorTwist(const Eigen::Vector6d& twists, double durations, const std::vector<double>& velocityLimits, bool respectCollision = true);

  aikido::trajectory::TrajectoryPtr planToEndEffectorOffset(
      const Eigen::Vector3d& direction, double length, bool respectCollision = true);

  aikido::trajectory::TrajectoryPtr planWithEndEffectorTwist(
      const Eigen::Vector6d& twists, double durations, bool respectCollision = true);

  /// Moves the robot to a configuration.
  /// Throws a runtime_error if no trajectory could be found.
  /// \return True if the trajectory was completed successfully.
  bool moveArmToConfiguration(const Eigen::Vector6d& configuration);

  /// Postprocesses and executes a trjectory.
  /// Throws runtime_error if the trajectory is empty.
  /// \return True if the trajectory was completed successfully.
  bool moveArmOnTrajectory(
      aikido::trajectory::TrajectoryPtr trajectory,
      TrajectoryPostprocessType postprocessType = SMOOTH,
      std::vector<double> smoothVelocityLimits = std::vector<double>());

  ada::Ada& mAda;
private:
  ros::NodeHandle mNodeHandle;
  aikido::statespace::dart::MetaSkeletonStateSpacePtr mArmSpace;
  aikido::constraint::dart::CollisionFreePtr mCollisionFreeConstraint;
};
}

#endif
