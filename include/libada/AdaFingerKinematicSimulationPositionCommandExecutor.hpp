#ifndef LIBADA_ADAFINGERKINEMATICSIMULATIONPOSITIONCOMMANDEXECUTOR_HPP_
#define LIBADA_ADAFINGERKINEMATICSIMULATIONPOSITIONCOMMANDEXECUTOR_HPP_

#include <future>
#include <mutex>

#include <aikido/common/pointers.hpp>
#include <aikido/control/PositionCommandExecutor.hpp>
#include <dart/dart.hpp>

namespace ada {

AIKIDO_DECLARE_POINTERS(AdaFingerKinematicSimulationPositionCommandExecutor)
// Todo(tapo): To confirm this behavior from Kinova people
/// This executor mimics the behavior of an AdaHand finger.
///
/// It moves a finger to a desired point; it may stop early if the joint limit
/// is reached or collision is detected. Only the proximal joint is actuated;
/// the distal joint moves with mimic ratio.
///
/// When collision is detected on the distal link, the finger stops.
/// When collision is detected on the proximal link, the distal link continues
/// to move until it reaches the joint limit or until distal collision is
/// detected.
class AdaFingerKinematicSimulationPositionCommandExecutor
    : public aikido::control::PositionCommandExecutor
{
public:
  /// Constructor.
  ///
  /// \param[in] finger Finger to be controlled by this Executor.
  /// \param[in] proximal Index of proximal dof
  /// \param[in] distal Index of distal dof
  /// \param[in] collisionDetector CollisionDetector to check finger collisions
  ///        If nullptr, default to FCLCollisionDetector.
  /// \param[in] collideWith CollisionGroup to check finger collisions
  ///        If nullptr, default to empty CollisionGroup.
  /// \param[in] collisionOptions
  ///        Default is (enableContact=false, binaryCheck=true,
  ///        maxNumContacts = 1.) See dart/collison/Option.h for more
  ///        information
  AdaFingerKinematicSimulationPositionCommandExecutor(
      ::dart::dynamics::ChainPtr finger,
      std::size_t proximal,
      std::size_t distal,
      ::dart::collision::CollisionDetectorPtr collisionDetector = nullptr,
      ::dart::collision::CollisionGroupPtr collideWith = nullptr,
      ::dart::collision::CollisionOption collisionOptions
      = ::dart::collision::CollisionOption(false, 1));

  /// Open or close finger to goal position. Call step() after this until future
  /// returns for actual execution.
  ///
  /// Proximal dof moves to goalPosition, joint limit, or until collision.
  /// Distal dof follows with mimic ratio.
  ///
  /// \param[in] goalPosition Desired angle for proximal joint
  /// \return future which becomes available when movement stops
  std::future<void> execute(const Eigen::VectorXd& goalPosition) override;

  /// Returns the mimic ratio, i.e. how much the distal joint moves relative to
  /// the proximal joint.
  /// \return mimic ratio
  constexpr static double getMimicRatio()
  {
    return kMimicRatio;
  }

  /// \copydoc BarrettHandKinematicSimulationPositionCommandExecutor::step()
  ///
  /// Moves the finger joint positions by dofVelocity * timeSincePreviousCall
  /// until either the proximal dof reaches goalPosition, a joint limit is
  /// reached, or collision is detected. timeSincePreviousCall is computed by
  /// subtracting the timepoint of the previous call from the current timepoint.
  /// When collision is detected on the distal link, the finger stops.
  /// When collision is detected on the proximal link, the distal link continues
  /// to move until it either reaches mimicRatio * goalPosition, a joint limit
  /// is reached, or collision is detected.
  void step(const std::chrono::system_clock::time_point& timepoint) override;

  // clang-format off
  /// \copydoc BarrettHandKinematicSimulationPositionCommandExecutor::setCollideWith()
  // clang-format on
  bool setCollideWith(::dart::collision::CollisionGroupPtr collideWith);

private:
  /// Creates a CollisionGroup in the CollisionDetector for the fingers.
  void setFingerCollisionGroup();

  /// Helper method for step() to set variables for terminating an execution.
  void terminate();

  /// How much the distal joint moves relative to the proximal joint. This ratio
  /// is only considered when both joints are moving.
  // TODO (Tapo): Figure out the correct mimic ratio for ADA
  constexpr static double kMimicRatio = 0.0;

  /// Proximal joint velocity limit
  // TODO (Tapo): Figure out the correct speed for ADA
  constexpr static double kProximalSpeed = 2.0;

  /// Distal joint velocity limit
  constexpr static double kDistalSpeed = kProximalSpeed * kMimicRatio;

  /// Finger to position command
  ::dart::dynamics::ChainPtr mFinger;

  /// Proximal DOF
  ::dart::dynamics::DegreeOfFreedom* mProximalDof;

  /// Distal DOF
  ::dart::dynamics::DegreeOfFreedom* mDistalDof;

  /// Joint limits for proximal and distal dof
  std::pair<double, double> mProximalLimits, mDistalLimits;

  /// Collision detector to check finger collisions with
  ::dart::collision::CollisionDetectorPtr mCollisionDetector;

  /// Collision group to check for finger collisions against
  ::dart::collision::CollisionGroupPtr mCollideWith;

  /// Collision options to check finger collisions with
  ::dart::collision::CollisionOption mCollisionOptions;

  /// Collision group for proximal link
  ::dart::collision::CollisionGroupPtr mProximalCollisionGroup;

  /// Collision group for distal link
  ::dart::collision::CollisionGroupPtr mDistalCollisionGroup;

  /// Desired end-position of proximal dof
  double mProximalGoalPosition;

  /// Desired end-position of distal dof
  double mDistalGoalPosition;

  /// Whether only the distal joint should move
  bool mDistalOnly;

  /// Whether a position command is being executed
  bool mInProgress;

  /// Promise whose future is returned by execute()
  std::unique_ptr<std::promise<void>> mPromise;

  /// Manages access to mCollideWith, mProximalGoalPosition,
  /// mDistalGoalPosition, mDistalOnly, mInProgress, mPromise
  mutable std::mutex mMutex;
};

} // ada

#endif // LIBADA_ADAFINGERKINEMATICSIMULATIONPOSITIONCOMMANDEXECUTOR_HPP_
