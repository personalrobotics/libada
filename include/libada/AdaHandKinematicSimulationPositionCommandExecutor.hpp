#ifndef LIBADA_ADAHANDKINEMATICSIMULATIONPOSITIONCOMMANDEXECUTOR_HPP_
#define LIBADA_ADAHANDKINEMATICSIMULATIONPOSITIONCOMMANDEXECUTOR_HPP_

#include <future>
#include <mutex>

#include <Eigen/Dense>
#include <aikido/control/PositionCommandExecutor.hpp>
#include <dart/dart.hpp>

#include "libada/AdaFingerKinematicSimulationPositionCommandExecutor.hpp"

namespace ada {

AIKIDO_DECLARE_POINTERS(AdaHandKinematicSimulationPositionCommandExecutor)

/// Position command executor for simulating AdaHand.
///
/// See AdaFingerKinematicSimulationPositionCommandExecutor and
/// for details.
class AdaHandKinematicSimulationPositionCommandExecutor
    : public aikido::control::PositionCommandExecutor
{
public:
  /// Constructor.
  ///
  /// \param[in] robot Robot to construct executor for
  /// \param[in] prefix String (either "/right/" or "/left/") to specify hand
  /// \param[in] collisionDetector CollisionDetector to check finger collisions
  ///        If nullptr, default to FCLCollisionDetector.
  /// \param[in] collideWith CollisionGroup to check finger collisions
  ///        If nullptr, default to empty CollisionGroup.
  /// \param[in] collisionOptions
  ///        Default is (enableContact=false, binaryCheck=true,
  ///        maxNumContacts = 1.) See dart/collison/Option.h for more
  ///        information
  AdaHandKinematicSimulationPositionCommandExecutor(
      dart::dynamics::SkeletonPtr robot,
      const std::string& prefix,
      ::dart::collision::CollisionDetectorPtr collisionDetector = nullptr,
      ::dart::collision::CollisionGroupPtr collideWith = nullptr,
      ::dart::collision::CollisionOption collisionOptions
      = ::dart::collision::CollisionOption(false, 1));

  /// Move fingers to goalPositions. Call step() after this until future
  /// returns for actual execution.
  ///
  /// \param[in] goalPositions Desired values for proximal and spread joints.
  ///        First 3 should specify proximal joints, last element should specify
  ///        spread. Joints will move only up to the joint limits.
  /// \return future which becomes available when the movement stops
  std::future<void> execute(const Eigen::VectorXd& goalPositions) override;

  /// \copydoc PositionCommandExecutor::step()
  /// \note Lock the Skeleton associated with this executor before calling this
  /// method.
  void step(const std::chrono::system_clock::time_point& timepoint) override;

  /// Sets CollisionGroup to check against for finger collisions.
  ///
  /// \param[in] collideWith CollisionGroup to check finger collisions
  /// \return false if collideWith cannot be set (during execution)
  bool setCollideWith(::dart::collision::CollisionGroupPtr collideWith);

private:
  /// Set up mPositionCommandExecutors and mSpreadCommandExecutor.
  /// \param[in] robot Robot to construct hand executor for
  /// \param[in] prefix String (either "/right/" or "/left/") to specify hand
  void setupExecutors(
      dart::dynamics::SkeletonPtr robot, const std::string& prefix);

  /// Number of finger position executors
  constexpr static int kNumPositionExecutors = 2;

  /// Indices of primal dofs
  constexpr static auto kPrimalDofs
      = std::array<std::size_t, kNumPositionExecutors>{{0, 0}};

  /// Indices of distal dofs
  constexpr static auto kDistalDofs
      = std::array<std::size_t, kNumPositionExecutors>{{1, 1}};

  /// Executor for proximal and distal joints
  std::array<AdaFingerKinematicSimulationPositionCommandExecutorPtr,
             kNumPositionExecutors>
      mPositionCommandExecutors;

  /// Duration to wait for futures from executors
  constexpr static auto kWaitPeriod = std::chrono::milliseconds(0);

  /// Finger futures that are being waited on
  std::vector<std::future<void>> mFingerFutures;

  /// Collision detector to check finger collisions with
  ::dart::collision::CollisionDetectorPtr mCollisionDetector;

  /// Collision group to check for finger collisions against
  ::dart::collision::CollisionGroupPtr mCollideWith;

  /// Collision options to check finger collisions with
  ::dart::collision::CollisionOption mCollisionOptions;

  /// Whether a position command is being executed
  bool mInProgress;

  /// Promise whose future is returned by execute()
  std::unique_ptr<std::promise<void>> mPromise;

  /// Manages access to mCollideWith, mInProgress, mPromise
  mutable std::mutex mMutex;
};

} // namespace ada

#endif // LIBADA_ADAHANDKINEMATICSIMULATIONPOSITIONCOMMANDEXECUTOR_HPP_
