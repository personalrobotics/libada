#ifndef LIBADA_ADA_HPP_
#define LIBADA_ADA_HPP_

#include <chrono>
#include <future>
#include <memory>

#include <Eigen/Core>
#include <actionlib/client/simple_action_client.h>
#include <aikido/common/ExecutorThread.hpp>
#include <aikido/common/RNG.hpp>
#include <aikido/constraint/Testable.hpp>
#include <aikido/constraint/TestableIntersection.hpp>
#include <aikido/constraint/dart/CollisionFree.hpp>
#include <aikido/constraint/dart/TSR.hpp>
#include <aikido/control/TrajectoryExecutor.hpp>
#include <aikido/control/ros/RosJointStateClient.hpp>
#include <aikido/io/CatkinResourceRetriever.hpp>
#include <aikido/planner/World.hpp>
#include <aikido/planner/kunzretimer/KunzRetimer.hpp>
#include <aikido/planner/vectorfield/VectorFieldPlanner.hpp>
#include <aikido/robot/ConcreteManipulator.hpp>
#include <aikido/robot/ConcreteRobot.hpp>
#include <aikido/robot/util.hpp>
#include <aikido/statespace/dart/MetaSkeletonStateSpace.hpp>
#include <aikido/trajectory/Interpolated.hpp>
#include <aikido/trajectory/Spline.hpp>
#include <aikido/trajectory/Trajectory.hpp>
#include <boost/optional.hpp>
#include <dart/dart.hpp>
#include <ros/ros.h>

#include "libada/AdaHand.hpp"

namespace ada {

extern dart::common::Uri defaultAdaUrdfUri;
extern dart::common::Uri defaultAdaSrdfUri;
extern std::vector<std::string> possibleTrajectoryExecutors;

/// ADA-specific defaults for the KunzRetimer.
// Default kunz parameters
constexpr static double DEFAULT_KUNZ_DEVIATION = 1e-3;
constexpr static double DEFAULT_KUNZ_STEP = 1e-3;
struct KunzParams : aikido::planner::kunzretimer::KunzRetimer::Params
{
  KunzParams(
      double _maxDeviation = DEFAULT_KUNZ_DEVIATION,
      double _timeStep = DEFAULT_KUNZ_STEP)
    : aikido::planner::kunzretimer::KunzRetimer::Params(
          _maxDeviation, _timeStep)
  {
    // Do nothing.
  }
};

class Ada final : public aikido::robot::Robot
{
public:
  // Expose base class functions.
  using aikido::robot::Robot::getMetaSkeleton;
  using aikido::robot::Robot::getStateSpace;

  // TODO(tapo) parameter.
  const double collisionResolution = 0.02;
  const double rosTrajectoryInterpolationTimestep = 0.1;
  const double rosTrajectoryGoalTimeTolerance = 5.0;

  // TODO(tapo) parameter.
  const std::chrono::milliseconds threadExecutionCycle{10};
  const std::chrono::milliseconds jointUpdateCycle{10};

  const aikido::robot::util::VectorFieldPlannerParameters vfParams
      = aikido::robot::util::VectorFieldPlannerParameters(
          0.2, 0.03, 0.01, 1e-3, 1e-3, 1.0, 0.2, 0.01);

  /// Construct Ada metaskeleton using a URI.
  /// \param[in] env World (either for planning, post-processing, or executing).
  /// \param[in] simulation True if running in simulation mode.
  /// \param[in] node ROS node. Required for running in real.
  /// \param[in] rngSeed seed for initializing random generator.
  ///            May be nullptr if simulation is true.
  /// \param[in] adaUrdfUri Path to Ada urdf model.
  /// \param[in] adaSrdfUri Path to Ada srdf file.
  /// \param[in] endEffectorName Name of end effector as in the urdf file.
  /// \param[in] retriever Resource retriever for retrieving Ada
  Ada(aikido::planner::WorldPtr env,
      bool simulation,
      const dart::common::Uri& adaUrdfUri = defaultAdaUrdfUri,
      const dart::common::Uri& adaSrdfUri = defaultAdaSrdfUri,
      const std::string& endEffectorName = "j2n6s200_end_effector",
      const std::string& armTrajectoryExecutorName
      = "rewd_trajectory_controller",
      const ::ros::NodeHandle* node = nullptr,
      aikido::common::RNG::result_type rngSeed = std::random_device{}(),
      const dart::common::ResourceRetrieverPtr& retriever
      = std::make_shared<aikido::io::CatkinResourceRetriever>());

  virtual ~Ada() = default;

  /// \copydoc ConcreteRobot::postProcessPath.
  template <typename PostProcessor>
  aikido::trajectory::UniqueSplinePtr postProcessPath(
      const aikido::trajectory::Trajectory* path,
      const aikido::constraint::TestablePtr& constraint,
      const typename PostProcessor::Params& postProcessorParams,
      const Eigen::VectorXd& velocityLimits = Eigen::Vector6d::Zero(),
      const Eigen::VectorXd& accelerationLimits = Eigen::Vector6d::Zero());
  /// Executes a trajectory.
  /// \param[in] trajectory Timed trajectory to execute.
  std::future<void> executeTrajectory(
      const aikido::trajectory::TrajectoryPtr& trajectory) const override;

  /// Returns a named configuration.
  /// \param[in] name Name of the configuration.
  boost::optional<Eigen::VectorXd> getNamedConfiguration(
      const std::string& name) const override;

  /// Sets the list of named configurations.
  /// \param[in] namedConfigurations Map of name, configuration.
  void setNamedConfigurations(
      std::unordered_map<std::string, const Eigen::VectorXd>
          namedConfigurations) override;

  /// Returns the Name of this Robot.
  std::string getName() const override;

  /// Returns the MetaSkeleton of this robot.
  dart::dynamics::ConstMetaSkeletonPtr getMetaSkeleton() const override;

  /// Returns the MetaSkeletonStateSpace of this robot.
  aikido::statespace::dart::ConstMetaSkeletonStateSpacePtr getStateSpace()
      const override;

  /// Sets the root of this robot.
  void setRoot(Robot* robot) override;

  /// Simulates up to the provided timepoint.
  /// Assumes that parent robot is locked.
  /// \param[in] timepoint Time to simulate to.
  void step(const std::chrono::system_clock::time_point& timepoint) override;

  /// Returns self collision constraint
  aikido::constraint::dart::CollisionFreePtr getSelfCollisionConstraint(
      const aikido::statespace::dart::ConstMetaSkeletonStateSpacePtr& space,
      const dart::dynamics::MetaSkeletonPtr& metaSkeleton) const override;

  /// Returns self-collision constraint along with provided constraint
  /// \param[in] collisionFree Collision constraint
  aikido::constraint::TestablePtr getFullCollisionConstraint(
      const aikido::statespace::dart::ConstMetaSkeletonStateSpacePtr& space,
      const dart::dynamics::MetaSkeletonPtr& metaSkeleton,
      const aikido::constraint::dart::CollisionFreePtr& collisionFree)
      const override;

  // Clones RNG.
  std::unique_ptr<aikido::common::RNG> cloneRNG();

  /// Get the world.
  aikido::planner::WorldPtr getWorld() const;

  /// Get the arm.
  aikido::robot::ConcreteManipulatorPtr getArm();

  /// Get the const arm.
  aikido::robot::ConstConcreteManipulatorPtr getArm() const;

  /// Get the hand.
  AdaHandPtr getHand();

  /// Get the const hand.
  ConstAdaHandPtr getHand() const;

  /// Get current configuration.
  Eigen::VectorXd getCurrentConfiguration() const;

  // Runs step with current time.
  void update();

  /// Generates a timed (but not smoothed) trajectory from the given list of
  /// configurations.
  /// \param[in] stateSpace The statespace for the trajectory.
  /// \param[in] waypoints Ordered configurations to add to the trajectory, each
  /// of which is paired with its desired time along the output trajectory.
  /// \return Timed trajectory.
  aikido::trajectory::TrajectoryPtr computeRetimedJointSpacePath(
      const aikido::statespace::dart::MetaSkeletonStateSpacePtr& stateSpace,
      const std::vector<std::pair<double, Eigen::VectorXd>>& waypoints);

  /// Generates a timed *and* smoothed trajectory from the given list of
  /// configurations.
  /// \param[in] space The statespace for the trajectory.
  /// \param[in] waypoints Ordered configurations to add to the trajectory, each
  /// of which is paired with its desired time along the output trajectory.
  /// \param[in] collisionFree Collision constraint during post-processing.
  /// \return Timed amd smoothed trajectory.
  aikido::trajectory::TrajectoryPtr computeSmoothJointSpacePath(
      const aikido::statespace::dart::MetaSkeletonStateSpacePtr& stateSpace,
      const std::vector<std::pair<double, Eigen::VectorXd>>& waypoints,
      const aikido::constraint::dart::CollisionFreePtr& collisionFree
      = nullptr);

  /// \copydoc Robot::planToConfiguration.
  aikido::trajectory::TrajectoryPtr planToConfiguration(
      const aikido::statespace::dart::MetaSkeletonStateSpacePtr& stateSpace,
      const dart::dynamics::MetaSkeletonPtr& metaSkeleton,
      const aikido::statespace::StateSpace::State* goalState,
      const aikido::constraint::dart::CollisionFreePtr& collisionFree,
      double timelimit);

  /// Wrapper for planToConfiguration using Eigen vectors.
  aikido::trajectory::TrajectoryPtr planToConfiguration(
      const aikido::statespace::dart::MetaSkeletonStateSpacePtr& stateSpace,
      const dart::dynamics::MetaSkeletonPtr& metaSkeleton,
      const Eigen::VectorXd& goal,
      const aikido::constraint::dart::CollisionFreePtr& collisionFree,
      double timelimit);

  /// \copydoc Robot::planToConfigurations.
  aikido::trajectory::TrajectoryPtr planToConfigurations(
      const aikido::statespace::dart::MetaSkeletonStateSpacePtr& stateSpace,
      const dart::dynamics::MetaSkeletonPtr& metaSkeleton,
      const std::vector<aikido::statespace::StateSpace::State*>& goalStates,
      const aikido::constraint::dart::CollisionFreePtr& collisionFree,
      double timelimit);

  /// Wrapper for planToConfigurations using Eigen vectors.
  aikido::trajectory::TrajectoryPtr planToConfigurations(
      const aikido::statespace::dart::MetaSkeletonStateSpacePtr& stateSpace,
      const dart::dynamics::MetaSkeletonPtr& metaSkeleton,
      const std::vector<Eigen::VectorXd>& goals,
      const aikido::constraint::dart::CollisionFreePtr& collisionFree,
      double timelimit);

  /// \copydoc ConcreteRobot::planToTSR.
  aikido::trajectory::TrajectoryPtr planToTSR(
      const aikido::statespace::dart::MetaSkeletonStateSpacePtr& stateSpace,
      const dart::dynamics::MetaSkeletonPtr& metaSkeleton,
      const dart::dynamics::BodyNodePtr& bn,
      const aikido::constraint::dart::TSRPtr& tsr,
      const aikido::constraint::dart::CollisionFreePtr& collisionFree,
      double timelimit,
      size_t maxNumTrials,
      const aikido::distance::ConfigurationRankerPtr& ranker = nullptr);

  /// Plans to a named configuration.
  /// \param[in] startState Starting state
  /// \param[in] name Name of the configuration to plan to
  /// \param[in] collisionFree Collision constraint
  /// \return Trajectory to the configuration, or nullptr if planning fails
  aikido::trajectory::TrajectoryPtr planToNamedConfiguration(
      const std::string& name,
      const aikido::constraint::dart::CollisionFreePtr& collisionFree,
      double timelimit);

  /// Switches controllers. The controllers must be switched to
  /// trajectory executors before calling \c executeTrajectory
  /// Makes sure that no other controllers are running and conflicting
  /// with the ones we are about to start.
  /// \return true if all controllers have been successfully switched
  bool startTrajectoryExecutor();

  /// Turns off controllers
  /// \return true if all controllers have been successfully switched
  bool stopTrajectoryExecutor();

  /// Sets VectorFieldPlanner parameters.
  /// TODO: To be removed with Planner API.
  /// \param[in] vfParameters VectorField Parameters
  void setVectorFieldPlannerParameters(
      const aikido::robot::util::VectorFieldPlannerParameters& vfParameters);

  /// \return TrajectoryExecutor.
  aikido::control::TrajectoryExecutorPtr getTrajectoryExecutor();

  /// Moves the end effector to a TSR.
  /// Throws a runtime_error if no trajectory could be found.
  /// \return True if the trajectory was completed successfully.
  template <typename PostProcessor = aikido::planner::kunzretimer::KunzRetimer>
  bool moveArmToTSR(
      const aikido::constraint::dart::TSR& tsr,
      const aikido::constraint::dart::CollisionFreePtr& collisionFree,
      double timelimit,
      size_t maxNumTrials,
      const aikido::distance::ConfigurationRankerPtr& ranker = nullptr,
      const Eigen::Vector6d& velocityLimits = Eigen::Vector6d::Zero(),
      const typename PostProcessor::Params& params = KunzParams());

  /// Moves the end effector along a certain position offset.
  /// Throws a runtime_error if no trajectory could be found.
  /// \return True if the trajectory was completed successfully.
  bool moveArmToEndEffectorOffset(
      const Eigen::Vector3d& direction,
      double length,
      const aikido::constraint::dart::CollisionFreePtr& collisionFree,
      double timelimit,
      double positionTolerance,
      double angularTolerance,
      const Eigen::Vector6d& velocityLimits = Eigen::Vector6d::Zero());

  /// Moves the robot to a configuration.
  /// Throws a runtime_error if no trajectory could be found.
  /// \param[in] configuration 6-D vector of joint values
  /// \param[in] collisionFree CollisionFree constraint to check.
  /// \param[in] timelimit Timelimit for planning.
  /// \param[in] velocityLimits 6-D velocity limit per joint (in value/s)
  ///             If Zero, defaults to URDF-specified limits.
  /// \return True if the trajectory was completed successfully.
  bool moveArmToConfiguration(
      const Eigen::Vector6d& configuration,
      const aikido::constraint::dart::CollisionFreePtr& collisionFree,
      double timelimit,
      const Eigen::Vector6d& velocityLimits = Eigen::Vector6d::Zero());

  /// Postprocesses and executes a trajectory.
  /// Throws runtime_error if the trajectory is empty.
  /// \param[in] trajectory Trajectory to execute.
  /// \param[in] collisionFree CollisionFree constraint to check.
  /// \param[in] velocityLimits 6-D velocity limit per joint (in value/s)
  ///             If Zero, defaults to URDF-specified limits.
  /// \param[in] params Parameters for the selected post-processor.
  /// \return True if the trajectory was completed successfully.
  template <typename PostProcessor = aikido::planner::kunzretimer::KunzRetimer>
  bool moveArmOnTrajectory(
      aikido::trajectory::TrajectoryPtr trajectory,
      const aikido::constraint::dart::CollisionFreePtr& collisionFree,
      const Eigen::Vector6d& velocityLimits = Eigen::Vector6d::Zero(),
      const typename PostProcessor::Params& params = KunzParams());

  /// Plans to a desired end-effector offset with fixed orientation.
  /// \param[in] space The StateSpace for the metaskeleton.
  /// \param[in] metaSkeleton Metaskeleton to plan with.
  /// \param[in] body Bodynode for the end effector.
  /// \param[in] collisionFree CollisionFree constraint to check.
  /// \param[in] direction Direction unit vector in the world frame.
  /// \param[in] distance Distance distance to move, in meters.
  /// \param[in] timelimit Timelimit for planning.
  /// \param[in] positionTolerance Constraint tolerance in meters.
  /// \param[in] angularTolerance Constraint tolerance in radians.
  /// \return Output trajectory
  /// \return trajectory if the planning is successful.
  aikido::trajectory::TrajectoryPtr planArmToEndEffectorOffset(
      const Eigen::Vector3d& direction,
      double length,
      const aikido::constraint::dart::CollisionFreePtr& collisionFree,
      double timelimit,
      double positionTolerance,
      double angularTolerance);

  /// Opens Ada's hand
  void openHand();

  /// Closes Ada's hand
  void closeHand();

  /// Switches between controllers.
  /// \param[in] startControllers Controllers to start.
  /// \param[in] stopControllers Controllers to stop.
  bool switchControllers(
      const std::vector<std::string>& startControllers,
      const std::vector<std::string>& stopControllers);

  /// Compute velocity limits from the MetaSkeleton
  Eigen::VectorXd getVelocityLimits() const;

  /// Compute acceleration limits from the MetaSkeleton
  Eigen::VectorXd getAccelerationLimits() const;

private:
  // Named Configurations are read from a YAML file
  using ConfigurationMap = std::unordered_map<std::string, Eigen::VectorXd>;

  /// Constructs the arm and the hand, and the manipulator.
  /// \param[in] armName Name of the arm.
  /// \param[in] retriever Resource retriever.
  /// \param[in] executor Trajectory executor for the arm
  /// \param[in] collisionDetector Collision detector for the manipulator.
  /// \param[in] selfCollisionFilter Self collision filter for the manipulator.
  /// \return Manipulator robot with Ada's arm and hand.
  aikido::robot::ConcreteManipulatorPtr configureArm(
      const std::string& armName,
      const dart::common::ResourceRetrieverPtr& retriever,
      const aikido::control::TrajectoryExecutorPtr& executor,
      dart::collision::CollisionDetectorPtr collisionDetector,
      const std::shared_ptr<dart::collision::BodyNodeCollisionFilter>&
          selfCollisionFilter);

  /// Creates and returns a trajectory executor.
  std::shared_ptr<aikido::control::TrajectoryExecutor>
  createTrajectoryExecutor();

  /// Plans the end effector to a TSR.
  /// Throws a runtime_error if no trajectory could be found.
  /// \return trajectory if the planning is successful.
  aikido::trajectory::TrajectoryPtr planArmToTSR(
      const aikido::constraint::dart::TSR& tsr,
      const aikido::constraint::dart::CollisionFreePtr& collisionFree,
      double timelimit,
      size_t maxNumTrials,
      const aikido::distance::ConfigurationRankerPtr& ranker = nullptr);

  // True if running in simulation.
  const bool mSimulation;

  // Names of the trajectory executors
  std::string mArmTrajectoryExecutorName;
  const std::string mHandTrajectoryExecutorName = "j2n6s200_hand_controller";

  // Collision resolution.
  double mCollisionResolution;

  /// Random generator
  aikido::common::RNGWrapper<std::mt19937> mRng;

  // Used by smoothTrajectory.
  // Next two comment lines are copied from libherb, might not be true for Ada.
  // We are temporarily hard-coding these to 1e-3 in the constructor. 1e-4 was
  // slow, 1e-2 was too loose of a tolerance and resulted in collisions.
  double mSmootherFeasibilityCheckResolution;
  double mSmootherFeasibilityApproxTolerance;

  // World associated with this robot.
  aikido::planner::WorldPtr mWorld;

  // Statespace of Ada.
  aikido::statespace::dart::MetaSkeletonStateSpacePtr mSpace;

  // mRobot is a wrapper around the meta skeleton
  aikido::robot::ConcreteRobotPtr mRobot;

  // mRobotSkeleton stores the full skeleton of all components (arm and hand)
  dart::dynamics::SkeletonPtr mRobotSkeleton;

  // Ros node associated with this robot.
  std::unique_ptr<::ros::NodeHandle> mNode;

  // Ros controller service client.
  std::unique_ptr<::ros::ServiceClient> mControllerServiceClient;

  // Ros joint state client.
  std::unique_ptr<aikido::control::ros::RosJointStateClient> mJointStateClient;

  // Thread for updating joint state from the Ros joint state client.
  std::unique_ptr<aikido::common::ExecutorThread> mJointStateThread;

  // Trajectory executor.
  std::shared_ptr<aikido::control::TrajectoryExecutor> mTrajectoryExecutor;

  // Name of the first link of the arm in the URDF
  std::string mArmBaseName;

  // Name of the last link of the arm in the URDF
  std::string mArmEndName;

  // Name of the End Effector in the URDF
  // might differ for different Ada configurations
  std::string mEndEffectorName;
  std::string mHandBaseName;

  // The robot arm
  aikido::robot::ConcreteManipulatorPtr mArm;
  aikido::statespace::dart::MetaSkeletonStateSpacePtr mArmSpace;

  // The hand
  AdaHandPtr mHand;

  // For trajectory executions.
  std::unique_ptr<aikido::common::ExecutorThread> mThread;

  // State Publisher in Simulation
  ros::Publisher mPub;
};

} // namespace ada

#include "detail/Ada-impl.hpp"

#endif // LIBADA_ADA_HPP_
