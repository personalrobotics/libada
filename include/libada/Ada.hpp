#ifndef LIBADA_ADA_HPP_
#define LIBADA_ADA_HPP_

#include <chrono>
#include <future>
#include <memory>

#include <Eigen/Core>
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
#include <aikido/planner/parabolic/ParabolicSmoother.hpp>
#include <aikido/robot/ros/RosRobot.hpp>
#include <aikido/statespace/dart/MetaSkeletonStateSpace.hpp>
#include <aikido/trajectory/Interpolated.hpp>
#include <aikido/trajectory/Spline.hpp>
#include <aikido/trajectory/Trajectory.hpp>
#include <dart/dart.hpp>
#include <ros/ros.h>

#include <iostream>

namespace ada {

/// ADA-specific defaults for postprocessors
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

// Default parabolic smoother parameters
constexpr static double DEFAULT_SMOOTH_RESOLUTION = 1e-3;
constexpr static double DEFAULT_SMOOTH_TOLERANCE = 1e-3;
struct SmoothParams : aikido::planner::parabolic::ParabolicSmoother::Params
{
  SmoothParams(
      double _resolution = DEFAULT_SMOOTH_RESOLUTION,
      double _tolerance = DEFAULT_SMOOTH_TOLERANCE)
    : aikido::planner::parabolic::ParabolicSmoother::Params()
  {
    aikido::planner::parabolic::ParabolicSmoother::Params::
        mFeasibilityCheckResolution
        = _resolution;
    aikido::planner::parabolic::ParabolicSmoother::Params::
        mFeasibilityApproxTolerance
        = _tolerance;
  }
};

class Ada final : public aikido::robot::ros::RosRobot
{
public:
  /// Inner AdaHand class that implements Aikido's Hand Interface
  class AdaHand;

// Default Parameters
#define DEFAULT_THREAD_CYCLE std::chrono::milliseconds(10)
#define DEFAULT_ROS_TRAJ_INTERP_TIME 0.1
#define DEFAULT_ROS_TRAJ_GOAL_TIME_TOL 5.0
#define DEFAULT_CONF_OBJ_NS "adaConf"
#define DEFAULT_ARM_TRAJ_CTRL "trajectory_controller"
#define DEFAULT_HAND_TRAJ_CTRL "hand_controller"
#define DEFAULT_EE_NAME "end_effector"
#define DEFAULT_URDF                                                           \
  "package://ada_description/robots_urdf/ada_with_camera.urdf"
#define DEFAULT_SRDF                                                           \
  "package://ada_description/robots_urdf/ada_with_camera.srdf"

  /// Construct Ada metaskeleton using a URI.
  /// \param[in] simulation True if running in simulation mode.
  /// \param[in] env World (either for planning, post-processing, or executing).
  /// \param[in] confNamespace rosparam namespace of configuration object.
  /// \param[in] threadCycle Period of self-driven spin thread (ms)
  /// \param[in] node ROS node. Required for running in real.
  /// \param[in] rngSeed seed for initializing random generator.
  ///            May be nullptr if simulation is true.
  /// \param[in] retriever Resource retriever for retrieving Ada
  Ada(bool simulation,
      aikido::planner::WorldPtr env = aikido::planner::World::create(),
      const std::string confNamespace = DEFAULT_CONF_OBJ_NS,
      const std::chrono::milliseconds threadCycle = DEFAULT_THREAD_CYCLE,
      const ::ros::NodeHandle* node = nullptr,
      aikido::common::RNG::result_type rngSeed = std::random_device{}(),
      const dart::common::ResourceRetrieverPtr& retriever
      = std::make_shared<aikido::io::CatkinResourceRetriever>());

  virtual ~Ada();

  /// Simulates up to the provided timepoint.
  /// Assumes that parent robot is locked.
  /// \param[in] timepoint Time to simulate to.
  void step(const std::chrono::system_clock::time_point& timepoint) override;

  /// Get the arm.
  aikido::robot::RobotPtr getArm();

  /// Get the const arm.
  aikido::robot::ConstRobotPtr getArm() const;

  /// Get the hand as an Aikido::Robot
  aikido::robot::RobotPtr getHandRobot();

  /// Get the const hand as an Aikido::Robot
  aikido::robot::ConstRobotPtr getHandRobot() const;

  /// Get the hand as an Aikido::Hand
  std::shared_ptr<AdaHand> getHand();

  /// Generates an arm-only trajectory from the given list of configurations.
  /// Runs through the provided or default postprocessor if enabled.
  /// \param[in] waypoints Ordered configurations to add to the trajectory, each
  /// of which is paired with its desired time along the output trajectory.
  /// \param[in] collisionFree Collision constraint during post-processing ONLY.
  /// \param[in] trajPostProcessor Optional explicit postprocessor.
  /// \return Trajectory pointer, or nullptr on error.
  aikido::trajectory::TrajectoryPtr computeArmJointSpacePath(
      const std::vector<std::pair<double, Eigen::VectorXd>>& waypoints,
      const aikido::constraint::dart::CollisionFreePtr& collisionFree,
      const std::shared_ptr<aikido::planner::TrajectoryPostProcessor>
          trajPostProcessor
      = nullptr);

  // Default to selfCollisionConstraint
  aikido::trajectory::TrajectoryPtr computeArmJointSpacePath(
      const std::vector<std::pair<double, Eigen::VectorXd>>& waypoints,
      const std::shared_ptr<aikido::planner::TrajectoryPostProcessor>
          trajPostProcessor
      = nullptr)
  {
    return computeArmJointSpacePath(
        waypoints, getSelfCollisionConstraint(), trajPostProcessor);
  }

  /// Sets the default trajectory post-processor
  /// Also enables automatic post-processing for all planning functions.
  /// \param[in] velocityLimits Can be arm-only DoF or whole-bot DoF
  /// Leave blank for MetaSkeleton defaults.
  /// \param[in] accelerationLimits Can be arm-only DoF or whole-bot DoF
  /// Leave blank for MetaSkeleton defaults.
  /// \param[in] params Can be KunzParams() [default], SmoothParams(), or custom
  template <typename PostProcessor = aikido::planner::kunzretimer::KunzRetimer>
  void setDefaultPostProcessor(
      const Eigen::VectorXd& velocityLimits = Eigen::VectorXd(),
      const Eigen::VectorXd& accelerationLimits = Eigen::VectorXd(),
      const typename PostProcessor::Params& params = KunzParams());

  /// Starts the provided trajectory controllers if not started already.
  /// Makes sure that no other controllers are running and conflicting
  /// with the ones we are about to start.
  /// \return true if all controllers have been successfully switched
  bool startTrajectoryControllers();

  /// Turns off provided trajectory controllers
  /// \return true if all controllers have been successfully switched
  bool stopTrajectoryControllers();

  /// Opens Ada's hand
  std::future<void> openHand();

  /// Closes Ada's hand
  std::future<void> closeHand();

  /// Get current soft velocity limits
  Eigen::VectorXd getVelocityLimits(bool armOnly = false) const;

  /// Get current soft acceleration limits
  Eigen::VectorXd getAccelerationLimits(bool armOnly = false) const;

  /// Get Body Node of End Effector
  dart::dynamics::BodyNodePtr getEndEffectorBodyNode();

private:
  /// Switches between controllers.
  /// \param[in] startControllers Controllers to start.
  /// \param[in] stopControllers Controllers to stop.
  /// Returns true if controllers successfully switched.
  bool switchControllers(
      const std::vector<std::string>& startControllers,
      const std::vector<std::string>& stopControllers);

  // Call to spin first to pass current time to step
  void spin();

  // Utility function to (re)-create and set trajectory executors
  void createTrajectoryExecutor(bool isHand);

  // True if running in simulation.
  const bool mSimulation;

  // Names of the ros trajectory controllers
  std::string mArmTrajControllerName;
  std::string mHandTrajControllerName;

  // Soft velocity and acceleration limits
  Eigen::VectorXd mSoftVelocityLimits;
  Eigen::VectorXd mSoftAccelerationLimits;
  Eigen::VectorXd mDefaultVelocityLimits;
  Eigen::VectorXd mDefaultAccelerationLimits;

  // Ros node associated with this robot.
  ::ros::NodeHandle mNode;

  // Ros controller service client.
  std::unique_ptr<::ros::ServiceClient> mControllerServiceClient;

  // Ros joint state client.
  std::unique_ptr<aikido::control::ros::RosJointStateClient> mJointStateClient;

  // Name of the End Effector in the URDF
  // might differ for different Ada configurations
  std::string mEndEffectorName;

  // The robot arm
  aikido::robot::RobotPtr mArm;

  // The robot hand as an Aikido Robot
  aikido::robot::RobotPtr mHandRobot;

  // Self-driving thread
  std::unique_ptr<aikido::common::ExecutorThread> mThread;

  // State Publisher in Simulation
  ros::Publisher mPub;

  // Inner Hand Object
  std::shared_ptr<AdaHand> mHand;
};

} // namespace ada

#include "detail/Ada-impl.hpp"

#include "libada/AdaHand.hpp"

#endif // LIBADA_ADA_HPP_
