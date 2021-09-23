#include "libada/Ada.hpp"
#include "libada/util.hpp"

#include <algorithm>
#include <cassert>
#include <mutex>
#include <stdexcept>
#include <string>

#include <aikido/common/RNG.hpp>
#include <aikido/common/Spline.hpp>
#include <aikido/constraint/Satisfied.hpp>
#include <aikido/control/KinematicSimulationTrajectoryExecutor.hpp>
#include <aikido/control/ros/RosTrajectoryExecutor.hpp>
#include <aikido/distance/defaults.hpp>
#include <aikido/io/yaml.hpp>
#include <aikido/planner/kunzretimer/KunzRetimer.hpp>
#include <aikido/planner/ompl/Planner.hpp>
#include <aikido/planner/parabolic/ParabolicSmoother.hpp>
#include <aikido/robot/ros/RosRobot.hpp>
#include <aikido/robot/util.hpp>
#include <aikido/statespace/dart/MetaSkeletonStateSpace.hpp>
#include <controller_manager_msgs/SwitchController.h>
#include <dart/utils/urdf/urdf.hpp>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <srdfdom/model.h>
#include <urdf/model.h>

#undef dtwarn
#define dtwarn (::dart::common::colorErr("Warning", __FILE__, __LINE__, 33))

#undef dtinfo
#define dtinfo (::dart::common::colorMsg("Info", 32))

namespace ada {

namespace internal {

dart::dynamics::BodyNodePtr getBodyNodeOrThrow(
    const dart::dynamics::MetaSkeletonPtr& skeleton, const std::string& bodyNodeName)
{
  auto bodyNode = skeleton->getBodyNode(bodyNodeName);

  if (!bodyNode)
  {
    std::stringstream message;
    message << "Bodynode [" << bodyNodeName << "] does not exist in skeleton.";
    throw std::runtime_error(message.str());
  }

  return bodyNode;
}

inline const dart::common::Uri getDartURI(const dart::common::Uri providedUri, const std::string confNamespace, const std::string key, const dart::common::Uri defaultUri) {
  if(providedUri.toString() != "") {
    return providedUri;
  }

  // Get Default from Parameter Server
  std::string uri;
  ros::param::param<std::string>("/" + confNamespace + "/" + key, uri, defaultUri.toString());

  return dart::common::Uri(uri);
}

} // namespace internal

//==============================================================================
Ada::Ada(bool simulation,
  const dart::common::Uri& adaUrdfUri,
  const dart::common::Uri& adaSrdfUri,
  const std::string confNamespace,
  const std::chrono::milliseconds threadCycle,
  aikido::planner::WorldPtr env,
  const ::ros::NodeHandle* node,
  aikido::common::RNG::result_type rngSeed,
  const dart::common::ResourceRetrieverPtr& retriever)
  : aikido::robot::ros::RosRobot(
      internal::getDartURI(adaUrdfUri, confNamespace, "default_urdf", defaultAdaUrdfUri),
      internal::getDartURI(adaSrdfUri, confNamespace, "default_srdf", defaultAdaSrdfUri),
      "ada",
      retriever)
  , mSimulation(simulation)
{
  // Metaskeleton loaded by RosRobot Constructor
  // Set up other args
  setWorld(env);
  setRNG(std::make_unique<aikido::common::RNGWrapper<std::mt19937>>(rngSeed));

  // Set up ROS Node
  if (!node)
  {
    mNode = std::make_unique<::ros::NodeHandle>();
  }
  else
  {
    mNode = std::make_unique<::ros::NodeHandle>(*node);
  }

  // Read default soft accleration/velocity limits
  double limit;
  mNode->param<double>("/" + confNamespace + "/default_accel_lim", limit, 0);
  mSoftAccelerationLimits = mDefaultAccelerationLimits = (limit != 0) ?
      Eigen::VectorXd::Constant(mMetaSkeleton->getNumDofs(), limit)
      : mMetaSkeleton->getAccelerationUpperLimits();
  mNode->param<double>("/" + confNamespace + "/default_vel_lim", limit, 0);
  mSoftVelocityLimits = mDefaultVelocityLimits = (limit != 0) ?
      Eigen::VectorXd::Constant(mMetaSkeleton->getNumDofs(), limit)
      : mMetaSkeleton->getVelocityUpperLimits();

  // Use limits to set default postprocessor
  setDefaultPostProcessor(mSoftVelocityLimits, mSoftAccelerationLimits, KunzParams());

  // Create sub-robot (arm)
  std::vector<std::string> armNodes;
  mNode->getParam("/" + confNamespace + "/arm", armNodes);
  if(armNodes.size() < 2) {
    std::stringstream message;
    message << "Configuration [/" << confNamespace << "/arm] is required.";
    throw std::runtime_error(message.str());
  }
  auto armBase = internal::getBodyNodeOrThrow(mMetaSkeleton, armNodes[0]);
  auto armEnd = internal::getBodyNodeOrThrow(mMetaSkeleton, armNodes[1]);
  auto arm = dart::dynamics::Chain::create(armBase, armEnd, "adaArm");
  mArm = registerSubRobot(arm, "adaArm");

  // Register initial End Effector Node
  std::string endEffector;
  mNode->param<std::string>("/" + confNamespace + "/end_effector", mEndEffectorName, defaultEndEffectorName);
  internal::getBodyNodeOrThrow(mMetaSkeleton, mEndEffectorName);

  // Create sub-robot (hand)
  std::string handBase;
  if(!mNode->getParam("/" + confNamespace + "/hand_base", handBase)) {
    std::stringstream message;
    message << "Configuration [/" << confNamespace << "/hand_base] is required.";
    throw std::runtime_error(message.str());
  }
  auto msCopy = getMetaSkeletonClone();
  auto hand = internal::getBodyNodeOrThrow(msCopy, handBase)->split("adaHand");
  mHand = registerSubRobot(hand, "adaHand");

  // Create Trajectory Executors
  // Should not execute trajectories on whole arm by default
  setTrajectoryExecutor(nullptr);

  // Arm Trajectory Executor
  mNode->param<std::string>("/" + confNamespace + "/arm_controller", mArmTrajControllerName, defaultArmTrajController);
  createTrajectoryExecutor(false);

  // Hand Trajectory Executor
  mNode->param<std::string>("/" + confNamespace + "/hand_controller", mHandTrajControllerName, defaultHandTrajController);
  createTrajectoryExecutor(true);

  // Load the named configurations if available
  std::string nameConfigs;
  if(mNode->getParam("/" + confNamespace + "/named_configs", nameConfigs)) {
    auto rootNode = aikido::io::loadYAML(nameConfigs, retriever);
    if(rootNode["hand"]) {
      mHand->setNamedConfigurations(aikido::robot::util::parseYAMLToNamedConfigurations(rootNode["hand"]));
    }
    if(rootNode["arm"]) {
      mHand->setNamedConfigurations(aikido::robot::util::parseYAMLToNamedConfigurations(rootNode["arm"]));
    }
  }

  // Create joint state updates
  if (!mSimulation)
  {
    // Real Robot, create state client
    mControllerServiceClient = std::make_unique<::ros::ServiceClient>(
        mNode->serviceClient<controller_manager_msgs::SwitchController>(
            "controller_manager/switch_controller"));
    mJointStateClient = std::make_unique<aikido::control::ros::RosJointStateClient>(
        mMetaSkeleton->getBodyNode(0)->getSkeleton(), *mNode, "/joint_states", 1);
  }
  else
  {
    // Simulation, create state publisher
    mPub = mNode->advertise<sensor_msgs::JointState>("joint_states", 5);
  }

  // Start driving self-thread
  mThread = std::make_unique<aikido::common::ExecutorThread>(
      std::bind(&Ada::spin, this), threadCycle);
}

//==============================================================================
void Ada::step(const std::chrono::system_clock::time_point& timepoint)
{
  Robot::step(timepoint);
  std::lock_guard<std::mutex> lock(mMetaSkeleton->getBodyNode(0)->getSkeleton()->getMutex());

  if (!mSimulation && mJointStateClient)
  {
    // Spin joint state client
    mJointStateClient->spin();

    // Get most recent joint states
    auto armSkeleton = mMetaSkeleton->getBodyNode(0)->getSkeleton();
    mMetaSkeleton->setPositions(
        mJointStateClient->getLatestPosition(*armSkeleton));
  }
  else {
    // Publish joint states to /joint_states
    sensor_msgs::JointState state;
    state.header.stamp = ros::Time::now();
    for (auto joint : mMetaSkeleton->getJoints())
    {
      if (joint->getNumDofs() < 1)
      {
        continue;
      }
      state.name.push_back(joint->getName());
      state.position.push_back(joint->getPosition(0));
      state.velocity.push_back(joint->getVelocity(0));
      state.effort.push_back(joint->getForce(0));
    }
    mPub.publish(state);
  }
}

//==============================================================================
aikido::robot::RobotPtr Ada::getArm()
{
  return mArm;
}

//==============================================================================
aikido::robot::ConstRobotPtr Ada::getArm() const
{
  return mArm;
}

//==============================================================================
aikido::robot::RobotPtr Ada::getHand()
{
  return mHand;
}

//==============================================================================
aikido::robot::ConstRobotPtr Ada::getHand() const
{
  return mHand;
}

//==============================================================================
void Ada::spin()
{
  if (mThread->isRunning())
  {
    step(std::chrono::system_clock::now());
  }
}

//==============================================================================
aikido::trajectory::TrajectoryPtr Ada::computeArmJointSpacePath(
      const std::vector<std::pair<double, Eigen::VectorXd>>& waypoints,
      const aikido::constraint::dart::CollisionFreePtr& collisionFree,
      const std::shared_ptr<aikido::planner::TrajectoryPostProcessor>
          trajPostProcessor)
{
  auto stateSpace = std::make_shared<aikido::statespace::dart::MetaSkeletonStateSpace>(
      mArm->getMetaSkeletonClone().get());

  std::shared_ptr<aikido::statespace::GeodesicInterpolator> interpolator
      = std::make_shared<aikido::statespace::GeodesicInterpolator>(stateSpace);
  std::shared_ptr<aikido::trajectory::Interpolated> traj
      = std::make_shared<aikido::trajectory::Interpolated>(
          stateSpace, interpolator);

  for (auto& waypoint : waypoints)
  {
    auto state = stateSpace->createState();
    try {
      stateSpace->convertPositionsToState(waypoint.second, state);
    } catch (const std::exception& e)
    {
      dtwarn << "Cannot convert configuration to robot state: " << e.what()
             << std::endl;
      return nullptr;
    }
    traj->addWaypoint(waypoint.first, state);
  }

  // Postprocess if enabled or provided
  auto postprocessor
      = (trajPostProcessor)
            ? trajPostProcessor
            : ((mEnablePostProcessing) ? mDefaultPostProcessor : nullptr);
  if (traj && postprocessor)
  {
    return postprocessor->postprocess(
        *traj, *(cloneRNG().get()), collisionFree);

    // Else return raw path
  }
  return traj;
}

//==============================================================================
bool Ada::startTrajectoryControllers()
{
  return switchControllers(
      std::vector<std::string>{mArmTrajControllerName,
                               mHandTrajControllerName},
      std::vector<std::string>());
}

//==============================================================================
bool Ada::stopTrajectoryControllers()
{
  cancelAllTrajectories();
  return switchControllers(
      std::vector<std::string>(),
      std::vector<std::string>{mArmTrajControllerName,
                               mHandTrajControllerName});
}

//==============================================================================
Eigen::VectorXd Ada::getVelocityLimits(bool armOnly) const
{
  return (armOnly) ? mSoftVelocityLimits.segment(0, mArm->getMetaSkeleton()->getNumDofs()) : mSoftVelocityLimits;
}

//==============================================================================
Eigen::VectorXd Ada::getAccelerationLimits(bool armOnly) const
{
  return (armOnly) ? mSoftAccelerationLimits.segment(0, mArm->getMetaSkeleton()->getNumDofs()) : mSoftAccelerationLimits;
}

//==============================================================================
void Ada::createTrajectoryExecutor(bool isHand)
{
  std::string controller = isHand ? mHandTrajControllerName : mArmTrajControllerName;
  aikido::robot::RobotPtr subrobot = isHand ? mHand : mArm;
  if(isHand) {

  }
  using aikido::control::KinematicSimulationTrajectoryExecutor;
  using aikido::control::ros::RosTrajectoryExecutor;

  if (mSimulation)
  {
    subrobot->setTrajectoryExecutor(std::make_shared<KinematicSimulationTrajectoryExecutor>(
        subrobot->getMetaSkeleton()->getBodyNode(0)->getSkeleton()));
  }
  else
  {
    std::string serverName
        = controller + "/follow_joint_trajectory";
    auto exec = std::make_shared<RosTrajectoryExecutor>(
        *mNode,
        serverName,
        defaultRosTrajectoryInterpolationTimestep,
        defaultRosTrajectoryGoalTimeTolerance);
    subrobot->setTrajectoryExecutor(exec);
  }
}

//==============================================================================
bool Ada::switchControllers(
    const std::vector<std::string>& startControllers,
    const std::vector<std::string>& stopControllers)
{
  if (!mNode)
    throw std::runtime_error("Ros node has not been instantiated.");

  if (!mControllerServiceClient)
    throw std::runtime_error("ServiceClient not instantiated.");

  controller_manager_msgs::SwitchController srv;
  // First try stopping the started controllers
  // Avoids us falsely detecting a failure if already started
  srv.request.stop_controllers = startControllers;
  srv.request.strictness
      = controller_manager_msgs::SwitchControllerRequest::BEST_EFFORT;
  mControllerServiceClient->call(srv); // Don't care about response code

  // Actual command
  srv.request.start_controllers = startControllers;
  srv.request.stop_controllers = stopControllers;
  srv.request.strictness
      = controller_manager_msgs::SwitchControllerRequest::STRICT;

  return mControllerServiceClient->call(srv) && srv.response.ok;
}

//==============================================================================
std::future<void> Ada::openHand()
{
  auto config = mHand->getNamedConfiguration("open");
  if(config.size() > 0) {
    auto traj = mHand->planToConfiguration(config);
    return mHand->executeTrajectory(traj);
  }

  // Return excepted future
  std::promise<void> promise; 
  promise.set_exception(std::make_exception_ptr(
          std::runtime_error("No 'open' configuration provided.")));
  return promise.get_future();
}

//==============================================================================
std::future<void> Ada::closeHand()
{
  auto config = mHand->getNamedConfiguration("closed");
  if(config.size() > 0) {
    auto traj = mHand->planToConfiguration(config);
    return mHand->executeTrajectory(traj);
  }

  // Return excepted future
  std::promise<void> promise; 
  promise.set_exception(std::make_exception_ptr(
          std::runtime_error("No 'closed' configuration provided.")));
  return promise.get_future();
}

//==============================================================================
dart::dynamics::BodyNodePtr Ada::getEndEffectorBodyNode()
{
  return internal::getBodyNodeOrThrow(mMetaSkeleton, mEndEffectorName);
}

} // namespace ada
