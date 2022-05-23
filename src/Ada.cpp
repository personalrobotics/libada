#include "libada/Ada.hpp"
#include "libada/util.hpp"

#include <algorithm>
#include <cassert>
#include <iostream>
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

// Internal Util function
namespace internal {

dart::dynamics::BodyNodePtr getBodyNodeOrThrow(
    const dart::dynamics::MetaSkeletonPtr& skeleton,
    const std::string& bodyNodeName)
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

inline const dart::common::Uri getDartURI(
    const std::string confNamespace,
    const std::string key,
    const std::string defaultUri)
{
  // Get Default from Parameter Server
  std::string uri = "";
  ros::param::param<std::string>(
      "/" + confNamespace + "/" + key, uri, defaultUri);

  return dart::common::Uri(uri);
}

inline hardware_interface::JointCommandModes modeFromString(std::string str) {
  if(str == "BEGIN")
    return hardware_interface::JointCommandModes::BEGIN;
  if(str == "POSITION")
    return hardware_interface::JointCommandModes::MODE_POSITION;
  if(str == "VELOCITY")
      return hardware_interface::JointCommandModes::MODE_VELOCITY;
  if(str == "EFFORT")
    return hardware_interface::JointCommandModes::MODE_EFFORT;
  if(str == "NOMODE" || str == "OTHER")
    return hardware_interface::JointCommandModes::NOMODE;
  if(str == "EMERGENCY_STOP" || str == "ESTOP")
    return hardware_interface::JointCommandModes::EMERGENCY_STOP;
  if(str == "SWITCHING")
      return hardware_interface::JointCommandModes::SWITCHING;
  
  // Else
  ROS_WARN_STREAM("Setting unknown mode '" << str.c_str() << "' to ERROR.");
  return hardware_interface::JointCommandModes::ERROR;
}

} // namespace internal

//==============================================================================
Ada::Ada(
    bool simulation,
    aikido::planner::WorldPtr env,
    std::string name,
    const std::string confNamespace,
    const std::chrono::milliseconds threadCycle,
    const ::ros::NodeHandle* node,
    aikido::common::RNG::result_type rngSeed,
    const dart::common::ResourceRetrieverPtr& retriever)
  : aikido::robot::ros::RosRobot(
        internal::getDartURI(confNamespace, "default_urdf", DEFAULT_URDF),
        internal::getDartURI(confNamespace, "default_srdf", DEFAULT_SRDF),
        name,
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
  mNode->param<double>("/" + confNamespace + "/default_accel_lim_acquisition", limit, 0);
  mSoftAccelerationLimits = mDefaultAccelerationLimits
      = (limit != 0)
            ? Eigen::VectorXd::Constant(mMetaSkeleton->getNumDofs(), limit)
            : mMetaSkeleton->getAccelerationUpperLimits();
  mNode->param<double>("/" + confNamespace + "/default_vel_lim_acquisition", limit, 0);
  mSoftVelocityLimits = mDefaultVelocityLimits
      = (limit != 0)
            ? Eigen::VectorXd::Constant(mMetaSkeleton->getNumDofs(), limit)
            : mMetaSkeleton->getVelocityUpperLimits();

  // Create sub-robot (arm)
  std::vector<std::string> armNodes;
  mNode->getParam("/" + confNamespace + "/arm", armNodes);
  if (armNodes.size() < 2)
  {
    std::stringstream message;
    message << "Configuration [/" << confNamespace << "/arm] is required.";
    throw std::runtime_error(message.str());
  }
  auto armBase = internal::getBodyNodeOrThrow(mMetaSkeleton, armNodes[0]);
  auto armEnd = internal::getBodyNodeOrThrow(mMetaSkeleton, armNodes[1]);
  auto arm = dart::dynamics::Chain::create(armBase, armEnd, "adaArm");
  mArm = registerSubRobot(arm, "adaArm");
  if (!mArm)
  {
    throw std::runtime_error("Could not create arm");
  }

  // Register initial End Effector Node
  std::string endEffector;
  mNode->param<std::string>(
      "/" + confNamespace + "/end_effector", mEndEffectorName, DEFAULT_EE_NAME);
  auto handEnd = internal::getBodyNodeOrThrow(mMetaSkeleton, mEndEffectorName);

  // Create sub-robot (hand)
  std::string handBaseName;
  if (!mNode->getParam("/" + confNamespace + "/hand_base", handBaseName))
  {
    std::stringstream message;
    message << "Configuration [/" << confNamespace
            << "/hand_base] is required.";
    throw std::runtime_error(message.str());
  }
  auto handBase = internal::getBodyNodeOrThrow(mMetaSkeleton, handBaseName);
  auto hand = dart::dynamics::Branch::create(
      dart::dynamics::Branch::Criteria(handBase), "adaHand");
  mHandRobot = registerSubRobot(hand, "adaHand");
  if (!mHandRobot)
  {
    throw std::runtime_error("Could not create hand");
  }

  // Create Trajectory Executors
  // Should not execute trajectories on whole arm by default
  // This ensures that trajectories are executed on subrobots only.
  setTrajectoryExecutor(nullptr);

  // Load Arm Trajectory controller name
  mNode->param<std::string>(
      "/" + confNamespace + "/arm_controller",
      mArmTrajControllerName,
      DEFAULT_ARM_TRAJ_CTRL);
  // Create executor for controller
  createTrajectoryExecutor(false);

  // Load Hand Trajectory controller name
  mNode->param<std::string>(
      "/" + confNamespace + "/hand_controller",
      mHandTrajControllerName,
      DEFAULT_HAND_TRAJ_CTRL);

  // Create executor for controller
  createTrajectoryExecutor(true);

  // Load the named configurations if available
  std::string nameConfigs;
  if (mNode->getParam("/" + confNamespace + "/named_configs", nameConfigs))
  {
    auto rootNode = aikido::io::loadYAML(nameConfigs, retriever);
    if (rootNode["hand"])
    {
      mHandRobot->setNamedConfigurations(
          aikido::robot::util::parseYAMLToNamedConfigurations(
              rootNode["hand"]));
    }
    if (rootNode["arm"])
    {
      mArm->setNamedConfigurations(
          aikido::robot::util::parseYAMLToNamedConfigurations(rootNode["arm"]));
    }
  }

  // Use limits to set default postprocessor
  setDefaultPostProcessor(
      mSoftVelocityLimits, mSoftAccelerationLimits, KunzParams());

  // Create joint state updates
  if (!mSimulation)
  {
    
    mRosControllerServiceClient = std::make_unique<::ros::ServiceClient>(
        mNode->serviceClient<controller_manager_msgs::SwitchController>(
            "controller_manager/switch_controller"));

    mRosJointModeCommandClient 
        = std::make_unique<aikido::control::ros::RosJointModeCommandClient>(*mNode,
            "joint_mode_controller/joint_mode_command",
            std::vector<std::string>{"joint_mode"});

    // Real Robot, create state client
    mJointStateClient
        = std::make_unique<aikido::control::ros::RosJointStateClient>(
            mMetaSkeleton->getBodyNode(0)->getSkeleton(),
            *mNode,
            "/joint_states",
            1);
  }
  else
  {
    // Simulation, create state publisher
    mPub = mNode->advertise<sensor_msgs::JointState>("joint_states_1", 5);
  }

  // Create Inner AdaHand
  mHand = std::make_shared<AdaHand>(this, handBase, handEnd);

  // Start driving self-thread
  mThread = std::make_unique<aikido::common::ExecutorThread>(
      std::bind(&Ada::spin, this), threadCycle);
}

//==============================================================================
Ada::~Ada()
{
  mThread->stop();
}

void Ada::switchTrajectoryLimits(std::string type)
{
  double limit;
  mNode->param<double>("/adaConf/default_accel_lim_"+type, limit, 0);
  mSoftAccelerationLimits = mDefaultAccelerationLimits
      = (limit != 0)
            ? Eigen::VectorXd::Constant(mMetaSkeleton->getNumDofs(), limit)
            : mMetaSkeleton->getAccelerationUpperLimits();
  mNode->param<double>("/adaConf/default_vel_lim_"+type, limit, 0);
  mSoftVelocityLimits = mDefaultVelocityLimits
      = (limit != 0)
            ? Eigen::VectorXd::Constant(mMetaSkeleton->getNumDofs(), limit)
            : mMetaSkeleton->getVelocityUpperLimits();

  // Use limits to set default postprocessor
  setDefaultPostProcessor(
      mSoftVelocityLimits, mSoftAccelerationLimits, KunzParams());
}
//==============================================================================
void Ada::step(const std::chrono::system_clock::time_point& timepoint)
{
  if (!mThread || !mThread->isRunning())
    return;

  Robot::step(timepoint);

  if (!mSimulation && mJointStateClient)
  {
    // Spin joint state client
    mJointStateClient->spin();

    // Lock Skeleton
    std::lock_guard<std::mutex> lock(getRootSkeleton()->getMutex());

    // Get most recent joint states
    try
    {
      mMetaSkeleton->setPositions(
          mJointStateClient->getLatestPosition(*mMetaSkeleton));
    }
    catch (const std::exception& e)
    {
      dtwarn << "Issue reading joints: " << e.what() << std::endl;
    }
  }
  else
  {
    // Lock Skeleton
    std::lock_guard<std::mutex> lock(getRootSkeleton()->getMutex());

    // Publish joint states to /joint_states
    sensor_msgs::JointState state;
    state.header.stamp = ros::Time::now();
    for (auto joint : mMetaSkeleton->getDofs())
    {
      state.name.push_back(joint->getName());
      state.position.push_back(joint->getPosition());
      state.velocity.push_back(joint->getVelocity());
      state.effort.push_back(joint->getForce());
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
aikido::robot::RobotPtr Ada::getHandRobot()
{
  return mHandRobot;
}

//==============================================================================
aikido::robot::ConstRobotPtr Ada::getHandRobot() const
{
  return mHandRobot;
}

//==============================================================================
std::shared_ptr<Ada::AdaHand> Ada::getHand()
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
  auto stateSpace
      = std::make_shared<aikido::statespace::dart::MetaSkeletonStateSpace>(
          mArm->getMetaSkeletonClone().get());

  std::shared_ptr<aikido::statespace::GeodesicInterpolator> interpolator
      = std::make_shared<aikido::statespace::GeodesicInterpolator>(stateSpace);
  std::shared_ptr<aikido::trajectory::Interpolated> traj
      = std::make_shared<aikido::trajectory::Interpolated>(
          stateSpace, interpolator);

  for (auto& waypoint : waypoints)
  {
    auto state = stateSpace->createState();
    try
    {
      stateSpace->convertPositionsToState(waypoint.second, state);
    }
    catch (const std::exception& e)
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
            : ((mEnablePostProcessing) ? mArm->getDefaultPostProcessor()
                                       : nullptr);
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
  std::cout<<"Starting controller: "<<mArmTrajControllerName<<std::endl;
  return switchControllers(
      std::vector<std::string>{mArmTrajControllerName, mHandTrajControllerName},
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
  return (armOnly) ? mSoftVelocityLimits.segment(
                         0, mArm->getMetaSkeleton()->getNumDofs())
                   : mSoftVelocityLimits;
}

//==============================================================================
Eigen::VectorXd Ada::getAccelerationLimits(bool armOnly) const
{
  return (armOnly) ? mSoftAccelerationLimits.segment(
                         0, mArm->getMetaSkeleton()->getNumDofs())
                   : mSoftAccelerationLimits;
}
//==============================================================================
/// Switches between controllers.
/// \param[in] startControllers Controllers to start.
/// \param[in] stopControllers Controllers to stop.
/// Returns true if controllers successfully switched.
bool Ada::switchControllersHack(const std::string targetMode,
    const std::string startController,
    const std::string stopController)
{
  using aikido::control::KinematicSimulationTrajectoryExecutor;
  using aikido::control::ros::RosTrajectoryExecutor;
  if (mSimulation)
  {
    mArm->setTrajectoryExecutor(
        std::make_shared<KinematicSimulationTrajectoryExecutor>(
            mArm->getMetaSkeleton()));
  }
  else
  {
    mArm->releaseTrajectoryExecutor();
    std::string serverName = startController + "/follow_joint_trajectory";
    auto exec = std::make_shared<RosTrajectoryExecutor>(
        *mNode,
        serverName,
        DEFAULT_ROS_TRAJ_INTERP_TIME,
        DEFAULT_ROS_TRAJ_GOAL_TIME_TOL,
        mArm->getMetaSkeleton());
    mArm->setTrajectoryExecutor(exec);
  }

  if(!switchControllers(std::vector<std::string>({startController}),std::vector<std::string>({stopController})))
    throw std::runtime_error("Switching controllers wasn't successful!");
  else
    std::cout<<"Successfully switched controller to "<<startController<<std::endl;

  if(targetMode != "")
  {
    if(!switchControllersModeHack(targetMode))
      throw std::runtime_error("Switching controller mode wasn't successful!");
    else
      std::cout<<"Successfully switched controller mode to EFFORT Mode"<<std::endl;
  }
  return true;
}

bool Ada::switchControllersModeHack(const std::string targetMode)
{
  if (!mRosJointModeCommandClient)
  {
    throw std::runtime_error("ROS joint mode controller actionlib client not instantiated.");
  }
  
  // Currently we only send the same control mode to each joint
  mRosJointModeCommandClient->execute(std::vector<hardware_interface::JointCommandModes>{internal::modeFromString(targetMode)});
  // try 
  // {
  //   std::cout<<"Waiting for future!"<<std::endl;
  //   future.get();
  //   std::cout<<"Future is now old man!"<<std::endl;
  // } catch (const std::exception &e) {
  //   dtwarn << "Exception in controller mode switching: " << e.what() << std::endl;
  //   return false;
  // }
  return true;
}

//==============================================================================
void Ada::createTrajectoryExecutor(bool isHand)
{
  std::string controller
      = isHand ? mHandTrajControllerName : mArmTrajControllerName;
  aikido::robot::RobotPtr subrobot = isHand ? mHandRobot : mArm;
  using aikido::control::KinematicSimulationTrajectoryExecutor;
  using aikido::control::ros::RosTrajectoryExecutor;

  if (mSimulation)
  {
    subrobot->setTrajectoryExecutor(
        std::make_shared<KinematicSimulationTrajectoryExecutor>(
            subrobot->getMetaSkeleton()));
  }
  else
  {
    std::string serverName = controller + "/follow_joint_trajectory";
    auto exec = std::make_shared<RosTrajectoryExecutor>(
        *mNode,
        serverName,
        DEFAULT_ROS_TRAJ_INTERP_TIME,
        DEFAULT_ROS_TRAJ_GOAL_TIME_TOL,
        subrobot->getMetaSkeleton());
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

  if (!mRosControllerServiceClient)
    throw std::runtime_error("ServiceClient not instantiated.");

  controller_manager_msgs::SwitchController srv;
  // First try stopping the started controllers
  // Avoids us falsely detecting a failure if already started
  srv.request.stop_controllers = startControllers;
  srv.request.strictness
      = controller_manager_msgs::SwitchControllerRequest::BEST_EFFORT;
  mRosControllerServiceClient->call(srv); // Don't care about response code

  // Actual command
  srv.request.start_controllers = startControllers;
  srv.request.stop_controllers = stopControllers;
  srv.request.strictness
      = controller_manager_msgs::SwitchControllerRequest::STRICT;

  for(int i=0; i<startControllers.size(); i++)
    std::cout<<"Start controller: "<<startControllers[i]<<std::endl;
  for(int i=0; i<stopControllers.size(); i++)
  std::cout<<"Stop controller: "<<stopControllers[i]<<std::endl;
  return mRosControllerServiceClient->call(srv) && srv.response.ok;
}

//==============================================================================
std::future<void> Ada::openHand()
{
  return mHand->executePreshape("open");
}

//==============================================================================
std::future<void> Ada::closeHand()
{
  return mHand->executePreshape("closed");
}

//==============================================================================
dart::dynamics::BodyNodePtr Ada::getEndEffectorBodyNode()
{
  return internal::getBodyNodeOrThrow(mMetaSkeleton, mEndEffectorName);
}

} // namespace ada
