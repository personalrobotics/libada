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
#include <aikido/control/ros/RosJointCommandExecutor.hpp>
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

} // namespace internal

//==============================================================================
Ada::Ada(
    bool simulation,
    aikido::planner::WorldPtr env,
    const std::string confNamespace,
    const std::chrono::milliseconds threadCycle,
    const ::ros::NodeHandle* node,
    aikido::common::RNG::result_type rngSeed,
    const dart::common::ResourceRetrieverPtr& retriever)
  : aikido::robot::ros::RosRobot(
        internal::getDartURI(confNamespace, "default_urdf", DEFAULT_URDF),
        internal::getDartURI(confNamespace, "default_srdf", DEFAULT_SRDF),
        "ada",
        false,
        retriever)
  , mSimulation(simulation)
{

  // Clear Default Executors (should be redundant)
  clearExecutors();

  // Metaskeleton loaded by RosRobot Constructor
  // Set up other args
  setWorld(env);
  setRNG(std::make_unique<aikido::common::RNGWrapper<std::mt19937>>(rngSeed));

  // Set up ROS Node
  if (node)
  {
    mNode = ::ros::NodeHandle(*node);
  }

  // Read default soft accleration/velocity limits
  double limit;
  mNode.param<double>("/" + confNamespace + "/default_accel_lim", limit, 0);
  mSoftAccelerationLimits = mDefaultAccelerationLimits
      = (limit != 0)
            ? Eigen::VectorXd::Constant(mMetaSkeleton->getNumDofs(), limit)
            : mMetaSkeleton->getAccelerationUpperLimits();
  mNode.param<double>("/" + confNamespace + "/default_vel_lim", limit, 0);
  mSoftVelocityLimits = mDefaultVelocityLimits
      = (limit != 0)
            ? Eigen::VectorXd::Constant(mMetaSkeleton->getNumDofs(), limit)
            : mMetaSkeleton->getVelocityUpperLimits();

  // Create sub-robot (arm)
  std::vector<std::string> armNodes;
  mNode.getParam("/" + confNamespace + "/arm", armNodes);
  if (armNodes.size() < 2)
  {
    std::stringstream message;
    message << "Configuration [/" << confNamespace << "/arm] is required.";
    throw std::runtime_error(message.str());
  }
  auto armBase = internal::getBodyNodeOrThrow(mMetaSkeleton, armNodes[0]);
  auto armEnd = internal::getBodyNodeOrThrow(mMetaSkeleton, armNodes[1]);
  auto arm = dart::dynamics::Chain::create(armBase, armEnd, "adaArm");
  mArm = std::dynamic_pointer_cast<aikido::robot::ros::RosRobot>(registerSubRobot(arm, "adaArm"));
  mArm->clearExecutors();
  if (!mArm)
  {
    throw std::runtime_error("Could not create arm");
  }

  // Register initial End Effector Node
  std::string endEffector;
  mNode.param<std::string>(
      "/" + confNamespace + "/end_effector", mEndEffectorName, DEFAULT_EE_NAME);
  auto handEnd = internal::getBodyNodeOrThrow(mMetaSkeleton, mEndEffectorName);

  // Create sub-robot (hand)
  std::string handBaseName;
  if (!mNode.getParam("/" + confNamespace + "/hand_base", handBaseName))
  {
    std::stringstream message;
    message << "Configuration [/" << confNamespace
            << "/hand_base] is required.";
    throw std::runtime_error(message.str());
  }
  auto handBase = internal::getBodyNodeOrThrow(mMetaSkeleton, handBaseName);
  auto hand = dart::dynamics::Branch::create(
      dart::dynamics::Branch::Criteria(handBase), "adaHand");
  mHandRobot = std::dynamic_pointer_cast<aikido::robot::ros::RosRobot>(registerSubRobot(hand, "adaHand"));
  mHandRobot->clearExecutors();
  if (!mHandRobot)
  {
    throw std::runtime_error("Could not create hand");
  }

  // Load the named configurations if available
  std::string nameConfigs;
  if (mNode.getParam("/" + confNamespace + "/named_configs", nameConfigs))
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
    mJointStateClient
        = std::make_unique<aikido::control::ros::RosJointStateClient>(
            mMetaSkeleton->getBodyNode(0)->getSkeleton(),
            mNode,
            "/joint_states",
            1);
  }
  else
  {
    // Simulation, create state publisher
    mPub = mNode.advertise<sensor_msgs::JointState>("joint_states", 5);
  }

  // Create Inner AdaHand
  mHand = std::make_shared<AdaHand>(this, handBase, handEnd);

  // Register all executors
  if (mSimulation)
  {
    // Kinematic Executors
    for (auto subrobot :
         std::set<aikido::robot::ros::RosRobotPtr>{mArm, mHandRobot})
    {
      subrobot->registerExecutor(
          std::make_shared<
              aikido::control::KinematicSimulationTrajectoryExecutor>(
              subrobot->getMetaSkeleton()));
      subrobot->registerExecutor(
          std::make_shared<
              aikido::control::KinematicSimulationPositionExecutor>(
              subrobot->getMetaSkeleton()));
      auto jvExec = std::make_shared<aikido::control::JacobianVelocityExecutor>(
          handEnd);
      auto vsExec
          = std::make_shared<aikido::control::VisualServoingVelocityExecutor>(
              handEnd, jvExec);
      subrobot->registerExecutor(vsExec);
      subrobot->registerExecutor(jvExec);
    }
  }
  else
  {
    auto rosLoadControllerServiceClient = std::make_shared<::ros::ServiceClient>(
        mNode.serviceClient<controller_manager_msgs::LoadController>(
            "controller_manager/load_controller"));
    auto rosSwitchControllerServiceClient = std::make_shared<::ros::ServiceClient>(
        mNode.serviceClient<controller_manager_msgs::SwitchController>(
            "controller_manager/switch_controller"));
    // Real ROS Executors
    for (auto subrobot :
         std::set<aikido::robot::ros::RosRobotPtr>{mArm, mHandRobot})
    {
      subrobot->setRosLoadControllerServiceClient(rosLoadControllerServiceClient);
      bool isHand = (subrobot == mHandRobot);
      std::string ns = isHand ? "/" + confNamespace + "/hand_executors/"
                              : "/" + confNamespace + "/arm_executors/";
      // Controller Switching Service Client
      bool enableControllerSwitching = mNode.param<bool>(
          ns + "enable_controller_switching",
          false);
      if(enableControllerSwitching)
        subrobot->setRosSwitchControllerServiceClient(rosSwitchControllerServiceClient);
      // Mode controller
      auto modeControllerName = mNode.param<std::string>(
          ns + "mode_controller",
          std::string(""));
      std::cout<<"Mode controller name: "<<modeControllerName<<std::endl;
      if(!modeControllerName.empty())
      {
        auto rosJointModeCommandClient 
            = std::make_shared<aikido::control::ros::RosJointModeCommandClient>(
                mNode,
                modeControllerName + "/joint_mode_command",
                std::vector<std::string>{"joint_mode"});
          subrobot->setRosJointModeCommandClient(rosJointModeCommandClient);
      }
      std::vector<util::ExecutorDetails> executorsDetails = util::loadExecutorsDetailsFromParameter(
          mNode,
          ns + "executors");
      if(executorsDetails.size() == 0)
      {
        std::stringstream message;
        message << "Unable to load executors details for " << ns;
        throw std::runtime_error(message.str());
      }
      for(auto executorDetails: executorsDetails)
      {
        if(executorDetails.mType == "TRAJECTORY")
        {
          std::cout<<"Executor type is TRAJECTORY!!"<<std::endl;
          auto trajExec
            = std::make_shared<aikido::control::ros::RosTrajectoryExecutor>(
                mNode,
                executorDetails.mController + "/follow_joint_trajectory",
                DEFAULT_ROS_TRAJ_INTERP_TIME,
                DEFAULT_ROS_TRAJ_GOAL_TIME_TOL,
                subrobot->getMetaSkeleton());
          auto controllerMode = util::modeFromString(
            mNode.param<std::string>("/" + executorDetails.mController + "/controller_mode", ""));
          subrobot->registerExecutor(trajExec, executorDetails.mId, executorDetails.mController, controllerMode);
        }
        else if(executorDetails.mType == "JOINT_COMMAND")
        {
          if(executorDetails.mMode == "POSITION")
          {
            auto posExec
              = std::make_shared<aikido::control::ros::RosJointPositionExecutor>(
                  mNode, executorDetails.mController, subrobot->getMetaSkeleton()->getDofs());
            auto controllerMode = util::modeFromString(
              mNode.param<std::string>("/" + executorDetails.mController + "/controller_mode", "POSITION"));
            subrobot->registerExecutor(posExec, executorDetails.mId, executorDetails.mController, controllerMode);
          }
          else if(executorDetails.mMode == "VELOCITY")
          {
            auto velExec
              = std::make_shared<aikido::control::ros::RosJointVelocityExecutor>(
                  mNode, executorDetails.mController, subrobot->getMetaSkeleton()->getDofs());
            auto controllerMode = util::modeFromString(
              mNode.param<std::string>("/" + executorDetails.mController + "/controller_mode", "VELOCITY"));
            subrobot->registerExecutor(velExec, executorDetails.mId, executorDetails.mController, controllerMode);
          }
          else if(executorDetails.mMode == "EFFORT")
          {
            auto effExec
              = std::make_shared<aikido::control::ros::RosJointEffortExecutor>(
                  mNode, executorDetails.mController, subrobot->getMetaSkeleton()->getDofs());
            auto controllerMode = util::modeFromString(
              mNode.param<std::string>("/" + executorDetails.mController + "/controller_mode", "EFFORT"));
            subrobot->registerExecutor(effExec, executorDetails.mId, executorDetails.mController, controllerMode);
          }
          else
          {
            std::stringstream message;
            message << "Executor mode for [/" << ns << "/executors] - with id: "<<executorDetails.mId<<" and type: "<<executorDetails.mType<<" is incorrectly specified.";
            throw std::runtime_error(message.str());
          }
        }
        else if(executorDetails.mType == "TASK_COMMAND")
        {
          if(executorDetails.mMode == "VELOCITY")
          {
            auto velExec
              = std::make_shared<aikido::control::ros::RosJointVelocityExecutor>(
                  mNode, executorDetails.mController, subrobot->getMetaSkeleton()->getDofs());
            auto jvExec
              = std::make_shared<aikido::control::JacobianVelocityExecutor>(
                  handEnd, velExec);
            auto controllerMode = util::modeFromString(
              mNode.param<std::string>("/" + executorDetails.mController + "/controller_mode", "VELOCITY"));
            subrobot->registerExecutor(jvExec, executorDetails.mId, executorDetails.mController, controllerMode);
          }
          else if(executorDetails.mMode == "EFFORT")
          {
            auto effExec
              = std::make_shared<aikido::control::ros::RosJointEffortExecutor>(
                  mNode, executorDetails.mController, subrobot->getMetaSkeleton()->getDofs());
            auto effJacExec
              = std::make_shared<aikido::control::JacobianEffortExecutor>(
                  handEnd, effExec);
            auto controllerMode = util::modeFromString(
              mNode.param<std::string>("/" + executorDetails.mController + "/controller_mode", "EFFORT"));
            subrobot->registerExecutor(effJacExec, executorDetails.mId, executorDetails.mController, controllerMode);
          }
          else
          {
            std::stringstream message;
            message << "Executor mode for [/" << ns << "/executors] - with id: "<<executorDetails.mId<<" and type: "<<executorDetails.mType<<" is incorrectly specified.";
            throw std::runtime_error(message.str());
          }
        }
        else if(executorDetails.mType == "VISUAL_SERVOING")
        {
          if(executorDetails.mMode == "VELOCITY")
          {
            auto velExec
              = std::make_shared<aikido::control::ros::RosJointVelocityExecutor>(
                  mNode, executorDetails.mController, subrobot->getMetaSkeleton()->getDofs());
            auto jvExec
              = std::make_shared<aikido::control::JacobianVelocityExecutor>(
                  handEnd, velExec);
            auto vsExec
              = std::make_shared<aikido::control::VisualServoingVelocityExecutor>(
                  handEnd, jvExec);
            auto controllerMode = util::modeFromString(
              mNode.param<std::string>("/" + executorDetails.mController + "/controller_mode", "VELOCITY"));
            subrobot->registerExecutor(vsExec, executorDetails.mId, executorDetails.mController, controllerMode);
          }
          else
          {
            std::stringstream message;
            message << "Executor mode for [/" << ns << "/executors] - with id: "<<executorDetails.mId<<" and type: "<<executorDetails.mType<<" is incorrectly specified.";
            throw std::runtime_error(message.str());
          }
        }
      }
    }
  }

  std::cout<<"Creating thread for executors!"<<std::endl;
  // Start driving self-thread
  
  mThread = std::make_unique<aikido::common::ExecutorThread>(
      std::bind(&Ada::spin, this), threadCycle);
  
  std::cout<<"Thread created!"<<std::endl;
  std::cout<<"Activating trajectory executor for arm!"<<std::endl;
  // Activate Trajectory Executor for Arm
  mArm->activateExecutor(aikido::control::ExecutorType::TRAJECTORY);
  std::cout<<"Activated trajectory executor!"<<std::endl;
}

//==============================================================================
Ada::~Ada()
{
  mThread->stop();
}

//==============================================================================
void Ada::step(const std::chrono::system_clock::time_point& timepoint)
{
  if (!mThread || !mThread->isRunning())
    return;

  Robot::step(timepoint);

  static bool firstRun = true;

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
      if (firstRun)
      {
        dtinfo << "Live joints successfully read into MetaSkeleton"
               << std::endl;
        firstRun = false;
      }
    }
    catch (const std::exception& e)
    {
      if (!firstRun)
      {
        dtwarn << "Issue reading joints: " << e.what() << std::endl;
      }
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
aikido::robot::ros::RosRobotPtr Ada::getArm()
{
  return mArm;
}
//==============================================================================
aikido::robot::ros::ConstRosRobotPtr Ada::getArm() const
{
  return mArm;
}
//==============================================================================
aikido::robot::ros::RosRobotPtr Ada::getHandRobot()
{
  return mHandRobot;
}
//==============================================================================
aikido::robot::ros::ConstRosRobotPtr Ada::getHandRobot() const
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
