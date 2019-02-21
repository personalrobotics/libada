#include "libada/Ada.hpp"
#include "libada/util.hpp"

#include <algorithm>
#include <cassert>
#include <mutex>
#include <stdexcept>
#include <string>

#include <aikido/common/RNG.hpp>
#include <aikido/common/Spline.hpp>
#include <aikido/constraint/Testable.hpp>
#include <aikido/constraint/TestableIntersection.hpp>
#include <aikido/control/KinematicSimulationTrajectoryExecutor.hpp>
#include <aikido/control/ros/RosTrajectoryExecutor.hpp>
#include <aikido/distance/defaults.hpp>
#include <aikido/io/yaml.hpp>
#include <aikido/planner/ompl/CRRTConnect.hpp>
#include <aikido/planner/ompl/Planner.hpp>
#include <aikido/robot/ConcreteManipulator.hpp>
#include <aikido/robot/ConcreteRobot.hpp>
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

using aikido::control::TrajectoryExecutorPtr;
using aikido::constraint::dart::CollisionFreePtr;
using aikido::constraint::dart::TSR;
using aikido::constraint::dart::TSRPtr;
using aikido::constraint::TestablePtr;
using aikido::robot::Robot;
using aikido::robot::ConcreteRobot;
using aikido::robot::ConcreteManipulator;
using aikido::robot::ConcreteManipulatorPtr;
using aikido::robot::ConstConcreteManipulatorPtr;
using aikido::robot::Hand;
using aikido::robot::HandPtr;
using aikido::robot::util::parseYAMLToNamedConfigurations;
using aikido::robot::util::VectorFieldPlannerParameters;
using aikido::robot::util::CRRTPlannerParameters;
using aikido::statespace::StateSpace;
using aikido::statespace::dart::MetaSkeletonStateSpace;
using aikido::statespace::dart::MetaSkeletonStateSpacePtr;
using aikido::statespace::dart::ConstMetaSkeletonStateSpacePtr;
using aikido::trajectory::Interpolated;
using aikido::trajectory::InterpolatedPtr;
using aikido::trajectory::TrajectoryPtr;
using aikido::common::cloneRNGFrom;

using dart::collision::FCLCollisionDetector;
using dart::common::make_unique;
using dart::dynamics::BodyNodePtr;
using dart::dynamics::MetaSkeleton;
using dart::dynamics::MetaSkeletonPtr;
using dart::dynamics::SkeletonPtr;

// We use this as default since the camera-attached version is the most
// frequent use case.
dart::common::Uri defaultAdaUrdfUri{
    "package://ada_description/robots_urdf/ada_with_camera.urdf"};
dart::common::Uri defaultAdaSrdfUri{
    "package://ada_description/robots_urdf/ada_with_camera.srdf"};

const dart::common::Uri namedConfigurationsUri{
    "package://libada/resources/configurations.yaml"};

// Arm trajectory controllers that are meant to be used by Ada.
// Needs to be consistent with the configurations in ada_launch.
const std::vector<std::string> availableArmTrajectoryExecutorNames{
    "trajectory_controller",
    "rewd_trajectory_controller",
    "move_until_touch_topic_controller"};

namespace {

BodyNodePtr getBodyNodeOrThrow(
    const SkeletonPtr& skeleton, const std::string& bodyNodeName)
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
} // ns

//==============================================================================
Ada::Ada(
    aikido::planner::WorldPtr env,
    bool simulation,
    const dart::common::Uri& adaUrdfUri,
    const dart::common::Uri& adaSrdfUri,
    const std::string& endEffectorName,
    const std::string& armTrajectoryExecutorName,
    const ::ros::NodeHandle* node,
    aikido::common::RNG::result_type rngSeed,
    const dart::common::ResourceRetrieverPtr& retriever)
  : mSimulation(simulation)
  , mArmTrajectoryExecutorName(armTrajectoryExecutorName)
  , mCollisionResolution(collisionResolution)
  , mRng(rngSeed)
  , mSmootherFeasibilityCheckResolution(1e-3)
  , mSmootherFeasibilityApproxTolerance(1e-3)
  , mWorld(std::move(env))
  , mEndEffectorName(endEffectorName)
{
  simulation = true; // temporarily set simulation to true

  if (std::find(
          availableArmTrajectoryExecutorNames.begin(),
          availableArmTrajectoryExecutorNames.end(),
          mArmTrajectoryExecutorName)
      == availableArmTrajectoryExecutorNames.end())
  {
    throw std::runtime_error("Arm Trajectory Controller is not valid!");
  }

  dtinfo << "Arm Executor " << armTrajectoryExecutorName << std::endl;
  using aikido::common::ExecutorThread;
  using aikido::control::ros::RosJointStateClient;

  std::string name = "j2n6s200";

  // Load Ada
  mRobotSkeleton = mWorld->getSkeleton(name);
  if (!mRobotSkeleton)
  {
    dart::utils::DartLoader urdfLoader;
    mRobotSkeleton = urdfLoader.parseSkeleton(adaUrdfUri, retriever);
    mWorld->addSkeleton(mRobotSkeleton);
  }

  if (!mRobotSkeleton)
  {
    throw std::runtime_error("Unable to load ADA model.");
  }

  // TODO: Read from robot configuration.
  mRobotSkeleton->setAccelerationLowerLimits(
      Eigen::VectorXd::Constant(mRobotSkeleton->getNumDofs(), -2.0));
  mRobotSkeleton->setAccelerationUpperLimits(
      Eigen::VectorXd::Constant(mRobotSkeleton->getNumDofs(), 2.0));

  // Define the collision detector and groups
  auto collisionDetector = FCLCollisionDetector::create();
  // auto collideWith = collisionDetector->createCollisionGroupAsSharedPtr();
  auto selfCollisionFilter
      = std::make_shared<dart::collision::BodyNodeCollisionFilter>();

  urdf::Model urdfModel;
  std::string adaUrdfXMLString = retriever->readAll(adaUrdfUri);
  urdfModel.initString(adaUrdfXMLString);

  srdf::Model srdfModel;
  std::string adaSrdfXMLString = retriever->readAll(adaSrdfUri);
  srdfModel.initString(urdfModel, adaSrdfXMLString);
  auto disabledCollisions = srdfModel.getDisabledCollisionPairs();

  for (auto disabledPair : disabledCollisions)
  {
    auto body0 = getBodyNodeOrThrow(mRobotSkeleton, disabledPair.link1_);
    auto body1 = getBodyNodeOrThrow(mRobotSkeleton, disabledPair.link2_);

#ifndef NDEBUG
    dtinfo << "Disabled collisions between " << disabledPair.link1_ << " and "
           << disabledPair.link2_ << std::endl;
#endif

    selfCollisionFilter->addBodyNodePairToBlackList(body0, body1);
  }

  if (!mSimulation)
  {
    if (!node)
    {
      mNode = make_unique<::ros::NodeHandle>();
    }
    else
    {
      mNode = make_unique<::ros::NodeHandle>(*node);
    }

    mControllerServiceClient = make_unique<::ros::ServiceClient>(
        mNode->serviceClient<controller_manager_msgs::SwitchController>(
            "controller_manager/switch_controller"));
    mJointStateClient = make_unique<RosJointStateClient>(
        mRobotSkeleton, *mNode, "/joint_states", 1);
    mJointStateThread = make_unique<ExecutorThread>(
        std::bind(&RosJointStateClient::spin, mJointStateClient.get()),
        jointUpdateCycle);
    ros::Duration(0.3).sleep(); // first callback at around 0.12 - 0.25 seconds
  }

  mSpace = std::make_shared<MetaSkeletonStateSpace>(mRobotSkeleton.get());

  mTrajectoryExecutor = createTrajectoryExecutor();

  // TODO: (GL) Ideally this should be set in the header as const but since
  // it requires mRng I am keeping it here.
  // Create default parameters (otherwise, they are undefined by default in
  // aikido). These parameters are taken mainly from the default constructors of
  // the structs (aikido/robot/util.hpp) and one of the herb demos.
  CRRTPlannerParameters crrtParams(
      &mRng,
      5,
      std::numeric_limits<double>::infinity(),
      0.1,
      0.05,
      0.1,
      20,
      1e-4);

  // Setting arm base and end names
  mArmBaseName = "j2n6s200_link_base";
  mArmEndName = "j2n6s200_link_6";
  mHandBaseName = "j2n6s200_hand_base";

  // Setup the arm
  mArm = configureArm(
      "j2n6s200",
      retriever,
      mTrajectoryExecutor,
      collisionDetector,
      selfCollisionFilter);
  mArm->setCRRTPlannerParameters(crrtParams);
  mArm->setVectorFieldPlannerParameters(vfParams);

  mArmSpace = mArm->getStateSpace();

  // Set up the concrete robot from the meta skeleton
  mRobot = std::make_shared<ConcreteRobot>(
      "adaRobot",
      mRobotSkeleton,
      mSimulation,
      cloneRNG(),
      mTrajectoryExecutor,
      collisionDetector,
      selfCollisionFilter);
  mRobot->setCRRTPlannerParameters(crrtParams);

  // TODO: When the named configurations are set in resources.
  // Load the named configurations
  // auto namedConfigurations = parseYAMLToNamedConfigurations(
  //     aikido::io::loadYAML(namedConfigurationsUri, retriever));
  // mRobot->setNamedConfigurations(namedConfigurations);

  mThread = make_unique<ExecutorThread>(
      std::bind(&Ada::update, this), threadExecutionCycle);
}

//==============================================================================
std::unique_ptr<aikido::trajectory::Spline> Ada::smoothPath(
    const dart::dynamics::MetaSkeletonPtr& metaSkeleton,
    const aikido::trajectory::Trajectory* path,
    const TestablePtr& constraint)
{
  return mRobot->smoothPath(metaSkeleton, path, constraint);
}

//==============================================================================
std::unique_ptr<aikido::trajectory::Spline> Ada::retimePath(
    const dart::dynamics::MetaSkeletonPtr& metaSkeleton,
    const aikido::trajectory::Trajectory* path)
{
  return mRobot->retimePath(metaSkeleton, path);
}

//==============================================================================
std::unique_ptr<aikido::trajectory::Spline> Ada::retimePathWithKunz(
    const dart::dynamics::MetaSkeletonPtr& metaSkeleton,
    const aikido::trajectory::Trajectory* path,
    double maxDeviation,
    double timestep)
{
  return mRobot->retimePathWithKunz(metaSkeleton, path, maxDeviation, timestep);
}

//==============================================================================
std::future<void> Ada::executeTrajectory(const TrajectoryPtr& trajectory) const
{
  return mRobot->executeTrajectory(trajectory);
}

//==============================================================================
boost::optional<Eigen::VectorXd> Ada::getNamedConfiguration(
    const std::string& name) const
{
  return mRobot->getNamedConfiguration(name);
}

//==============================================================================
void Ada::setNamedConfigurations(
    std::unordered_map<std::string, const Eigen::VectorXd> namedConfigurations)
{
  mRobot->setNamedConfigurations(namedConfigurations);
}

//==============================================================================
std::string Ada::getName() const
{
  return mRobot->getName();
}

//==============================================================================
dart::dynamics::ConstMetaSkeletonPtr Ada::getMetaSkeleton() const
{
  return mRobot->getMetaSkeleton();
}

//==============================================================================
ConstMetaSkeletonStateSpacePtr Ada::getStateSpace() const
{
  return mRobot->getStateSpace();
}

//==============================================================================
void Ada::setRoot(Robot* robot)
{
  mRobot->setRoot(robot);
}

//==============================================================================
void Ada::step(const std::chrono::system_clock::time_point& timepoint)
{
  std::lock_guard<std::mutex> lock(mRobotSkeleton->getMutex());
  mRobot->step(timepoint);
  mArm->step(timepoint);

  if (!mSimulation)
  {
    auto armSkeleton = mRobot->getMetaSkeleton();
    armSkeleton->setPositions(
        mJointStateClient->getLatestPosition(*armSkeleton));
  }
}

//==============================================================================
CollisionFreePtr Ada::getSelfCollisionConstraint(
    const ConstMetaSkeletonStateSpacePtr& space,
    const dart::dynamics::MetaSkeletonPtr& metaSkeleton) const
{
  return mRobot->getSelfCollisionConstraint(space, metaSkeleton);
}

//==============================================================================
TestablePtr Ada::getFullCollisionConstraint(
    const ConstMetaSkeletonStateSpacePtr& space,
    const dart::dynamics::MetaSkeletonPtr& metaSkeleton,
    const CollisionFreePtr& collisionFree) const
{
  return mRobot->getFullCollisionConstraint(space, metaSkeleton, collisionFree);
}
//==============================================================================
std::unique_ptr<aikido::common::RNG> Ada::cloneRNG()
{
  return std::move(cloneRNGFrom(mRng)[0]);
}

//==============================================================================
aikido::planner::WorldPtr Ada::getWorld() const
{
  return mWorld;
}

//==============================================================================
ConcreteManipulatorPtr Ada::getArm()
{
  return mArm;
}

//==============================================================================
ConstConcreteManipulatorPtr Ada::getArm() const
{
  return mArm;
}

//==============================================================================
AdaHandPtr Ada::getHand()
{
  return mHand;
}

//==============================================================================
ConstAdaHandPtr Ada::getHand() const
{
  return mHand;
}

//==============================================================================
void Ada::update()
{
  step(std::chrono::system_clock::now());
}

//==============================================================================
TrajectoryPtr Ada::planToConfiguration(
    const MetaSkeletonStateSpacePtr& space,
    const dart::dynamics::MetaSkeletonPtr& metaSkeleton,
    const aikido::statespace::StateSpace::State* goalState,
    const CollisionFreePtr& collisionFree,
    double timelimit)
{
  return mRobot->planToConfiguration(
      space, metaSkeleton, goalState, collisionFree, timelimit);
}

//==============================================================================
TrajectoryPtr Ada::planToConfiguration(
    const MetaSkeletonStateSpacePtr& space,
    const dart::dynamics::MetaSkeletonPtr& metaSkeleton,
    const Eigen::VectorXd& goal,
    const CollisionFreePtr& collisionFree,
    double timelimit)
{
  return mRobot->planToConfiguration(
      space, metaSkeleton, goal, collisionFree, timelimit);
}

//==============================================================================
TrajectoryPtr Ada::planToConfigurations(
    const MetaSkeletonStateSpacePtr& space,
    const dart::dynamics::MetaSkeletonPtr& metaSkeleton,
    const std::vector<StateSpace::State*>& goalStates,
    const CollisionFreePtr& collisionFree,
    double timelimit)
{
  return mRobot->planToConfigurations(
      space, metaSkeleton, goalStates, collisionFree, timelimit);
}

//==============================================================================
TrajectoryPtr Ada::planToConfigurations(
    const MetaSkeletonStateSpacePtr& space,
    const dart::dynamics::MetaSkeletonPtr& metaSkeleton,
    const std::vector<Eigen::VectorXd>& goals,
    const CollisionFreePtr& collisionFree,
    double timelimit)
{
  return mRobot->planToConfigurations(
      space, metaSkeleton, goals, collisionFree, timelimit);
}

//==============================================================================
TrajectoryPtr Ada::planToTSR(
    const MetaSkeletonStateSpacePtr& space,
    const dart::dynamics::MetaSkeletonPtr& metaSkeleton,
    const dart::dynamics::BodyNodePtr& bn,
    const TSRPtr& tsr,
    const CollisionFreePtr& collisionFree,
    double timelimit,
    size_t maxNumTrials,
    const aikido::distance::ConfigurationRankerPtr& ranker)
{
  return mRobot->planToTSR(
      space,
      metaSkeleton,
      bn,
      tsr,
      collisionFree,
      timelimit,
      maxNumTrials,
      ranker);
}

//==============================================================================
TrajectoryPtr Ada::planToTSRwithTrajectoryConstraint(
    const MetaSkeletonStateSpacePtr& space,
    const dart::dynamics::MetaSkeletonPtr& metaSkeleton,
    const dart::dynamics::BodyNodePtr& bodyNode,
    const TSRPtr& goalTsr,
    const TSRPtr& constraintTsr,
    const CollisionFreePtr& collisionFree,
    double timelimit)
{
  return mRobot->planToTSRwithTrajectoryConstraint(
      space,
      metaSkeleton,
      bodyNode,
      goalTsr,
      constraintTsr,
      collisionFree,
      timelimit);
}

//==============================================================================
TrajectoryPtr Ada::planToNamedConfiguration(
    const std::string& name,
    const CollisionFreePtr& collisionFree,
    double timelimit)
{
  return mRobot->planToNamedConfiguration(name, collisionFree, timelimit);
}

//==============================================================================
bool Ada::startTrajectoryExecutor()
{
  return switchControllers(
      std::vector<std::string>{mArmTrajectoryExecutorName,
                               mHandTrajectoryExecutorName},
      std::vector<std::string>());
}

//==============================================================================
bool Ada::stopTrajectoryExecutor()
{
  return switchControllers(
      std::vector<std::string>(),
      std::vector<std::string>{mArmTrajectoryExecutorName,
                               mHandTrajectoryExecutorName});
}

//=============================================================================
void Ada::setCRRTPlannerParameters(const CRRTPlannerParameters& crrtParameters)
{
  mRobot->setCRRTPlannerParameters(crrtParameters);
}

//=============================================================================
void Ada::setVectorFieldPlannerParameters(
    const VectorFieldPlannerParameters& vfParameters)
{
  mArm->setVectorFieldPlannerParameters(vfParameters);
}

//==============================================================================
ConcreteManipulatorPtr Ada::configureArm(
    const std::string& armName,
    const dart::common::ResourceRetrieverPtr& retriever,
    const TrajectoryExecutorPtr& executor,
    dart::collision::CollisionDetectorPtr collisionDetector,
    const std::shared_ptr<dart::collision::BodyNodeCollisionFilter>&
        selfCollisionFilter)
{
  using dart::dynamics::Chain;

  auto armBase = getBodyNodeOrThrow(mRobotSkeleton, mArmBaseName);
  auto armEnd = getBodyNodeOrThrow(mRobotSkeleton, mArmEndName);

  auto arm = Chain::create(armBase, armEnd, armName);
  auto armSpace = std::make_shared<MetaSkeletonStateSpace>(arm.get());

  mHand = std::make_shared<AdaHand>(
      armName,
      mSimulation,
      getBodyNodeOrThrow(mRobotSkeleton, mHandBaseName),
      getBodyNodeOrThrow(mRobotSkeleton, mEndEffectorName),
      selfCollisionFilter,
      mNode.get(),
      retriever);

  // Hardcoding to acceleration limits used in OpenRAVE
  // This is necessary because ADA is loaded from URDF, which
  // provides no means of specifying acceleration limits
  // TODO : update acceleration limits by checking Kinova spec.
  arm->setAccelerationLowerLimits(
      Eigen::VectorXd::Constant(arm->getNumDofs(), -2.0));
  arm->setAccelerationUpperLimits(
      Eigen::VectorXd::Constant(arm->getNumDofs(), 2.0));

  auto manipulatorRobot = std::make_shared<ConcreteRobot>(
      armName,
      arm,
      mSimulation,
      cloneRNG(),
      executor,
      collisionDetector,
      selfCollisionFilter);

  auto manipulator
      = std::make_shared<ConcreteManipulator>(manipulatorRobot, mHand);
  return manipulator;
}

//==============================================================================
Eigen::VectorXd Ada::getCurrentConfiguration() const
{
  return mRobot->getMetaSkeleton()->getPositions();
}

//==============================================================================
Eigen::VectorXd Ada::getVelocityLimits() const
{
  return mRobot->getMetaSkeleton()->getVelocityUpperLimits();
}

//==============================================================================
Eigen::VectorXd Ada::getAccelerationLimits() const
{
  return mRobot->getMetaSkeleton()->getAccelerationUpperLimits();
}

//==============================================================================
std::shared_ptr<aikido::control::TrajectoryExecutor>
Ada::createTrajectoryExecutor()
{
  using aikido::control::KinematicSimulationTrajectoryExecutor;
  using aikido::control::ros::RosTrajectoryExecutor;

  if (mSimulation)
  {
    return std::make_shared<KinematicSimulationTrajectoryExecutor>(
        mRobotSkeleton);
  }
  else
  {
    std::string serverName
        = mArmTrajectoryExecutorName + "/follow_joint_trajectory";
    return std::make_shared<RosTrajectoryExecutor>(
        *mNode,
        serverName,
        rosTrajectoryInterpolationTimestep,
        rosTrajectoryGoalTimeTolerance);
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
  srv.request.start_controllers = startControllers;
  srv.request.stop_controllers = stopControllers;
  srv.request.strictness
      = controller_manager_msgs::SwitchControllerRequest::STRICT;

  if (mControllerServiceClient->call(srv) && srv.response.ok)
    return true;
  else
    throw std::runtime_error("SwitchController failed.");
}

//==============================================================================
aikido::control::TrajectoryExecutorPtr Ada::getTrajectoryExecutor()
{
  return mTrajectoryExecutor;
}

//==============================================================================
aikido::trajectory::TrajectoryPtr Ada::planArmToTSR(
    const aikido::constraint::dart::TSR& tsr,
    const aikido::constraint::dart::CollisionFreePtr& collisionFree,
    double timelimit,
    size_t maxNumTrials,
    const aikido::distance::ConfigurationRankerPtr& ranker)

{
  auto goalTSR = std::make_shared<aikido::constraint::dart::TSR>(tsr);

  return planToTSR(
      mArmSpace,
      mArm->getMetaSkeleton(),
      mHand->getEndEffectorBodyNode(),
      goalTSR,
      collisionFree,
      timelimit,
      maxNumTrials,
      ranker);
}

//==============================================================================
bool Ada::moveArmToTSR(
    const aikido::constraint::dart::TSR& tsr,
    const aikido::constraint::dart::CollisionFreePtr& collisionFree,
    double timelimit,
    size_t maxNumTrials,
    const aikido::distance::ConfigurationRankerPtr& ranker,
    const std::vector<double>& velocityLimits,
    TrajectoryPostprocessType postprocessType)
{
  auto trajectory
      = planArmToTSR(tsr, collisionFree, timelimit, maxNumTrials, ranker);

  if (!trajectory)
    return false;

  return moveArmOnTrajectory(
      trajectory, collisionFree, postprocessType, velocityLimits);
}

//==============================================================================
bool Ada::moveArmToEndEffectorOffset(
    const Eigen::Vector3d& direction,
    double length,
    const aikido::constraint::dart::CollisionFreePtr& collisionFree,
    double timelimit,
    double positionTolerance,
    double angularTolerance,
    const std::vector<double>& velocityLimits)
{
  auto traj = planArmToEndEffectorOffset(
      direction,
      length,
      collisionFree,
      timelimit,
      positionTolerance,
      angularTolerance);

  if (!traj)
    return false;

  return moveArmOnTrajectory(traj, collisionFree, KUNZ, velocityLimits);
}

//==============================================================================
aikido::trajectory::TrajectoryPtr Ada::planArmToEndEffectorOffset(
    const Eigen::Vector3d& direction,
    double length,
    const aikido::constraint::dart::CollisionFreePtr& collisionFree,
    double timelimit,
    double positionTolerance,
    double angularTolerance)
{
  auto trajectory = mArm->planToEndEffectorOffset(
      mArmSpace,
      mArm->getMetaSkeleton(),
      mHand->getEndEffectorBodyNode(),
      collisionFree,
      direction,
      length,
      timelimit,
      positionTolerance,
      angularTolerance);

  return trajectory;
}

//==============================================================================
bool Ada::moveArmToConfiguration(
    const Eigen::Vector6d& configuration,
    const aikido::constraint::dart::CollisionFreePtr& collisionFree,
    double timelimit)
{
  auto trajectory = planToConfiguration(
      mArmSpace,
      mArm->getMetaSkeleton(),
      configuration,
      collisionFree,
      timelimit);

  return moveArmOnTrajectory(trajectory, collisionFree, KUNZ);
}

//==============================================================================
bool Ada::moveArmOnTrajectory(
    aikido::trajectory::TrajectoryPtr trajectory,
    const aikido::constraint::dart::CollisionFreePtr& collisionFree,
    TrajectoryPostprocessType postprocessType,
    std::vector<double> smoothVelocityLimits)
{
  if (!trajectory)
  {
    throw std::runtime_error("Trajectory execution failed: Empty trajectory.");
  }

  std::vector<aikido::constraint::ConstTestablePtr> constraints;

  if (collisionFree)
  {
    constraints.push_back(collisionFree);
  }
  auto testable = std::make_shared<aikido::constraint::TestableIntersection>(
      mArmSpace, constraints);

  aikido::trajectory::TrajectoryPtr timedTrajectory;

  auto armSkeleton = mArm->getMetaSkeleton();

  switch (postprocessType)
  {
    case PARABOLIC_RETIME:
      timedTrajectory = retimePath(armSkeleton, trajectory.get());
      break;

    case PARABOLIC_SMOOTH:
      if (smoothVelocityLimits.size() == 6)
      {
        Eigen::Vector6d velocityLimits;
        velocityLimits << smoothVelocityLimits[0], smoothVelocityLimits[1],
            smoothVelocityLimits[2], smoothVelocityLimits[3],
            smoothVelocityLimits[4], smoothVelocityLimits[5];
        Eigen::VectorXd previousLowerLimits
            = mArm->getMetaSkeleton()->getVelocityLowerLimits();
        Eigen::VectorXd previousUpperLimits
            = mArm->getMetaSkeleton()->getVelocityUpperLimits();
        armSkeleton->setVelocityLowerLimits(-velocityLimits);
        armSkeleton->setVelocityUpperLimits(velocityLimits);
        timedTrajectory = smoothPath(armSkeleton, trajectory.get(), testable);
        armSkeleton->setVelocityLowerLimits(previousLowerLimits);
        armSkeleton->setVelocityUpperLimits(previousUpperLimits);
      }
      else
      {
        timedTrajectory = smoothPath(armSkeleton, trajectory.get(), testable);
      }
      break;

    case KUNZ:
    {
      timedTrajectory = retimePathWithKunz(
          armSkeleton, trajectory.get(), kunzMaxDeviation, kunzTimeStep);

      if (!timedTrajectory)
      {
        // If using kunz retimer fails, fall back to parabolic timing
        timedTrajectory = retimePath(armSkeleton, trajectory.get());
      }

      if (!timedTrajectory)
      {
        throw std::runtime_error("Retiming failed");
      }
      break;
    }

    default:
      throw std::invalid_argument("Unexpected trajectory post processing type");
  }

  auto future = executeTrajectory(std::move(timedTrajectory));
  try
  {
    future.get();
  }
  catch (const std::exception& e)
  {
    dtwarn << "Exception in trajectoryExecution: " << e.what() << std::endl;
    return false;
  }
  return true;
}

//==============================================================================
void Ada::openHand()
{
  mHand->executePreshape("open").wait();
}

//==============================================================================
void Ada::closeHand()
{
  mHand->executePreshape("closed").wait();
}

} // namespace ada
