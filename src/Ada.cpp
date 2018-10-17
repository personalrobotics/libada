#include "libada/Ada.hpp"

#include <algorithm>
#include <cassert>
#include <mutex>
#include <stdexcept>
#include <string>

#include <aikido/common/RNG.hpp>
#include <aikido/common/Spline.hpp>
#include <aikido/constraint/CyclicSampleable.hpp>
#include <aikido/constraint/FiniteSampleable.hpp>
#include <aikido/constraint/NewtonsMethodProjectable.hpp>
#include <aikido/constraint/Satisfied.hpp>
#include <aikido/constraint/Testable.hpp>
#include <aikido/constraint/TestableIntersection.hpp>
#include <aikido/control/KinematicSimulationTrajectoryExecutor.hpp>
#include <aikido/control/ros/RosTrajectoryExecutor.hpp>
#include <aikido/distance/defaults.hpp>
#include <aikido/io/yaml.hpp>
#include <aikido/planner/PlanningResult.hpp>
#include <aikido/planner/SnapPlanner.hpp>
#include <aikido/planner/ompl/CRRTConnect.hpp>
#include <aikido/planner/ompl/Planner.hpp>
#include <aikido/planner/vectorfield/VectorFieldPlanner.hpp>
#include <aikido/robot/ConcreteManipulator.hpp>
#include <aikido/robot/ConcreteRobot.hpp>
#include <aikido/robot/util.hpp>
#include <aikido/statespace/GeodesicInterpolator.hpp>
#include <aikido/statespace/dart/MetaSkeletonStateSpace.hpp>
#include <controller_manager_msgs/SwitchController.h>
#include <dart/utils/urdf/urdf.hpp>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <srdfdom/model.h>
#include <urdf/model.h>

#include "external/optimal_trajectory_following/Path.h"
#include "external/optimal_trajectory_following/Trajectory.h"

namespace ada {

using aikido::control::TrajectoryExecutorPtr;
using aikido::constraint::dart::CollisionFreePtr;
using aikido::constraint::dart::TSR;
using aikido::constraint::dart::TSRPtr;
using aikido::constraint::TestablePtr;
using aikido::distance::createDistanceMetric;
using aikido::planner::parabolic::ParabolicSmoother;
using aikido::planner::parabolic::ParabolicTimer;
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
using aikido::statespace::GeodesicInterpolator;
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
using dart::dynamics::ChainPtr;
using dart::dynamics::InverseKinematics;
using dart::dynamics::MetaSkeleton;
using dart::dynamics::MetaSkeletonPtr;
using dart::dynamics::SkeletonPtr;

dart::common::Uri defaultAdaUrdfUri{
    "package://ada_description/robots_urdf/ada_with_camera_forque.urdf"};
dart::common::Uri defaultAdaSrdfUri{
    "package://ada_description/robots_urdf/ada_with_camera_forque.srdf"};

const dart::common::Uri namedConfigurationsUri{
    "package://libada/resources/configurations.yaml"};

// arm trajectory controllers that are meant to be used by ada.
// needs to be consistent with the configurations in ada_launch
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
    std::cout << "[INFO] Disabled collisions between " << disabledPair.link1_
              << " and " << disabledPair.link2_ << std::endl;
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

  // TODO: change Smoother/Timer to not take a testable in constructor.
  auto testable = std::make_shared<aikido::constraint::Satisfied>(mSpace);

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
  VectorFieldPlannerParameters vfParams(
      0.2, 0.001, 0.004, 0.001, 1e-3, 1e-3, 1.0, 0.2, 0.1);

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
aikido::trajectory::TrajectoryPtr Ada::convertTrajectory(
    const dart::dynamics::MetaSkeletonPtr& metaSkeleton,
    const aikido::trajectory::Trajectory* path)
{

  auto interpolated = dynamic_cast<const Interpolated*>(path);
  auto interpolator
      = dynamic_cast<const aikido::statespace::GeodesicInterpolator*>(
          (interpolated->getInterpolator()).get());

  auto numWaypoints = interpolated->getNumWaypoints();

  auto space
      = std::make_shared<aikido::statespace::dart::MetaSkeletonStateSpace>(
          metaSkeleton.get());

  // Create a new statespace with appropriate subspaces
  std::cout << "Creating a new compound space" << std::endl;
  std::vector<aikido::statespace::ConstStateSpacePtr> subspaces;
  for (std::size_t i = 0; i < path->getStateSpace()->getDimension(); ++i)
  {
    subspaces.emplace_back(std::make_shared<aikido::statespace::R1>());
  }

  auto compoundSpace
      = std::make_shared<const aikido::statespace::CartesianProduct>(subspaces);

  // Create the corresponding interpolator
  std::cout << "Creating a new interpolator" << std::endl;
  auto compoundInterpolator
      = std::make_shared<aikido::statespace::GeodesicInterpolator>(
          compoundSpace);

  // Create the new trajectory
  std::cout << "Creating a new trajectory" << std::endl;
  auto returnTrajectory = std::make_shared<aikido::trajectory::Interpolated>(
      space, compoundInterpolator);

  // Add the first waypoint
  std::cout << "Adding the first waypoint" << std::endl;
  Eigen::VectorXd position(mSpace->getDimension());
  auto state = mSpace->createState();
  space->copyState(interpolated->getWaypoint(0), state);
  space->convertStateToPositions(state, position);
  aikido::statespace::CartesianProduct::ScopedState s1
      = compoundSpace->createState();

  Eigen::VectorXd pos(1);
  std::cout << "First point is: " << position.transpose() << std::endl;
  for (auto i = 0; i < position.size(); ++i)
  {
    pos << position[i];
    s1.getSubStateHandle<aikido::statespace::R1>(i).setValue(pos);
  }
  returnTrajectory->addWaypoint(0, s1);


  for (std::size_t i = 0; i < numWaypoints - 1; ++i)
  {
    auto currWaypoint = interpolated->getWaypoint(i);
    auto nextWaypoint = interpolated->getWaypoint(i + 1);
    space->copyState(nextWaypoint, state);
    space->convertStateToPositions(state, position);
    std::cout << "Old position is: " << position.transpose() << std::endl;


    auto diff = interpolator->getTangentVector(currWaypoint, nextWaypoint);
    space->copyState(currWaypoint, state);
    space->convertStateToPositions(state, position);
    position += diff;
    std::cout << "New position is: " << position.transpose() << std::endl;
    for (auto j = 0; j < position.size(); ++j)
    {
      pos << position[j];
      s1.getSubStateHandle<aikido::statespace::R1>(j).setValue(pos);
    }
    returnTrajectory->addWaypoint(i + 1, s1);
  }
  std::cin.get();
  return returnTrajectory;
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
    const Eigen::VectorXd& nominalPosition,
    const CollisionFreePtr& collisionFree,
    double timelimit,
    size_t maxNumTrials)
{
  return mRobot->planToTSR(
      space,
      metaSkeleton,
      bn,
      tsr,
      nominalPosition,
      collisionFree,
      timelimit,
      maxNumTrials);
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
TrajectoryPtr Ada::planToEndEffectorOffset(
    const aikido::statespace::dart::MetaSkeletonStateSpacePtr& space,
    const dart::dynamics::MetaSkeletonPtr& metaSkeleton,
    const dart::dynamics::BodyNodePtr& body,
    const aikido::constraint::dart::CollisionFreePtr& collisionFree,
    const Eigen::Vector3d& direction,
    double distance,
    double timelimit,
    double positionTolerance,
    double angularTolerance)
{
  return mArm->planToEndEffectorOffset(
      space,
      metaSkeleton,
      body,
      collisionFree,
      direction,
      distance,
      timelimit,
      positionTolerance,
      angularTolerance);
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
      // collideWith,
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

std::shared_ptr<aikido::control::TrajectoryExecutor>
Ada::getTrajectoryExecutor()
{
  return mTrajectoryExecutor;
}

std::unique_ptr<aikido::trajectory::Spline> Ada::retimeTimeOptimalPath(
    const dart::dynamics::MetaSkeletonPtr& metaSkeleton,
    const aikido::trajectory::Trajectory* path)
{
  double MAX_DEVIATION = 1e-5;
  double TIME_STEP = 0.002;

  // get max velocities and accelerantions
  Eigen::VectorXd maxVelocities(metaSkeleton->getNumDofs());
  Eigen::VectorXd maxAccelerations(metaSkeleton->getNumDofs());
  for (std::size_t i = 0; i < metaSkeleton->getNumDofs(); i++)
  {
    maxVelocities(i) = std::min(
        std::abs(metaSkeleton->getVelocityUpperLimit(i)),
        std::abs(metaSkeleton->getVelocityLowerLimit(i)));
    maxAccelerations(i) = std::min(
        std::abs(metaSkeleton->getAccelerationUpperLimit(i)),
        std::abs(metaSkeleton->getAccelerationLowerLimit(i)));
  }

  // create waypoints from path
  std::list<Eigen::VectorXd> waypoints;
  auto interpolated
      = dynamic_cast<const aikido::trajectory::Interpolated*>(path);

  auto space
      = std::make_shared<aikido::statespace::dart::MetaSkeletonStateSpace>(
          metaSkeleton.get());

  Eigen::VectorXd position(metaSkeleton->getNumDofs());
  if (interpolated)
  {
    auto state = space->createState();

    auto interpolator
      = dynamic_cast<const aikido::statespace::GeodesicInterpolator*>(
          (interpolated->getInterpolator()).get());

    // Push the first point
    auto firstPoint = interpolated->getWaypoint(0);
    space->copyState(firstPoint, state);
    space->convertStateToPositions(state, position);
    waypoints.push_back(position);

    Eigen::VectorXd previousPosition(space->getDimension());
    for (std::size_t i = 0; i < interpolated->getNumWaypoints() - 1; ++i)
    {
        space->convertPositionsToState(position, state);

        auto nextWaypoint = interpolated->getWaypoint(i + 1);
        auto diff = interpolator->getTangentVector(state, nextWaypoint);

        position += diff;
        waypoints.push_back(position);
    }

//    std::cout << "The configurations pushed for timing are: " << std::endl;
//    for (auto iter = waypoints.begin(); iter != waypoints.end(); ++iter)
//    {
//      std::cout << (*iter).transpose() << std::endl;
//    }
//    std::cin.get();
//  }

  auto spline = dynamic_cast<const aikido::trajectory::Spline*>(path);
  if (spline)
  {
    auto tmpState = path->getStateSpace()->createState();
    Eigen::VectorXd tmpVec(metaSkeleton->getNumDofs());

    spline->getWaypoint(0, tmpState);
    spline->getStateSpace()->logMap(tmpState, tmpVec);
    waypoints.push_back(tmpVec);

    auto interpolator
      = std::make_shared<aikido::statespace::GeodesicInterpolator>(
          space);

    for (std::size_t i = 0; i < spline->getNumWaypoints() - 1; ++i)
    {
      auto state = spline->getStateSpace()->createState();
      spline->getStateSpace()->expMap(tmpVec, tmpState);
      spline->getWaypoint(i + 1, state);
      auto diff = interpolator->getTangentVector(tmpState, state);
      for (int citer = 0; citer < diff.size(); ++citer)
      {
          if (diff(i) > M_PI)
          {
              diff(i) = 2*M_PI - diff(i);
          }
          if (diff(i) < -M_PI)
          {
              diff(i) = 2*M_PI + diff(i);
          }
      }
      tmpVec += diff;
      waypoints.push_back(tmpVec);
    }
//    std::cout << "The configurations pushed for timing are: " << std::endl;
//    for (auto iter = waypoints.begin(); iter != waypoints.end(); ++iter)
//    {
//      std::cout << (*iter).transpose() << std::endl;
//    }
//    std::cin.get();
//  }

  Trajectory trajectory(
      Path(waypoints, MAX_DEVIATION),
      maxVelocities,
      maxAccelerations,
      TIME_STEP);
  if (trajectory.isValid())
  {
    std::cout << "TIME-OPTIMAL RETIMING SUCCEEDED" << std::endl;
    // create spline
    using dart::common::make_unique;

    std::size_t dimension = metaSkeleton->getNumDofs();

    auto stateSpace = path->getStateSpace();
    std::vector<aikido::statespace::ConstStateSpacePtr> subspaces;
    for (std::size_t i = 0; i < path->getStateSpace()->getDimension(); ++i)
    {
      subspaces.emplace_back(std::make_shared<aikido::statespace::R1>());
    }
    auto compoundSpace
        = std::make_shared<const aikido::statespace::CartesianProduct>(subspaces);

    auto outputTrajectory = make_unique<aikido::trajectory::Spline>(stateSpace);

    using CubicSplineProblem = aikido::common::
        SplineProblem<double, int, 4, Eigen::Dynamic, Eigen::Dynamic>;

    const Eigen::VectorXd zeroPosition = Eigen::VectorXd::Zero(dimension);
    auto currState = compoundSpace->createState();
    double currT = 0.0;
    double nextT = TIME_STEP;
    while (currT < trajectory.getDuration())
    {
      const double segmentDuration = nextT - currT;
      Eigen::VectorXd currentPosition = trajectory.getPosition(currT);
      Eigen::VectorXd nextPosition = trajectory.getPosition(nextT);
      Eigen::VectorXd currentVelocity = trajectory.getVelocity(currT);
      Eigen::VectorXd nextVelocity = trajectory.getVelocity(nextT);

      CubicSplineProblem problem(
          Eigen::Vector2d{0., segmentDuration}, 4, dimension);
      problem.addConstantConstraint(1, 0, zeroPosition);
      problem.addConstantConstraint(0, 1, currentVelocity);
      problem.addConstantConstraint(1, 0, nextPosition - currentPosition);
      problem.addConstantConstraint(1, 1, nextVelocity);
      const auto solution = problem.fit();
      const auto coefficients = solution.getCoefficients().front();

      compoundSpace->expMap(currentPosition, currState);
      outputTrajectory->addSegment(coefficients, segmentDuration, currState);

      currT += TIME_STEP;
      nextT += TIME_STEP;
    }

    return outputTrajectory;
  }
  else
  {
    std::cout << "TIME-OPTIMAL RETIMING FAILED" << std::endl;
  }

  return nullptr;
}

} // adamSmootherFeasibilityCheckResolution
