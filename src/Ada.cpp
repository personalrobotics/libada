#include "libada/Ada.hpp"

#include <cassert>
#include <mutex>
#include <stdexcept>
#include <string>

#include <aikido/common/RNG.hpp>
#include <aikido/constraint/CyclicSampleable.hpp>
#include <aikido/constraint/FiniteSampleable.hpp>
#include <aikido/constraint/NewtonsMethodProjectable.hpp>
#include <aikido/constraint/Testable.hpp>
#include <aikido/constraint/TestableIntersection.hpp>
#include <aikido/constraint/Satisfied.hpp>
#include <aikido/control/KinematicSimulationTrajectoryExecutor.hpp>
#include <aikido/control/ros/RosTrajectoryExecutor.hpp>
#include <aikido/distance/defaults.hpp>
#include <aikido/io/yaml.hpp>
#include <aikido/planner/PlanningResult.hpp>
#include <aikido/planner/SnapPlanner.hpp>
#include <aikido/planner/ompl/CRRTConnect.hpp>
#include <aikido/planner/ompl/Planner.hpp>
#include <aikido/planner/vectorfield/VectorFieldPlanner.hpp>
#include <aikido/robot/ConcreteRobot.hpp>
#include <aikido/robot/ConcreteManipulator.hpp>
#include <aikido/robot/util.hpp>
#include <aikido/statespace/GeodesicInterpolator.hpp>
#include <aikido/statespace/dart/MetaSkeletonStateSpace.hpp>
#include <controller_manager_msgs/SwitchController.h>
#include <dart/common/Console.hpp>
#include <dart/common/StlHelpers.hpp>
#include <dart/common/Timer.hpp>
#include <dart/utils/urdf/DartLoader.hpp>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <srdfdom/model.h>
#include <urdf/model.h>

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

const dart::common::Uri adaUrdfUri{"package://ada_description/robots/ada_accessories.urdf"};
const dart::common::Uri adaSrdfUri{"package://ada_description/robots/ada_accessories.srdf"};
const dart::common::Uri namedConfigurationsUri{
    "package://libada/resources/configurations.yaml"};
const std::vector<std::string> gravityCompensationControllers
    = {"gravity_compensation_controller"};
const std::vector<std::string> trajectoryExecutors
    = {"trajectory_controller"};
// TODO define in ada_launch 

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
  const ::ros::NodeHandle* node,
  aikido::common::RNG::result_type rngSeed,
  const dart::common::Uri& adaUrdfUri,
  const dart::common::ResourceRetrieverPtr& retriever)
  : mSimulation(simulation) 
  , mCollisionResolution(collisionResolution)
  , mRng(rngSeed)
  , mSmootherFeasibilityCheckResolution(1e-3)
  , mSmootherFeasibilityApproxTolerance(1e-3)
  , mWorld(std::move(env))
{
  simulation = true; // temporarily set simulation to true

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

  // Manually set the acceleration limit
  mRobotSkeleton->setAccelerationLowerLimits(
      Eigen::VectorXd::Constant(mRobotSkeleton->getNumDofs(), -2.0));
  mRobotSkeleton->setAccelerationUpperLimits(
      Eigen::VectorXd::Constant(mRobotSkeleton->getNumDofs(), 2.0));

  // Define the collision detector and groups
  auto collisionDetector = FCLCollisionDetector::create();
  //auto collideWith = collisionDetector->createCollisionGroupAsSharedPtr();
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

   // TODO
   mControllerServiceClient = make_unique<::ros::ServiceClient>(
        mNode->serviceClient<controller_manager_msgs::SwitchController>(
            "controller_manager/switch_controller"));
   mJointStateClient
       = make_unique<RosJointStateClient>(mRobotSkeleton, *mNode, "/joint_states", 1);
   mJointStateThread = make_unique<ExecutorThread>(
       std::bind(&RosJointStateClient::spin, mJointStateClient.get()),
       jointUpdateCycle);
   ros::Duration(0.3).sleep(); // first callback at around 0.12 - 0.25 seconds
  }
 
  mSpace = std::make_shared<MetaSkeletonStateSpace>(mRobotSkeleton.get());

  mTrajectoryExecutor = createTrajectoryExecutor();

  // TODO: change Smoother/Timer to not take a testable in constructor.
  auto testable = std::make_shared<aikido::constraint::Satisfied>(mSpace);

  // create default parameters (otherwise, they are undefined by default in aikido)
  CRRTPlannerParameters crrtParams(&mRng, 5, std::numeric_limits<double>::infinity(), 0.1, 0.05, 0.1, 20, 1e-4);
  VectorFieldPlannerParameters vfParams(0.2, 0.001, 0.001, 0.001, 1e-3, 1e-3, 1.0, 0.2, 0.1);

    // Setup the arm
  mArm = configureArm("j2n6s200", retriever, mTrajectoryExecutor,
        collisionDetector, selfCollisionFilter);
  mArm->setCRRTPlannerParameters(crrtParams);
  mArm->setVectorFieldPlannerParameters(vfParams);

  // Set up the concrete robot from the meta skeleton
  mRobot = std::make_shared<ConcreteRobot>("adaRobot", mRobotSkeleton,
        mSimulation, cloneRNG(), mTrajectoryExecutor,
        collisionDetector, selfCollisionFilter);
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
std::future<void> Ada::executeTrajectory(
      const TrajectoryPtr& trajectory) const
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
      std::unordered_map<std::string,
      const Eigen::VectorXd> namedConfigurations)
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
    auto armSkeleton = mRobot-> getMetaSkeleton();
    armSkeleton->setPositions(mJointStateClient->getLatestPosition(*armSkeleton));
  }
}

//==============================================================================
CollisionFreePtr Ada::getSelfCollisionConstraint(
  const MetaSkeletonStateSpacePtr& space,
  const dart::dynamics::MetaSkeletonPtr& metaSkeleton) const
{
  return mRobot->getSelfCollisionConstraint(space, metaSkeleton);
}


//==============================================================================
TestablePtr Ada::getFullCollisionConstraint(
      const MetaSkeletonStateSpacePtr& space,
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
HandPtr Ada::getHand()
{
  return mArm->getHand();
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
  return mRobot->planToConfigurations(space, metaSkeleton,
      goals, collisionFree, timelimit);
}

//==============================================================================
TrajectoryPtr Ada::planToTSR(
      const MetaSkeletonStateSpacePtr& space,
      const dart::dynamics::MetaSkeletonPtr& metaSkeleton,
      const dart::dynamics::BodyNodePtr& bn,
      const TSRPtr& tsr,
      const CollisionFreePtr& collisionFree,
      double timelimit,
      size_t maxNumTrials)
{
  return mRobot->planToTSR(space, metaSkeleton, bn, tsr, collisionFree, 
      timelimit, maxNumTrials);
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
      space, metaSkeleton, bodyNode, goalTsr, constraintTsr,
      collisionFree, timelimit);
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
bool Ada::switchFromGravityCompensationControllersToTrajectoryExecutors()
{
  return switchControllers(trajectoryExecutors, gravityCompensationControllers);
}

//==============================================================================
bool Ada::switchFromTrajectoryExecutorsToGravityCompensationControllers()
{
 return switchControllers(gravityCompensationControllers, trajectoryExecutors);
}

//=============================================================================
void Ada::setCRRTPlannerParameters(
      const CRRTPlannerParameters& crrtParameters)
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
    //dart::collision::CollisionGroupPtr collideWith,
    const std::shared_ptr<dart::collision::BodyNodeCollisionFilter>&
      selfCollisionFilter)
{
  using dart::dynamics::Chain;

  std::stringstream armBaseName;
  armBaseName << "j2n6s200_link_base";

  std::stringstream armEndName;
  armEndName << "j2n6s200_link_6";

  std::stringstream endEffectorName;
  endEffectorName << "j2n6s200_forque_end_effector";

  auto armBase = getBodyNodeOrThrow(mRobotSkeleton, armBaseName.str());
  auto armEnd = getBodyNodeOrThrow(mRobotSkeleton, armEndName.str());

  auto arm = Chain::create(armBase, armEnd, armName);
  auto armSpace = std::make_shared<MetaSkeletonStateSpace>(arm.get());

  auto hand = std::make_shared<AdaHand>(
       armName,
       mSimulation,
       getBodyNodeOrThrow(mRobotSkeleton, endEffectorName.str()),
       selfCollisionFilter,
       mNode.get(),
       retriever);

  // Hardcoding to acceleration limits used in OpenRAVE
  // This is necessary because ADA is loaded from URDF, which
  // provides no means of specifying acceleration limits
  // TODO : update acceleration limits after hearing back from HEBI.us
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
      //collideWith,
      selfCollisionFilter);

  auto manipulator = std::make_shared<ConcreteManipulator>(
      manipulatorRobot, hand);

  return manipulator;
}

//==============================================================================
Eigen::VectorXd Ada::getCurrentConfiguration() const
{
  return mRobot->getMetaSkeleton()->getPositions();
}

//==============================================================================
// TODO : fill the right value in URDF
Eigen::VectorXd Ada::getVelocityLimits(
    dart::dynamics::MetaSkeleton& metaSkeleton) const
{
  // The arm composes of the following actuators:
  //  8-3 8-9 8-9 x5-1 x5-1
  // Speed Limit (RPM), according to specification
  //  84 30 30 90 90
  return mRobot->getMetaSkeleton()->getVelocityUpperLimits();
}   

//==============================================================================
Eigen::VectorXd Ada::getAccelerationLimits(
    dart::dynamics::MetaSkeleton& metaSkeleton) const
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
    return std::make_shared<KinematicSimulationTrajectoryExecutor>(mRobotSkeleton);
  }
  else
  {
    // TODO (k):need to check trajectory_controller exists?
    std::string serverName
        = "trajectory_controller/follow_joint_trajectory";
    return std::make_shared<RosTrajectoryExecutor>(
        *mNode,
        serverName,
        rosTrajectoryInterpolationTimestep,
        rosTrajectoryGoalTimeTolerance);
  }
}

//==============================================================================
bool Ada::switchControllers(
    const std::vector<std::string>& start_controllers,
    const std::vector<std::string>& stop_controllers)
{
  if (!mNode)
    throw std::runtime_error("Ros node has not been instantiated.");

  if (!mControllerServiceClient)
    throw std::runtime_error("ServiceClient not instantiated.");

  controller_manager_msgs::SwitchController srv;
  srv.request.start_controllers = start_controllers;
  srv.request.stop_controllers = stop_controllers;
  srv.request.strictness
      = controller_manager_msgs::SwitchControllerRequest::STRICT;

  if (mControllerServiceClient->call(srv))
    return srv.response.ok;
  else
    throw std::runtime_error("SwitchController failed.");
}

} // ada
