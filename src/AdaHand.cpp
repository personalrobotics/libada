#include "libada/AdaHand.hpp"

#include <chrono>

#include <aikido/control/ros/RosPositionCommandExecutor.hpp>
#include <aikido/planner/World.hpp>

#include "libada/AdaHandKinematicSimulationPositionCommandExecutor.hpp"

namespace ada {

using dart::common::make_unique;

const dart::common::Uri preshapesUri{
    "package://libada/resources/preshapes.yaml"};
const dart::common::Uri tsrEndEffectorTransformsUri{
    "package://libada/resources/tsr_transforms.yaml"};

namespace {

void disablePairwiseSelfCollision(
    const dart::dynamics::BodyNodePtr& singleNode,
    const dart::dynamics::BodyNodePtr& rootNode,
    const std::shared_ptr<dart::collision::BodyNodeCollisionFilter>&
        selfCollisionFilter)
{
#ifndef NDEBUG
  std::cout << "Disabling collision between " << rootNode->getName() << " and "
            << singleNode->getName() << std::endl;
#endif

  selfCollisionFilter->addBodyNodePairToBlackList(rootNode, singleNode);
  for (std::size_t i = 0; i < rootNode->getNumChildBodyNodes(); ++i)
  {
    disablePairwiseSelfCollision(
        singleNode, rootNode->getChildBodyNode(i), selfCollisionFilter);
  }
}

void enablePairwiseSelfCollision(
    const dart::dynamics::BodyNodePtr& singleNode,
    const dart::dynamics::BodyNodePtr& rootNode,
    const std::shared_ptr<dart::collision::BodyNodeCollisionFilter>&
        selfCollisionFilter)
{
#ifndef NDEBUG
  std::cout << "Enabling collision between " << rootNode->getName() << " and "
            << singleNode->getName() << std::endl;
#endif
  selfCollisionFilter->removeBodyNodePairFromBlackList(rootNode, singleNode);
  for (std::size_t i = 0; i < rootNode->getNumChildBodyNodes(); ++i)
  {
    enablePairwiseSelfCollision(
        singleNode, rootNode->getChildBodyNode(i), selfCollisionFilter);
  }
}

} // namespace

//==============================================================================
AdaHand::AdaHand(
    const std::string& name,
    bool simulation,
    dart::dynamics::BodyNodePtr endEffectorBodyNode,
    std::shared_ptr<dart::collision::BodyNodeCollisionFilter>
        selfCollisionFilter,
    const ::ros::NodeHandle* node,
    const dart::common::ResourceRetrieverPtr& retriever)
  : mName(name)
  , mHand(nullptr)
  , mSimulation(simulation)
  , mEndEffectorBodyNode(endEffectorBodyNode)
  , mSelfCollisionFilter(selfCollisionFilter)
  , mGrabMetadata(nullptr)
{
  using dart::dynamics::Branch;

  if (!mSimulation)
  {
    if (!node)
      throw std::runtime_error("ROS node not provided in real.");
    mNode = make_unique<::ros::NodeHandle>(*node);
  }

  mHand = Branch::create(endEffectorBodyNode.get(), mName + "_hand");
  mExecutor
      = createAdaHandPositionExecutor(mEndEffectorBodyNode->getSkeleton());

  loadPreshapes(preshapesUri, retriever);

  loadTSRTransforms(tsrEndEffectorTransformsUri, retriever);
}

//==============================================================================
dart::dynamics::ConstMetaSkeletonPtr AdaHand::getMetaSkeleton() const
{
  return mHand;
}

//==============================================================================
dart::dynamics::MetaSkeletonPtr AdaHand::getMetaSkeleton()
{
  return std::const_pointer_cast<dart::dynamics::MetaSkeleton>(
      const_cast<const AdaHand*>(this)->getMetaSkeleton());
}

//==============================================================================
void AdaHand::grab(const dart::dynamics::SkeletonPtr& bodyToGrab)
{
  using dart::dynamics::Joint;
  using dart::dynamics::FreeJoint;
  using dart::dynamics::WeldJoint;

  // TODO: implement grabbing multiple objects

  // Check if end-effector is already grabbing object
  if (mGrabMetadata)
  {
    std::stringstream ss;
    // TODO: use proper logging
    ss << "[Hand::grab] An end effector may only grab one object."
       << " '" << mEndEffectorBodyNode->getName() << "' is grabbing '"
       << mGrabMetadata->mOldName << "'" << std::endl;

    throw std::runtime_error(ss.str());
  }

  // Assume the skeleton is a single pair of FreeJoint and BodyNode
  if (bodyToGrab->getNumBodyNodes() != 1)
  {
    std::stringstream ss;
    // TODO: use proper logging
    ss << "[Hand::grab] Only Skeletons with one BodyNode may be "
       << "grabbed. Skeleton '" << bodyToGrab->getName() << "' has "
       << bodyToGrab->getNumBodyNodes() << " BodyNodes" << std::endl;

    throw std::runtime_error(ss.str());
  }

  // TODO: this should be Skeleton::getRootJoint() once DART 6.2 is released
  Joint* joint = bodyToGrab->getJoint(0);
  FreeJoint* freeJoint = dynamic_cast<FreeJoint*>(joint);
  if (freeJoint == nullptr)
  {
    std::stringstream ss;
    // TODO: use proper logging
    ss << "[Hand::grab] Only Skeletons with a root FreeJoint may "
       << "be grabbed. Skeleton '" << bodyToGrab->getName() << "' has a "
       << "root " << joint->getType() << std::endl;

    throw std::runtime_error(ss.str());
  }

  // Get fields for GrabMetadata
  auto bodyNode = freeJoint->getChildBodyNode();
  std::string bodyNodeName = bodyNode->getName();
  FreeJoint::Properties jointProperties = freeJoint->getFreeJointProperties();

  // Get relative transform between end effector and BodyNode
  auto endEffectorToBodyTransform
      = bodyNode->getTransform(mEndEffectorBodyNode);

  // Connect grabbed BodyNode to end effector
  WeldJoint::Properties weldJointProperties;
  weldJointProperties.mT_ParentBodyToJoint = endEffectorToBodyTransform;
  bodyNode->moveTo<WeldJoint>(mEndEffectorBodyNode, weldJointProperties);

  // Moving the grabbed object into the same skeleton as the hand means that it
  // will be considered during self-collision checking. Therefore, we need to
  // disable self-collision checking between grabbed object and hand.
  disablePairwiseSelfCollision(
      bodyNode, mEndEffectorBodyNode, mSelfCollisionFilter);

  mGrabMetadata = make_unique<aikido::robot::GrabMetadata>(
      bodyNode, bodyNodeName, bodyToGrab, jointProperties);
}

//==============================================================================
void AdaHand::ungrab()
{
  using dart::dynamics::Joint;
  using dart::dynamics::FreeJoint;

  // Ensure end effector is already grabbing object
  if (!mGrabMetadata)
  {
    std::stringstream ss;

    // TODO: use proper logging
    ss << "[AdaHand::ungrab] End effector \"" << mEndEffectorBodyNode->getName()
       << "\" is not grabbing an object." << std::endl;
    throw std::runtime_error(ss.str());
  }

  // Get grabbed body node and its transform wrt the world
  dart::dynamics::BodyNodePtr grabbedBodyNode = mGrabMetadata->mBodyNode;
  Eigen::Isometry3d grabbedBodyTransform = grabbedBodyNode->getTransform();

  // Re-enable self-collision checking between grabbed object and hand
  enablePairwiseSelfCollision(
      grabbedBodyNode, mEndEffectorBodyNode, mSelfCollisionFilter);

  // Move grabbed BodyNode to root of the old object Skeleton
  dart::dynamics::SkeletonPtr skeleton = mGrabMetadata->mParentSkeleton;
  grabbedBodyNode->moveTo<FreeJoint>(
      skeleton, nullptr, mGrabMetadata->mJointProperties);

  // Set transform of skeleton FreeJoint wrt world
  Joint* joint = skeleton->getJoint(0);
  assert(joint != nullptr);
  FreeJoint* freeJoint = dynamic_cast<FreeJoint*>(joint);
  freeJoint->setTransform(grabbedBodyTransform);

  // Restore old name. If the skeleton of the grabbedBodyNode adds a body with
  // the name oldName while it is removed from the skeleton, then the object
  // cannot be named oldName when it is added back. Instead, DART will rename it
  // to something like oldName(1).
  std::string oldName = mGrabMetadata->mOldName;
  std::string newName = grabbedBodyNode->setName(oldName);
  if (newName != oldName)
  {
    // TODO: use proper logging (warn)
    std::cout << "[Hand::ungrab] Released object was renamed from \"" << oldName
              << "\" to \"" << newName << "\"" << std::endl;
  }

  mGrabMetadata.reset();
}

//==============================================================================
std::future<void> AdaHand::executePreshape(const std::string& preshapeName)
{
  boost::optional<Eigen::VectorXd> preshape = getPreshape(preshapeName);

  if (!preshape)
  {
    std::stringstream message;
    message << "[Hand::executePreshape] Unknown preshape name '" << preshapeName
            << "' specified.";
    throw std::runtime_error(message.str());
  }
  return mExecutor->execute(preshape.get());
}

//==============================================================================
void AdaHand::step(const std::chrono::system_clock::time_point& timepoint)
{
  mExecutor->step(timepoint);
}

//==============================================================================
dart::dynamics::BodyNode* AdaHand::getEndEffectorBodyNode() const
{
  return mEndEffectorBodyNode.get();
}

//==============================================================================
dart::dynamics::BodyNode* AdaHand::getHandBaseBodyNode() const
{
  return mEndEffectorBodyNode.get();
}

//==============================================================================
void AdaHand::loadPreshapes(
    const dart::common::Uri& preshapesUri,
    const dart::common::ResourceRetrieverPtr& retriever)
{
  mPreshapeConfigurations
      = parseYAMLToPreshapes(aikido::io::loadYAML(preshapesUri, retriever));
}

//==============================================================================
void AdaHand::loadTSRTransforms(
    const dart::common::Uri& tsrTransformsUri,
    const dart::common::ResourceRetrieverPtr& retriever)
{
  mEndEffectorTransforms = parseYAMLToEndEffectorTransforms(
      aikido::io::loadYAML(tsrTransformsUri, retriever));
}

//==============================================================================
boost::optional<Eigen::VectorXd> AdaHand::getPreshape(
    const std::string& preshapeName)
{
  auto preshape = mPreshapeConfigurations.find(preshapeName);
  if (preshape == mPreshapeConfigurations.end())
    return boost::none;

  return preshape->second;
}

//==============================================================================
boost::optional<Eigen::Isometry3d> AdaHand::getEndEffectorTransform(
    const std::string& objectType) const
{
  auto transform = mEndEffectorTransforms.find(objectType);
  if (transform == mEndEffectorTransforms.end())
    return boost::none;

  return transform->second;
}

//==============================================================================
AdaHand::PreshapeMap AdaHand::parseYAMLToPreshapes(const YAML::Node& node)
{
  PreshapeMap preshapeMap;
  for (const auto preshapeNode : node)
  {
    auto preshapeName = preshapeNode.first.as<std::string>();
    auto jointNodes = preshapeNode.second;

    // TODO: check
    Eigen::VectorXd preshape(2);
    for (auto joint : jointNodes)
    {
      auto jointName = joint.first.as<std::string>();
      auto jointIndex = adaFingerJointNameToPositionIndexMap.find(jointName);
      if (jointIndex == adaFingerJointNameToPositionIndexMap.end())
      {
        std::stringstream message;
        message << "Joint '" << jointName << "' does not exist." << std::endl;
        throw std::runtime_error(message.str());
      }
      preshape[jointIndex->second] = joint.second.as<double>();
    }

    preshapeMap.emplace(preshapeName, preshape);
  }
  return preshapeMap;
}

//==============================================================================
AdaHand::EndEffectorTransformMap AdaHand::parseYAMLToEndEffectorTransforms(
    const YAML::Node& node)
{
  return node[mName].as<EndEffectorTransformMap>();
}

//==============================================================================
std::shared_ptr<aikido::control::PositionCommandExecutor>
AdaHand::createAdaHandPositionExecutor(const dart::dynamics::SkeletonPtr& robot)
{
  using aikido::control::ros::RosPositionCommandExecutor;

  if (mSimulation)
  {
    return std::make_shared<AdaHandKinematicSimulationPositionCommandExecutor>(
        robot, "/" + mName + "/");
  }
  else
  {
    const std::string serverName
        = "/" + mName + "_hand_controller/set_position";

    std::vector<std::string> jointNames(2);
    for (const auto kv : adaFingerJointNameToPositionIndexMap)
    {
      assert(kv.second < jointNames.size());
      jointNames[kv.second] = kv.first;
    }

    return std::make_shared<RosPositionCommandExecutor>(
        *mNode, serverName, jointNames);
  }
}

const std::unordered_map<std::string, size_t>
    AdaHand::adaFingerJointNameToPositionIndexMap
    = {{"j2n6s200_link_finger_1", 0}, {"j2n6s200_link_finger_2", 1}};

} // namespace ada
