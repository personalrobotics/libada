#include "libada/AdaHand.hpp"

#include <chrono>

#include <aikido/constraint/Satisfied.hpp>
#include <aikido/robot/util.hpp>

#undef dtwarn
#define dtwarn (::dart::common::colorErr("Warning", __FILE__, __LINE__, 33))

#undef dtinfo
#define dtinfo (::dart::common::colorMsg("Info", 32))

namespace ada {

//==============================================================================
Ada::AdaHand::AdaHand(
    Ada* ada,
    dart::dynamics::BodyNodePtr handBaseBodyNode,
    dart::dynamics::BodyNodePtr endEffectorBodyNode)
  : mAda(ada)
  , mHandBaseBodyNode(handBaseBodyNode)
  , mEndEffectorBodyNode(endEffectorBodyNode)
  , mGrabMetadata(nullptr)
{
}

//==============================================================================
dart::dynamics::ConstMetaSkeletonPtr Ada::AdaHand::getMetaSkeleton() const
{
  return mAda->mHandRobot->getMetaSkeleton();
}

//==============================================================================
dart::dynamics::MetaSkeletonPtr Ada::AdaHand::getMetaSkeleton()
{
  return mAda->mHandRobot->getMetaSkeleton();
}

//==============================================================================
void Ada::AdaHand::grab(const dart::dynamics::SkeletonPtr& bodyToGrab)
{
  using dart::dynamics::FreeJoint;
  using dart::dynamics::Joint;
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

  Joint* joint = bodyToGrab->getRootJoint();
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
  bodyNode->moveTo<WeldJoint>(
      mAda->getRootSkeleton(), mEndEffectorBodyNode, weldJointProperties);

  // Moving the grabbed object into the same skeleton as the hand means that it
  // will be considered during self-collision checking. Therefore, we need to
  // disable self-collision checking between grabbed object and hand.
  std::vector<std::pair<std::string, std::string>> vec{
      std::make_pair(mEndEffectorBodyNode->getName(), bodyNodeName)};
  mAda->ignoreSelfCollisionPairs(vec);

  mGrabMetadata = std::make_unique<aikido::robot::GrabMetadata>(
      bodyNode, bodyNodeName, bodyToGrab, jointProperties);
}

//==============================================================================
void Ada::AdaHand::ungrab()
{
  using dart::dynamics::FreeJoint;
  using dart::dynamics::Joint;

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
  std::vector<std::pair<std::string, std::string>> vec{std::make_pair(
      mEndEffectorBodyNode->getName(), grabbedBodyNode->getName())};
  mAda->enforceSelfCollisionPairs(vec);

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
    dtwarn << "[Hand::ungrab] Released object was renamed from \"" << oldName
           << "\" to \"" << newName << "\"" << std::endl;
  }

  mGrabMetadata.reset();
}

//==============================================================================
std::future<void> Ada::AdaHand::executePreshape(const std::string& preshapeName)
{
  auto config = mAda->mHandRobot->getNamedConfiguration(preshapeName);

  if (config.size() == 0)
  {
    // Return excepted future
    std::promise<void> promise;
    promise.set_exception(std::make_exception_ptr(
        std::runtime_error("No 'open' configuration provided.")));
    return promise.get_future();
  }

  return executePreshape(config);
}

//==============================================================================
std::future<void> Ada::AdaHand::executePreshape(const Eigen::VectorXd& preshape)
{
  auto traj = mAda->mHandRobot->planToConfiguration(preshape);
  return mAda->mHandRobot->executeTrajectory(traj);
}

//==============================================================================
void Ada::AdaHand::step(
    const std::chrono::system_clock::time_point& /*timepoint */)
{
  // Do Nothing, Ada handles step()
}

//==============================================================================
dart::dynamics::BodyNode* Ada::AdaHand::getEndEffectorBodyNode() const
{
  return mEndEffectorBodyNode.get();
}

//==============================================================================
dart::dynamics::BodyNode* Ada::AdaHand::getHandBaseBodyNode() const
{
  return mHandBaseBodyNode.get();
}

} // namespace ada
