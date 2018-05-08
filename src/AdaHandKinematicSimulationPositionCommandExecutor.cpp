#include "libada/AdaHandKinematicSimulationPositionCommandExecutor.hpp"

#include <dart/collision/fcl/FCLCollisionDetector.hpp>

namespace ada {

constexpr std::chrono::milliseconds
    AdaHandKinematicSimulationPositionCommandExecutor::kWaitPeriod;
constexpr int
    AdaHandKinematicSimulationPositionCommandExecutor::kNumPositionExecutors;
constexpr std::array<std::size_t,
                     AdaHandKinematicSimulationPositionCommandExecutor::
                         kNumPositionExecutors>
    AdaHandKinematicSimulationPositionCommandExecutor::kPrimalDofs;
constexpr std::array<std::size_t,
                     AdaHandKinematicSimulationPositionCommandExecutor::
                         kNumPositionExecutors>
    AdaHandKinematicSimulationPositionCommandExecutor::kDistalDofs;

//==============================================================================
AdaHandKinematicSimulationPositionCommandExecutor::
    AdaHandKinematicSimulationPositionCommandExecutor(
        dart::dynamics::SkeletonPtr robot,
        const std::string& prefix,
        ::dart::collision::CollisionDetectorPtr collisionDetector,
        ::dart::collision::CollisionGroupPtr collideWith,
        ::dart::collision::CollisionOption collisionOptions)
  : mCollisionDetector(std::move(collisionDetector))
  , mCollideWith(std::move(collideWith))
  , mCollisionOptions(std::move(collisionOptions))
  , mInProgress(false)
{
  if (!robot)
    throw std::invalid_argument("Robot is null");

  if (mCollisionDetector && mCollideWith)
  {
    // If a collision group is given and its collision detector does not match
    // mCollisionDetector, set the collision group to an empty collision group.
    if (mCollisionDetector != mCollideWith->getCollisionDetector())
    {
      std::cerr << "[AdaHandKinematicSimulationPositionCommandExecutor] "
                << "CollisionDetector of type " << mCollisionDetector->getType()
                << " does not match CollisionGroup's CollisionDetector of type "
                << mCollideWith->getCollisionDetector()->getType() << std::endl;

      ::dart::collision::CollisionGroupPtr newCollideWith
          = mCollisionDetector->createCollisionGroup();
      for (auto i = 0u; i < mCollideWith->getNumShapeFrames(); ++i)
        newCollideWith->addShapeFrame(mCollideWith->getShapeFrame(i));
      mCollideWith = std::move(newCollideWith);
    }
  }
  else if (mCollisionDetector && !mCollideWith)
  {
    mCollideWith = mCollisionDetector->createCollisionGroup();
  }
  else if (!mCollisionDetector && mCollideWith)
  {
    mCollisionDetector = mCollideWith->getCollisionDetector();
  }
  else
  {
    // Default mCollisionDetector to FCL collision detector and mCollideWith to
    // empty collision group.
    mCollisionDetector = dart::collision::FCLCollisionDetector::create();
    mCollideWith = mCollisionDetector->createCollisionGroup();
  }

  setupExecutors(std::move(robot), prefix);
}

//==============================================================================
void AdaHandKinematicSimulationPositionCommandExecutor::setupExecutors(
    dart::dynamics::SkeletonPtr robot, const std::string& /*prefix*/)
{
  using dart::dynamics::Chain;
  using dart::dynamics::ChainPtr;
  using FingerPositionCommandExecutor
      = AdaFingerKinematicSimulationPositionCommandExecutor;

  const auto fingerChains = std::array<ChainPtr, kNumPositionExecutors>{
      {Chain::create(
           robot->getBodyNode("j2n6s200_link_finger_1"),     // finger0Primal
           robot->getBodyNode("j2n6s200_link_finger_tip_1"), // finger0Distal
           Chain::IncludeBoth),
       Chain::create(
           robot->getBodyNode("j2n6s200_link_finger_2"),     // finger1Primal
           robot->getBodyNode("j2n6s200_link_finger_tip_2"), // finger1Distal
           Chain::IncludeBoth)}};

  for (std::size_t i = 0; i < fingerChains.size(); ++i)
  {
    mPositionCommandExecutors[i]
        = std::make_shared<FingerPositionCommandExecutor>(
            fingerChains[i],
            kPrimalDofs[i],
            kDistalDofs[i],
            mCollisionDetector,
            mCollideWith,
            mCollisionOptions);
  }
}

//==============================================================================
std::future<void> AdaHandKinematicSimulationPositionCommandExecutor::execute(
    const Eigen::VectorXd& goalPositions)
{
  std::lock_guard<std::mutex> lock(mMutex);

  if (mInProgress)
    throw std::runtime_error("Another position command is in progress.");

  if (goalPositions.size() != 2)
  {
    std::stringstream message;
    message << "AdaHand goal positions must have 2 elements, but ["
            << goalPositions.size() << "] given.";
    throw std::invalid_argument(message.str());
  }

  mPromise.reset(new std::promise<void>());

  const Eigen::Vector2d proximalGoalPositions = goalPositions.head<2>();
  mFingerFutures.clear();

  mFingerFutures.reserve(kNumPositionExecutors);
  for (std::size_t i = 0; i < kNumPositionExecutors; ++i)
    mFingerFutures.emplace_back(
        mPositionCommandExecutors[i]->execute(proximalGoalPositions.row(i)));

  mInProgress = true;

  return mPromise->get_future();
}

//==============================================================================
void AdaHandKinematicSimulationPositionCommandExecutor::step(
    const std::chrono::system_clock::time_point& timepoint)
{
  std::lock_guard<std::mutex> lock(mMutex);

  if (!mInProgress)
    return;

  // Check whether fingers have completed.
  std::vector<std::future_status> statuses;
  std::exception_ptr expr;
  bool allFingersCompleted = true;

  for (std::size_t i = 0; i < mFingerFutures.size(); ++i)
  {
    // Check the status of each finger
    auto status = mFingerFutures[i].wait_for(kWaitPeriod);
    if (status != std::future_status::ready)
    {
      allFingersCompleted = false;
      break;
    }
  }

  if (allFingersCompleted)
  {
    for (std::size_t i = 0; i < mFingerFutures.size(); ++i)
    {
      try
      {
        mFingerFutures[i].get();
      }
      catch (...)
      {
        expr = std::current_exception();
        break;
      }
    }
  }

  // Termination condition
  if (expr || allFingersCompleted)
  {
    if (expr)
      mPromise->set_exception(expr);
    else if (allFingersCompleted)
      mPromise->set_value();

    mInProgress = false;
    return;
  }

  // Call the finger executors' step function.
  for (int i = 0; i < kNumPositionExecutors; ++i)
    mPositionCommandExecutors[i]->step(timepoint);
}

//==============================================================================
bool AdaHandKinematicSimulationPositionCommandExecutor::setCollideWith(
    ::dart::collision::CollisionGroupPtr collideWith)
{
  std::lock_guard<std::mutex> lock(mMutex);

  if (mInProgress)
    return false;

  mCollideWith = std::move(collideWith);
  mCollisionDetector = mCollideWith->getCollisionDetector();

  for (std::size_t i = 0; i < kNumPositionExecutors; ++i)
    mPositionCommandExecutors[i]->setCollideWith(mCollideWith);

  return true;
}

} // ada
