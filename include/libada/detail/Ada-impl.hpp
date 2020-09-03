
namespace ada {

//==============================================================================
template <typename PostProcessor>
aikido::trajectory::UniqueSplinePtr Ada::postProcessPath(
    const aikido::trajectory::Trajectory* path,
    const aikido::constraint::TestablePtr& constraint,
    const typename PostProcessor::Params& postProcessorParams,
    const Eigen::VectorXd& velocityLimits,
    const Eigen::VectorXd& accelerationLimits)
{
  auto sentVelocityLimits = (velocityLimits.squaredNorm() == 0.0)
                                ? getVelocityLimits()
                                : velocityLimits;
  auto sentAccelerationLimits = (accelerationLimits.squaredNorm() == 0.0)
                                    ? getAccelerationLimits()
                                    : accelerationLimits;

  if(sentAccelerationLimits.size() != 6) {
    sentAccelerationLimits.resize(6);
    auto defaultAccelerationLimits = getAccelerationLimits();
    sentAccelerationLimits << defaultAccelerationLimits(0), defaultAccelerationLimits(1), defaultAccelerationLimits(2),
                              defaultAccelerationLimits(3), defaultAccelerationLimits(4), defaultAccelerationLimits(5);
  }

  return mRobot->postProcessPath<PostProcessor>(
      sentVelocityLimits,
      sentAccelerationLimits,
      path,
      constraint,
      postProcessorParams);
}

//==============================================================================
template <typename PostProcessor>
bool Ada::moveArmToTSR(
    const aikido::constraint::dart::TSR& tsr,
    const aikido::constraint::dart::CollisionFreePtr& collisionFree,
    double timelimit,
    size_t maxNumTrials,
    const aikido::distance::ConfigurationRankerPtr& ranker,
    const Eigen::Vector6d& velocityLimits,
    const typename PostProcessor::Params& params)
{
  auto trajectory
      = planArmToTSR(tsr, collisionFree, timelimit, maxNumTrials, ranker);

  if (!trajectory)
  {
    dtwarn << "Failed to plan to TSR" << std::endl;
    return false;
  }

  return moveArmOnTrajectory<PostProcessor>(
      trajectory, collisionFree, velocityLimits, params);
}

//==============================================================================
template <typename PostProcessor>
bool Ada::moveArmOnTrajectory(
    aikido::trajectory::TrajectoryPtr trajectory,
    const aikido::constraint::dart::CollisionFreePtr& collisionFree,
    const Eigen::Vector6d& velocityLimits,
    const typename PostProcessor::Params& params)
{
  if (!trajectory)
    return false;

  std::vector<aikido::constraint::ConstTestablePtr> constraints;

  if (collisionFree)
  {
    constraints.push_back(collisionFree);
  }
  auto testable = std::make_shared<aikido::constraint::TestableIntersection>(
      mArmSpace, constraints);

  aikido::trajectory::TrajectoryPtr timedTrajectory;

  auto armSkeleton = mArm->getMetaSkeleton();

  timedTrajectory = postProcessPath<PostProcessor>(
      trajectory.get(), testable, params, velocityLimits);

  // TODO: Have this die more gracefully
  if (!timedTrajectory)
  {
    throw std::runtime_error("Retiming failed");
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

} // namespace ada
