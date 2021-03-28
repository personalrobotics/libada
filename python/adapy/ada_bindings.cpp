#include <aikido/rviz/InteractiveMarkerViewer.hpp>
#include <dart/dynamics/JacobianNode.hpp>
#include <pybind11/cast.h>
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <aikido/planner/World.hpp>
#include "libada/Ada.hpp"
#include "aikido/constraint/Satisfied.hpp"
#include "aikido/planner/kunzretimer/KunzRetimer.hpp"
#include "aikido/statespace/ScopedState.hpp"
namespace py = pybind11;

using aikido::planner::kunzretimer::KunzRetimer;

// NOTE: These functions define the Python API for Ada.

std::shared_ptr<aikido::constraint::dart::CollisionFree>
get_self_collision_constraint(ada::Ada* self)
{
  return self->getSelfCollisionConstraint(
      self->getStateSpace(), self->getMetaSkeleton());
}

void start_trajectory_executor(ada::Ada* self)
{
  self->startTrajectoryExecutor();
}

void stop_trajectory_executor(ada::Ada* self)
{
  self->stopTrajectoryExecutor();
}

std::string get_name(ada::Ada* self)
{
  return self->getName();
}

aikido::planner::WorldPtr get_world(ada::Ada* self)
{
  return self->getWorld();
}

ada::AdaHandPtr get_hand(ada::Ada* self)
{
  return self->getHand();
}

dart::dynamics::MetaSkeletonPtr get_skeleton(ada::Ada* self)
{
  return self->getMetaSkeleton();
}

dart::dynamics::MetaSkeletonPtr get_arm_skeleton(ada::Ada* self)
{
  return self->getArm()->getMetaSkeleton();
}

aikido::statespace::dart::MetaSkeletonStateSpacePtr get_arm_state_space(
    ada::Ada* self)
{
  return self->getArm()->getStateSpace();
}

void set_positions(ada::Ada* self, const Eigen::VectorXd& configuration)
{
  auto arm = self->getArm();
  auto armSkeleton = arm->getMetaSkeleton();
  armSkeleton->setPositions(configuration);
}

std::shared_ptr<aikido::constraint::dart::CollisionFree>
set_up_collision_detection(
    ada::Ada* self,
    const aikido::statespace::dart::MetaSkeletonStateSpacePtr& armSpace,
    const dart::dynamics::MetaSkeletonPtr& armSkeleton,
    const std::vector<dart::dynamics::SkeletonPtr>& envSkeletons)
{
  dart::collision::FCLCollisionDetectorPtr collisionDetector
      = dart::collision::FCLCollisionDetector::create();
  std::shared_ptr<dart::collision::CollisionGroup> armCollisionGroup
      = collisionDetector->createCollisionGroup(self->getMetaSkeleton().get());
  std::shared_ptr<dart::collision::CollisionGroup> envCollisionGroup
      = collisionDetector->createCollisionGroup();
  for (const auto& envSkeleton : envSkeletons)
  {
    envCollisionGroup->addShapeFramesOf(envSkeleton.get());
  }
  std::shared_ptr<aikido::constraint::dart::CollisionFree>
      collisionFreeConstraint
      = std::make_shared<aikido::constraint::dart::CollisionFree>(
          armSpace, armSkeleton, collisionDetector);
  collisionFreeConstraint->addPairwiseCheck(
      armCollisionGroup, envCollisionGroup);
  return collisionFreeConstraint;
}

aikido::constraint::TestablePtr get_full_collision_constraint(
    ada::Ada* self,
    const aikido::statespace::dart::MetaSkeletonStateSpacePtr& armSpace,
    const dart::dynamics::MetaSkeletonPtr& armSkeleton,
    const aikido::constraint::dart::CollisionFreePtr& collisionFreeConstraint)
{
  return self->getFullCollisionConstraint(
      armSpace, armSkeleton, collisionFreeConstraint);
}

aikido::trajectory::TrajectoryPtr compute_joint_space_path(
    ada::Ada* self,
    const aikido::statespace::dart::MetaSkeletonStateSpacePtr& stateSpace,
    const std::vector<std::pair<double, Eigen::VectorXd>>& waypoints)
{
  return self->computeRetimedJointSpacePath(stateSpace, waypoints);
}

aikido::trajectory::TrajectoryPtr compute_smooth_joint_space_path(
    ada::Ada* self,
    const aikido::statespace::dart::MetaSkeletonStateSpacePtr& stateSpace,
    const std::vector<std::pair<double, Eigen::VectorXd>>& waypoints,
    const aikido::constraint::dart::CollisionFreePtr& collisionFreeConstraint
    = nullptr)
{
  return self->computeSmoothJointSpacePath(
      stateSpace, waypoints, collisionFreeConstraint);
}

aikido::trajectory::TrajectoryPtr compute_retime_path(
    ada::Ada* self,
    const dart::dynamics::MetaSkeletonPtr& armSkeleton,
    aikido::trajectory::InterpolatedPtr trajectory_ptr)
{
  auto satisfied = std::make_shared<aikido::constraint::Satisfied>(
      trajectory_ptr->getStateSpace());
  return self->postProcessPath<KunzRetimer>(
      trajectory_ptr.get(),
      satisfied,
      ada::KunzParams(),
      armSkeleton->getVelocityUpperLimits(),
      armSkeleton->getAccelerationUpperLimits());
}

aikido::trajectory::TrajectoryPtr plan_to_configuration(
    ada::Ada* self,
    const aikido::statespace::dart::MetaSkeletonStateSpacePtr& armSpace,
    const dart::dynamics::MetaSkeletonPtr& armSkeleton,
    const Eigen::VectorXd& configuration)
{
  auto state = armSpace->createState();
  armSpace->convertPositionsToState(configuration, state);
  auto trajectory
      = self->planToConfiguration(armSpace, armSkeleton, state, nullptr, 10);
  return trajectory;
}

void execute_trajectory(
    ada::Ada* self, const aikido::trajectory::TrajectoryPtr& trajectory)
{
  auto future = self->executeTrajectory(trajectory);

  if (!future.valid())
  {
    std::__throw_future_error(0);
  }
  future.wait();
  // Throw any exceptions
  future.get();
}

std::shared_ptr<aikido::rviz::InteractiveMarkerViewer> start_viewer(
    ada::Ada* self,
    const std::string& topicName,
    const std::string& baseFrameName)
{
  auto viewer = std::make_shared<aikido::rviz::InteractiveMarkerViewer>(
      topicName, baseFrameName, self->getWorld());
  viewer->setAutoUpdate(true);
  return viewer;
}

//====================================LIBADA====================================

void Ada(pybind11::module& m)
{
  py::class_<ada::Ada, std::shared_ptr<ada::Ada>>(m, "Ada")
      .def(py::init([](bool simulation) -> std::shared_ptr<ada::Ada> {
        return std::make_shared<ada::Ada>(
            aikido::planner::World::create(), simulation);
      }))
      .def("get_self_collision_constraint", get_self_collision_constraint)
      .def("start_trajectory_executor", start_trajectory_executor)
      .def("stop_trajectory_executor", stop_trajectory_executor)
      .def("get_name", get_name)
      .def("get_world", get_world)
      .def("get_hand", get_hand)
      .def("get_skeleton", get_skeleton)
      .def("get_arm_skeleton", get_arm_skeleton)
      .def("get_arm_state_space", get_arm_state_space)
      .def("set_positions", set_positions)
      .def("set_up_collision_detection", set_up_collision_detection)
      .def("get_full_collision_constraint", get_full_collision_constraint)
      .def("compute_joint_space_path", compute_joint_space_path)
      .def("compute_smooth_joint_space_path", compute_smooth_joint_space_path)
      .def("compute_retime_path", compute_retime_path)
      .def("plan_to_configuration", plan_to_configuration)
      .def("execute_trajectory", execute_trajectory)
      .def("start_viewer", start_viewer);
  py::class_<ada::AdaHand, std::shared_ptr<ada::AdaHand>>(m, "AdaHand")
      .def(
          "get_skeleton",
          [](ada::AdaHand* self) -> dart::dynamics::MetaSkeletonPtr {
            return self->getMetaSkeleton();
          })
      .def(
          "get_state_space",
          [](ada::AdaHand* self)
              -> aikido::statespace::dart::MetaSkeletonStateSpacePtr {
            auto handSkeleton = self->getMetaSkeleton();
            auto handSpace = std::make_shared<
                aikido::statespace::dart::MetaSkeletonStateSpace>(
                handSkeleton.get());
            return handSpace;
          })
      .def(
          "execute_preshape",
          [](ada::AdaHand* self, const Eigen::Vector2d& d) -> void {
            auto future = self->executePreshape(d);

            if (!future.valid())
            {
              std::__throw_future_error(0);
            }

            future.wait();
            // Throw any exceptions
            future.get();
          })
      .def(
          "open",
          [](ada::AdaHand* self) -> void {
            auto future = self->executePreshape("open");

            if (!future.valid())
            {
              std::__throw_future_error(0);
            }

            future.wait();
            // Throw any exceptions
            future.get();
          })
      .def(
          "close",
          [](ada::AdaHand* self) -> void {
            auto future = self->executePreshape("closed");

            if (!future.valid())
            {
              std::__throw_future_error(0);
            }

            future.wait();
            // Throw any exceptions
            future.get();
          })
      .def(
          "get_endeffector_transform",
          [](ada::AdaHand* self,
             const std::string& objectType) -> Eigen::Matrix4d {
            return self->getEndEffectorTransform(objectType)->matrix();
          })
      .def(
          "get_endeffector_body_node",
          [](ada::AdaHand* self) -> dart::dynamics::BodyNode* {
            return self->getEndEffectorBodyNode();
          })
      .def(
          "grab",
          [](ada::AdaHand* self, dart::dynamics::SkeletonPtr object) -> void {
            self->grab(object);
          });
}
