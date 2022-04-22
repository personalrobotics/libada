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
#include "aikido/planner/parabolic/ParabolicSmoother.hpp"
#include "aikido/statespace/ScopedState.hpp"
namespace py = pybind11;

using aikido::planner::kunzretimer::KunzRetimer;
using aikido::planner::parabolic::ParabolicSmoother;

//==============================================================================
// NOTE: These functions define the Python API for Ada.

aikido::constraint::TestablePtr get_self_collision_constraint(ada::Ada* self)
{
  return self->getArm()->getSelfCollisionConstraint();
}

bool start_trajectory_controllers(ada::Ada* self)
{
  return self->startTrajectoryControllers();
}

bool stop_trajectory_controllers(ada::Ada* self)
{
  return self->stopTrajectoryControllers();
}

std::string get_name(ada::Ada* self)
{
  return self->getName();
}

aikido::planner::WorldPtr get_world(ada::Ada* self)
{
  return self->getWorld();
}

std::shared_ptr<ada::Ada::AdaHand> get_hand(ada::Ada* self)
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
  auto armSkeleton = self->getArm()->getMetaSkeleton();
  auto armSpace
      = std::make_shared<aikido::statespace::dart::MetaSkeletonStateSpace>(
          armSkeleton.get());
  return armSpace;
}

void set_arm_positions(ada::Ada* self, const Eigen::VectorXd& configuration)
{
  auto arm = self->getArm();
  auto armSkeleton = arm->getMetaSkeleton();
  armSkeleton->setPositions(configuration);
}

Eigen::VectorXd get_arm_positions(ada::Ada* self)
{
  return self->getArm()->getCurrentConfiguration();
}

aikido::constraint::TestablePtr get_world_collision_constraint(
    ada::Ada* self, const std::vector<std::string> bodyNames)
{
  return self->getArm()->getWorldCollisionConstraint(bodyNames);
}

aikido::trajectory::TrajectoryPtr compute_joint_space_path(
    ada::Ada* self,
    const std::vector<std::pair<double, Eigen::VectorXd>>& waypoints)
{
  return self->computeArmJointSpacePath(waypoints);
}

aikido::trajectory::TrajectoryPtr compute_smooth_joint_space_path(
    ada::Ada* self,
    const std::vector<std::pair<double, Eigen::VectorXd>>& waypoints,
    const aikido::constraint::dart::CollisionFreePtr& collisionFreeConstraint
    = nullptr)
{
  auto vLimits = self->getVelocityLimits(true);
  auto aLimits = self->getAccelerationLimits(true);
  ParabolicSmoother postprocessor(vLimits, aLimits, ada::SmoothParams());

  auto traj = self->computeArmJointSpacePath(waypoints);
  if (traj)
  {
    // Cast to interpolated or spline:
    auto interpolated
        = dynamic_cast<const aikido::trajectory::Interpolated*>(traj.get());
    if (interpolated)
    {
      return postprocessor.postprocess(
          *interpolated, *(self->cloneRNG().get()), collisionFreeConstraint);
    }

    auto spline = dynamic_cast<const aikido::trajectory::Spline*>(traj.get());
    if (spline)
    {
      return postprocessor.postprocess(
          *spline, *(self->cloneRNG().get()), collisionFreeConstraint);
    }

    // Else return null
  }
  return traj;
}

aikido::trajectory::TrajectoryPtr compute_retime_path(
    ada::Ada* self, aikido::trajectory::InterpolatedPtr trajectory_ptr)
{
  auto vLimits = self->getVelocityLimits(true);
  auto aLimits = self->getAccelerationLimits(true);
  KunzRetimer postprocessor(vLimits, aLimits, ada::KunzParams());
  return postprocessor.postprocess(
      *(trajectory_ptr),
      *(self->cloneRNG().get()),
      self->getSelfCollisionConstraint());
}

aikido::trajectory::TrajectoryPtr plan_to_configuration(
    ada::Ada* self, const Eigen::VectorXd& configuration)
{
  auto trajectory = self->getArm()->planToConfiguration(configuration);
  return trajectory;
}

aikido::trajectory::TrajectoryPtr plan_to_offset(
    ada::Ada* self,
    const std::string bodyNodeName,
    const Eigen::Vector3d& offset)
{
  auto trajectory = self->getArm()->planToOffset(bodyNodeName, offset);
  return trajectory;
}

aikido::trajectory::TrajectoryPtr plan_to_tsr(
    ada::Ada* self,
    const std::string bodyNodeName,
    const aikido::constraint::dart::TSRPtr& tsr)
{
  auto trajectory = self->getArm()->planToTSR(bodyNodeName, tsr);
  return trajectory;
}

void execute_trajectory(
    ada::Ada* self, const aikido::trajectory::TrajectoryPtr& trajectory)
{
  auto future = self->getArm()->executeTrajectory(trajectory);

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

//==============================================================================
// NOTE: These functions define the Python API for AdaHand.

dart::dynamics::MetaSkeletonPtr hand_get_skeleton(ada::Ada::AdaHand* self)
{
  return self->getMetaSkeleton();
}

aikido::statespace::dart::MetaSkeletonStateSpacePtr hand_get_state_space(
    ada::Ada::AdaHand* self)
{
  auto handSkeleton = self->getMetaSkeleton();
  auto handSpace
      = std::make_shared<aikido::statespace::dart::MetaSkeletonStateSpace>(
          handSkeleton.get());
  return handSpace;
}

void execute_preshape(ada::Ada::AdaHand* self, const Eigen::VectorXd& d)
{
  auto future = self->executePreshape(d);

  if (!future.valid())
  {
    std::__throw_future_error(0);
  }

  future.wait();
  // Throw any exceptions
  future.get();
}

void hand_open(ada::Ada::AdaHand* self)
{
  auto future = self->executePreshape("open");

  if (!future.valid())
  {
    std::__throw_future_error(0);
  }

  future.wait();
  // Throw any exceptions
  future.get();
}

void hand_close(ada::Ada::AdaHand* self)
{
  auto future = self->executePreshape("closed");

  if (!future.valid())
  {
    std::__throw_future_error(0);
  }

  future.wait();
  // Throw any exceptions
  future.get();
}

dart::dynamics::BodyNode* get_end_effector_body_node(ada::Ada::AdaHand* self)
{
  return self->getEndEffectorBodyNode();
}

void grab(ada::Ada::AdaHand* self, dart::dynamics::SkeletonPtr object)
{
  self->grab(object);
}

void ungrab(ada::Ada::AdaHand* self)
{
  self->ungrab();
}

//====================================LIBADA====================================

void Ada(pybind11::module& m)
{
  py::class_<ada::Ada, std::shared_ptr<ada::Ada>>(m, "Ada")
      .def(py::init([](bool simulation) -> std::shared_ptr<ada::Ada> {
        return std::make_shared<ada::Ada>(simulation);
      }))
      .def("get_self_collision_constraint", get_self_collision_constraint)
      .def("start_trajectory_controllers", start_trajectory_controllers)
      .def("stop_trajectory_controllers", stop_trajectory_controllers)
      .def("get_name", get_name)
      .def("get_world", get_world)
      .def("get_hand", get_hand)
      .def("get_skeleton", get_skeleton)
      .def("get_arm_skeleton", get_arm_skeleton)
      .def("get_arm_state_space", get_arm_state_space)
      .def("set_arm_positions", set_arm_positions)
      .def("get_arm_positions", get_arm_positions)
      .def(
          "get_world_collision_constraint",
          get_world_collision_constraint,
          "Get collision constraint with requested bodies in world (or all "
          "bodies if left blank)",
          py::arg("bodyNames") = std::vector<std::string>())
      .def("compute_joint_space_path", compute_joint_space_path)
      .def("compute_smooth_joint_space_path", compute_smooth_joint_space_path)
      .def("compute_retime_path", compute_retime_path)
      .def("plan_to_configuration", plan_to_configuration)
      .def("plan_to_offset", plan_to_offset)
      .def("plan_to_tsr", plan_to_tsr)
      .def("execute_trajectory", execute_trajectory)
      .def("start_viewer", start_viewer);

  py::class_<ada::Ada::AdaHand, std::shared_ptr<ada::Ada::AdaHand>>(
      m, "AdaHand")
      .def("get_skeleton", hand_get_skeleton)
      .def("get_state_space", hand_get_state_space)
      .def("execute_preshape", execute_preshape)
      .def("open", hand_open)
      .def("close", hand_close)
      .def("get_end_effector_body_node", get_end_effector_body_node)
      .def("grab", grab)
      .def("ungrab", ungrab);
}
