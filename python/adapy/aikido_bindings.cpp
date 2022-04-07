#include <aikido/constraint/Satisfied.hpp>
#include <aikido/constraint/dart/CollisionFree.hpp>
#include <aikido/io/CatkinResourceRetriever.hpp>
#include <aikido/io/util.hpp>
#include <aikido/planner/World.hpp>
#include <aikido/rviz/InteractiveMarkerViewer.hpp>
#include <dart/collision/CollisionDetector.hpp>
#include <dart/collision/CollisionGroup.hpp>
#include <dart/dart.hpp>
#include <pybind11/cast.h>
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "aikido/statespace/ScopedState.hpp"
#include "aikido/statespace/StateHandle.hpp"
#include "aikido/statespace/StateSpace.hpp"

namespace py = pybind11;

Eigen::Isometry3d vectorToIsometry(std::vector<double>& poseVector)
{
  double* ptr = &poseVector[0];
  Eigen::Map<Eigen::VectorXd> p(ptr, 7);

  Eigen::Isometry3d pose(Eigen::Isometry3d::Identity());
  pose.translation() = p.head(3);
  Eigen::Quaterniond q(p[3], p[4], p[5], p[6]);
  pose.linear() = Eigen::Matrix3d(q);
  return pose;
}

Eigen::Isometry3d matrixToIsometry(Eigen::Matrix4d& poseMatrix)
{
  Eigen::Isometry3d pose(Eigen::Isometry3d::Identity());
  pose.translation() = poseMatrix.block<3, 1>(0, 3);
  pose.linear() = poseMatrix.block<3, 3>(0, 0);
  return pose;
}

//==============================================================================
// NOTE: These functions define the Python API for World.

std::shared_ptr<::dart::dynamics::Skeleton> add_body_from_urdf(
    aikido::planner::World* self,
    const std::string& uri,
    std::vector<double> objectPose)
{
  auto transform = vectorToIsometry(objectPose);

  const auto skeleton = aikido::io::loadSkeletonFromURDF(
      std::make_shared<aikido::io::CatkinResourceRetriever>(), uri);

  if (!skeleton)
    throw std::runtime_error("unable to load '" + uri + "'");

  dynamic_cast<dart::dynamics::FreeJoint*>(skeleton->getJoint(0))
      ->setTransform(transform);

  self->addSkeleton(skeleton);
  return skeleton;
}

std::shared_ptr<::dart::dynamics::Skeleton> add_body_from_urdf_matrix(
    aikido::planner::World* self,
    const std::string& uri,
    Eigen::Matrix4d& objectPose)
{
  auto transform = matrixToIsometry(objectPose);

  const auto skeleton = aikido::io::loadSkeletonFromURDF(
      std::make_shared<aikido::io::CatkinResourceRetriever>(), uri);

  if (!skeleton)
    throw std::runtime_error("unable to load '" + uri + "'");

  dynamic_cast<dart::dynamics::FreeJoint*>(skeleton->getJoint(0))
      ->setTransform(transform);

  self->addSkeleton(skeleton);
  return skeleton;
}

void remove_skeleton(
    aikido::planner::World* self,
    std::shared_ptr<::dart::dynamics::Skeleton> skeleton)
{
  self->removeSkeleton(skeleton);
}

dart::dynamics::SkeletonPtr get_skeleton(aikido::planner::World* self, int i)
{
  return self->getSkeleton(i);
}

//==============================================================================
// NOTE: These functions define the Python API for InteractiveMarkerViewer.

void update(aikido::rviz::InteractiveMarkerViewer* self)
{
  self->update();
}

void add_frame(
    aikido::rviz::InteractiveMarkerViewer* self,
    std::shared_ptr<dart::dynamics::BodyNode> node)
{
  self->addFrameMarker(node);
}

aikido::rviz::TSRMarkerPtr add_tsr_marker(
    aikido::rviz::InteractiveMarkerViewer* self,
    std::shared_ptr<aikido::constraint::dart::TSR> tsr)
{
  return self->addTSRMarker(*tsr.get());
}

//==============================================================================
// NOTE: These functions define the Python API for Testable.

bool is_satisfied(
    aikido::constraint::Testable* self, const Eigen::VectorXd& positions)
{
  auto armSpace = self->getStateSpace();
  auto testState = armSpace->createState();
  armSpace->expMap(positions, testState);
  return self->isSatisfied(testState);
}

//====================================AIKIDO====================================

void Aikido(pybind11::module& m)
{
  py::class_<aikido::planner::World, std::shared_ptr<aikido::planner::World>>(
      m, "World")
      .def("add_body_from_urdf", add_body_from_urdf)
      .def("add_body_from_urdf_matrix", add_body_from_urdf_matrix)
      .def("remove_skeleton", remove_skeleton)
      .def("get_skeleton", get_skeleton);

  py::class_<
      aikido::rviz::InteractiveMarkerViewer,
      std::shared_ptr<aikido::rviz::InteractiveMarkerViewer>>(
      m, "InteractiveMarkerViewer")
      .def("update", update)
      .def("add_frame", add_frame)
      .def("add_tsr_marker", add_tsr_marker);

  py::class_<
      aikido::statespace::dart::MetaSkeletonStateSpace,
      aikido::statespace::dart::MetaSkeletonStateSpacePtr>(
      m, "MetaSkeletonStateSpace");

  py::class_<aikido::trajectory::Trajectory, aikido::trajectory::TrajectoryPtr>(
      m, "Trajectory");

  py::class_<aikido::rviz::TSRMarker, aikido::rviz::TSRMarkerPtr>(
      m, "TSRMarker");

  py::class_<aikido::constraint::Testable, aikido::constraint::TestablePtr>(
      m, "Testable")
      .def("is_satisfied", is_satisfied);
}
