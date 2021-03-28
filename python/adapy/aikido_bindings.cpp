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

void Aikido(pybind11::module& m)
{
  //====================================AIKIDO=============================================================================
  py::class_<aikido::planner::World, std::shared_ptr<aikido::planner::World>>(
      m, "World")
      .def(
          "add_body_from_urdf",
          [](aikido::planner::World* self,
             const std::string& uri,
             std::vector<double> objectPose)
              -> std::shared_ptr<::dart::dynamics::Skeleton> {
            auto transform = vectorToIsometry(objectPose);

            const auto skeleton = aikido::io::loadSkeletonFromURDF(
                std::make_shared<aikido::io::CatkinResourceRetriever>(), uri);

            if (!skeleton)
              throw std::runtime_error("unable to load '" + uri + "'");

            dynamic_cast<dart::dynamics::FreeJoint*>(skeleton->getJoint(0))
                ->setTransform(transform);

            self->addSkeleton(skeleton);
            return skeleton;
          })
      .def(
          "add_body_from_urdf_matrix",
          [](aikido::planner::World* self,
             const std::string& uri,
             Eigen::Matrix4d& objectPose)
              -> std::shared_ptr<::dart::dynamics::Skeleton> {
            auto transform = matrixToIsometry(objectPose);

            const auto skeleton = aikido::io::loadSkeletonFromURDF(
                std::make_shared<aikido::io::CatkinResourceRetriever>(), uri);

            if (!skeleton)
              throw std::runtime_error("unable to load '" + uri + "'");

            dynamic_cast<dart::dynamics::FreeJoint*>(skeleton->getJoint(0))
                ->setTransform(transform);

            self->addSkeleton(skeleton);
            return skeleton;
          })
      .def(
          "remove_skeleton",
          [](aikido::planner::World* self,
             std::shared_ptr<::dart::dynamics::Skeleton> skeleton) -> void {
            self->removeSkeleton(skeleton);
          })
      .def(
          "get_skeleton",
          [](aikido::planner::World* self, int i)
              -> dart::dynamics::SkeletonPtr { return self->getSkeleton(i); });
  py::class_<
      aikido::rviz::InteractiveMarkerViewer,
      std::shared_ptr<aikido::rviz::InteractiveMarkerViewer>>(
      m, "InteractiveMarkerViewer")
      .def(
          "update",
          [](aikido::rviz::InteractiveMarkerViewer* self) -> void {
            self->update();
          })
      .def(
          "add_frame",
          [](aikido::rviz::InteractiveMarkerViewer* self,
             dart::dynamics::BodyNode* node) -> void {
            self->addFrameMarker(node);
          })
      .def(
          "add_tsr_marker",
          [](aikido::rviz::InteractiveMarkerViewer* self,
             std::shared_ptr<aikido::constraint::dart::TSR> tsr)
              -> aikido::rviz::TSRMarkerPtr {
            return self->addTSRMarker(*tsr.get());
          });
  py::class_<
      aikido::constraint::dart::CollisionFree,
      std::shared_ptr<aikido::constraint::dart::CollisionFree>>(
      m, "CollisionFree");
  py::class_<
      aikido::statespace::dart::MetaSkeletonStateSpace,
      aikido::statespace::dart::MetaSkeletonStateSpacePtr>(
      m, "MetaSkeletonStateSpace");
  py::class_<aikido::trajectory::Trajectory, aikido::trajectory::TrajectoryPtr>(
      m, "Trajectory");
  py::class_<aikido::rviz::TSRMarker, aikido::rviz::TSRMarkerPtr>(
      m, "TSRMarker");
  py::class_<aikido::constraint::Testable, aikido::constraint::TestablePtr>(
      m, "FullCollisionFree")
      .def(
          "is_satisfied",
          [](aikido::constraint::Testable* self,
             const aikido::statespace::dart::MetaSkeletonStateSpacePtr&
                 armSpace,
             const dart::dynamics::MetaSkeletonPtr& armSkeleton,
             const Eigen::VectorXd& positions) -> bool {
            auto armState = armSpace->createState();
            armSpace->convertPositionsToState(positions, armState);
            auto currentState
                = armSpace->getScopedStateFromMetaSkeleton(armSkeleton.get());
            aikido::constraint::DefaultTestableOutcome
                fullCollisionCheckOutcome;
            bool collisionResult
                = self->isSatisfied(armState, &fullCollisionCheckOutcome);
            armSpace->setState(armSkeleton.get(), currentState);
            return collisionResult;
          });
}