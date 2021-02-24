#include <aikido/rviz/WorldInteractiveMarkerViewer.hpp>
#include <dart/dynamics/JacobianNode.hpp>
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/cast.h>
#include <pybind11/stl.h>

#include "libada/Ada.hpp"
#include "aikido/statespace/ScopedState.hpp"
#include <aikido/planner/World.hpp>
namespace py = pybind11;

void Ada(pybind11::module& m) {
  //====================================LIBADA==========================================================
  py::class_<ada::Ada, std::shared_ptr<ada::Ada>>(m, "Ada")
      .def(py::init([](bool simulation) -> std::shared_ptr<ada::Ada> {
        return std::make_shared<ada::Ada>(
            aikido::planner::World::create(), simulation);
      }))
      .def("get_self_collision_constraint",
           [](ada::Ada *self) -> std::shared_ptr<aikido::constraint::dart::CollisionFree> {
             return self->getSelfCollisionConstraint(
                 self->getStateSpace(),
                 self->getMetaSkeleton()
             );
           })
      .def("start_trajectory_executor", [](ada::Ada *self) -> void {
        self->startTrajectoryExecutor();
      })
      .def("get_name",
           [](ada::Ada *self) -> std::string { return self->getName(); })
      .def("get_world",
           [](ada::Ada *self) -> aikido::planner::WorldPtr {
             return self->getWorld();
           })
      .def("get_hand",
           [](ada::Ada *self) -> ada::AdaHandPtr {
        return self->getHand();
      })
      .def("get_skeleton", [](ada::Ada *self) -> dart::dynamics::MetaSkeletonPtr {
        return self->getMetaSkeleton();
      })
      .def("get_arm_skeleton", [](ada::Ada *self) -> dart::dynamics::MetaSkeletonPtr {
        return self->getArm()->getMetaSkeleton();
      })
      .def("get_arm_state_space", [](ada::Ada *self) -> aikido::statespace::dart::MetaSkeletonStateSpacePtr {
        return self->getArm()->getStateSpace();
      })
      .def("set_positions",[](ada::Ada *self,const Eigen::VectorXd& configuration) -> void {
           auto arm = self->getArm();
           auto armSkeleton = arm->getMetaSkeleton();
           armSkeleton->setPositions(configuration);
      })
      .def("set_up_collision_detection",
           [](ada::Ada *self,
              const aikido::statespace::dart::MetaSkeletonStateSpacePtr &armSpace,
              const dart::dynamics::MetaSkeletonPtr& armSkeleton,
              const std::vector<dart::dynamics::SkeletonPtr>& envSkeletons) -> std::shared_ptr<aikido::constraint::dart::CollisionFree> {
             dart::collision::FCLCollisionDetectorPtr collisionDetector = dart::collision::FCLCollisionDetector::create();
             std::shared_ptr<dart::collision::CollisionGroup> armCollisionGroup =
                 collisionDetector->createCollisionGroup(self->getMetaSkeleton().get());
             std::shared_ptr<dart::collision::CollisionGroup> envCollisionGroup =
                 collisionDetector->createCollisionGroup();
             for (const auto& envSkeleton : envSkeletons) {
               envCollisionGroup->addShapeFramesOf(envSkeleton.get());
             }
             std::shared_ptr<aikido::constraint::dart::CollisionFree> collisionFreeConstraint =
                 std::make_shared<aikido::constraint::dart::CollisionFree>(armSpace,
                                                                           armSkeleton,
                                                                           collisionDetector);
             collisionFreeConstraint->addPairwiseCheck(armCollisionGroup, envCollisionGroup);
             return collisionFreeConstraint;
           })
      .def("get_full_collision_constraint",
           [](ada::Ada *self,
              const aikido::statespace::dart::MetaSkeletonStateSpacePtr &armSpace,
              const dart::dynamics::MetaSkeletonPtr& armSkeleton,
              const aikido::constraint::dart::CollisionFreePtr& collisionFreeConstraint) -> aikido::constraint::TestablePtr {
             return self->getFullCollisionConstraint(armSpace, armSkeleton, collisionFreeConstraint);
           })
      .def("compute_joint_space_path",
           [](ada::Ada *self,
              const aikido::statespace::dart::MetaSkeletonStateSpacePtr &stateSpace,
              const std::vector<std::pair<double, Eigen::VectorXd>> &waypoints)
               -> aikido::trajectory::TrajectoryPtr {
             return self->computeJointSpacePath(stateSpace, waypoints);
           })
      .def("compute_smooth_joint_space_path",
           [](ada::Ada *self,
              const aikido::statespace::dart::MetaSkeletonStateSpacePtr &stateSpace,
              const std::vector<std::pair<double, Eigen::VectorXd>> &waypoints,
              const aikido::constraint::dart::CollisionFreePtr &collisionFreeConstraint=nullptr)
               -> aikido::trajectory::TrajectoryPtr {
             return self->computeSmoothJointSpacePath(stateSpace, waypoints, collisionFreeConstraint);
           })
      .def("compute_retime_path",
           [](ada::Ada *self,
              const dart::dynamics::MetaSkeletonPtr& armSkeleton,
              aikido::trajectory::TrajectoryPtr trajectory_ptr) -> aikido::trajectory::TrajectoryPtr {
        return self->retimePath(armSkeleton, trajectory_ptr.get());
      })
      .def("plan_to_configuration",
           [](ada::Ada *self,
              const aikido::statespace::dart::MetaSkeletonStateSpacePtr &armSpace,
              const dart::dynamics::MetaSkeletonPtr& armSkeleton,
              const Eigen::VectorXd& configuration) -> aikido::trajectory::TrajectoryPtr {
              auto state = armSpace->createState();
              armSpace->convertPositionsToState(configuration, state);
              auto trajectory = self->planToConfiguration(armSpace,
                                                          armSkeleton,
                                                          state,
                                                          nullptr,
                                                          10);
              return trajectory;
          })
      .def("execute_trajectory",
           [](ada::Ada *self,
              const aikido::trajectory::TrajectoryPtr &trajectory)
               -> void {
             auto future = self->executeTrajectory(trajectory);

//             if (!future.valid()) {
//               std::__throw_future_error(0);
//             }
//
//             future.wait();
//              ros::Duration(15).sleep();
           })
      .def("start_viewer",
           [](ada::Ada *self, const std::string &topicName, const std::string &baseFrameName)
               -> std::shared_ptr<aikido::rviz::WorldInteractiveMarkerViewer> {
             int argc = 1;
             char *argv[] = {"adapy"};
             ros::init(argc, argv, "ada");
             auto viewer = std::make_shared<aikido::rviz::WorldInteractiveMarkerViewer>(
                 self->getWorld(),
                 topicName,
                 baseFrameName);
             viewer->setAutoUpdate(true);
             return viewer;
           });
  py::class_<ada::AdaHand, std::shared_ptr<ada::AdaHand>>(m, "AdaHand")
      .def("get_skeleton",
           [](ada::AdaHand *self) -> dart::dynamics::MetaSkeletonPtr {
        return self->getMetaSkeleton();
      })
    .def("get_state_space",
           [](ada::AdaHand *self) -> aikido::statespace::dart::MetaSkeletonStateSpacePtr {
        auto handSkeleton = self->getMetaSkeleton();
        auto handSpace =   std::make_shared<aikido::statespace::dart::MetaSkeletonStateSpace>(handSkeleton.get());
        return handSpace;
      })
    .def("execute_preshape",
           [](ada::AdaHand *self, const Eigen::Vector2d &d)-> void {
		auto future = self->executePreshape(d);
//                future.wait();
//        ros::Duration(5).sleep();
           })
      .def("get_endeffector_transform",
           [](ada::AdaHand *self,
              const std::string& objectType) -> Eigen::Matrix4d {
        return self->getEndEffectorTransform(objectType)->matrix();
      })
      .def("get_endeffector_body_node",
          [](ada::AdaHand *self) -> dart::dynamics::BodyNode* {
        return self->getEndEffectorBodyNode();
      })
      .def("grab",
          [](ada::AdaHand *self, dart::dynamics::SkeletonPtr object) -> void {
           self->grab(object);
      });
}

