#include <dart/collision/CollisionDetector.hpp>
#include <dart/collision/CollisionGroup.hpp>
#include <dart/dart.hpp>
#include <dart/utils/urdf/DartLoader.hpp>
#include <pybind11/cast.h>
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

//==============================================================================
// NOTE: These functions define the Python API for Skeleton.

std::string get_name(dart::dynamics::Skeleton* self)
{
  return self->getName();
}

Eigen::VectorXd get_positions(dart::dynamics::Skeleton* self)
{
  return self->getPositions();
}

int get_num_joints(dart::dynamics::Skeleton* self)
{
  return self->getNumJoints();
}

//==================================DART========================================

void Dart(pybind11::module& m)
{
  py::class_<
      dart::dynamics::Skeleton,
      std::shared_ptr<dart::dynamics::Skeleton>>(m, "Skeleton")
      .def("get_name", get_name)
      .def("get_positions", get_positions)
      .def("get_num_joints", get_num_joints);
  py::class_<
      dart::dynamics::MetaSkeleton,
      std::shared_ptr<dart::dynamics::MetaSkeleton>>(m, "MetaSkeleton")
      .def(
          "get_name",
          [](dart::dynamics::MetaSkeleton* self) -> std::string {
            return self->getName();
          })
      .def(
          "get_positions",
          [](dart::dynamics::MetaSkeleton* self) -> Eigen::VectorXd {
            return self->getPositions();
          })
      .def(
          "get_num_joints",
          [](dart::dynamics::MetaSkeleton* self) -> int {
            return self->getNumJoints();
          })
      .def(
          "get_linear_jacobian",
          [](dart::dynamics::MetaSkeleton* self,
             const dart::dynamics::BodyNode* _node)
              -> dart::math::LinearJacobian {
            return self->getLinearJacobian(_node);
          })
      .def(
          "get_jacobian",
          [](dart::dynamics::MetaSkeleton* self,
             const dart::dynamics::BodyNode* _node) -> dart::math::Jacobian {
            return self->getJacobian(_node);
          });
}
