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

std::string skeleton_get_name(dart::dynamics::Skeleton* self)
{
  return self->getName();
}

Eigen::VectorXd skeleton_get_positions(dart::dynamics::Skeleton* self)
{
  return self->getPositions();
}

int skeleton_get_num_joints(dart::dynamics::Skeleton* self)
{
  return self->getNumJoints();
}

//==============================================================================
// NOTE: These functions define the Python API for MetaSkeleton.

std::string metaskeleton_get_name(dart::dynamics::MetaSkeleton* self)
{
  return self->getName();
}

Eigen::VectorXd metaskeleton_get_positions(dart::dynamics::MetaSkeleton* self)
{
  return self->getPositions();
}

int metaskeleton_get_num_joints(dart::dynamics::MetaSkeleton* self)
{
  return self->getNumJoints();
}

dart::math::LinearJacobian get_linear_jacobian(
    dart::dynamics::MetaSkeleton* self, const dart::dynamics::BodyNode* _node)
{
  return self->getLinearJacobian(_node);
}

dart::math::Jacobian get_jacobian(
    dart::dynamics::MetaSkeleton* self, const dart::dynamics::BodyNode* _node)
{
  return self->getJacobian(_node);
}

//==================================DART========================================

void Dart(pybind11::module& m)
{
  py::class_<
      dart::dynamics::Skeleton,
      std::shared_ptr<dart::dynamics::Skeleton>>(m, "Skeleton")
      .def("get_name", skeleton_get_name)
      .def("get_positions", skeleton_get_positions)
      .def("get_num_joints", skeleton_get_num_joints);

  py::class_<
      dart::dynamics::MetaSkeleton,
      std::shared_ptr<dart::dynamics::MetaSkeleton>>(m, "MetaSkeleton")
      .def("get_name", metaskeleton_get_name)
      .def("get_positions", metaskeleton_get_positions)
      .def("get_num_joints", metaskeleton_get_num_joints)
      .def("get_linear_jacobian", get_linear_jacobian)
      .def("get_jacobian", get_jacobian);
}
