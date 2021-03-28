#include <aikido/constraint/dart/TSR.hpp>
#include <pr_tsr/can.hpp>
#include <pybind11/cast.h>
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

Eigen::Isometry3d vectorToIsometry(std::vector<double>& poseVector);
Eigen::Isometry3d matrixToIsometry(Eigen::Matrix4d& poseMatrix);

//==============================================================================
// NOTE: These functions define the Python API for TSR.

Eigen::Matrix4d get_T0_w(aikido::constraint::dart::TSR* self)
{
  return self->mT0_w.matrix();
}

void set_T0_w(aikido::constraint::dart::TSR* self, Eigen::Matrix4d& pose)
{
  self->mT0_w = matrixToIsometry(pose);
}

void set_Bw(aikido::constraint::dart::TSR* self, Eigen::MatrixXf& Bw)
{
  self->mBw = Bw.cast<double>();
}

Eigen::Matrix4d get_Tw_e(aikido::constraint::dart::TSR* self)
{
  return self->mTw_e.matrix();
}

void set_Tw_e(aikido::constraint::dart::TSR* self, Eigen::Matrix4d& pose)
{
  self->mTw_e = matrixToIsometry(pose);
}

//==================================TSR=========================================

void Pr_Tsr(pybind11::module& m)
{
  m.def("get_default_TSR", &pr_tsr::getDefaultCanTSR);

  py::class_<
      aikido::constraint::dart::TSR,
      std::shared_ptr<aikido::constraint::dart::TSR>>(m, "TSR")
      .def("get_T0_w", get_T0_w)
      .def("set_T0_w", set_T0_w)
      .def("set_Bw", set_Bw)
      .def("get_Tw_e", get_Tw_e)
      .def("set_Tw_e", set_Tw_e);
}
