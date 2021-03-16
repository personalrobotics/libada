#include <aikido/constraint/dart/TSR.hpp>
#include <pr_tsr/can.hpp>
#include <pybind11/cast.h>
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

Eigen::Isometry3d vectorToIsometry(std::vector<double>& poseVector);
Eigen::Isometry3d matrixToIsometry(Eigen::Matrix4d& poseMatrix);

void Pr_Tsr(pybind11::module& m)
{
  m.def("get_default_TSR", &pr_tsr::getDefaultCanTSR);

  py::class_<
      aikido::constraint::dart::TSR,
      std::shared_ptr<aikido::constraint::dart::TSR>>(m, "TSR")
      .def(
          "get_T0_w",
          [](aikido::constraint::dart::TSR* self) -> Eigen::Matrix4d {
            return self->mT0_w.matrix();
          })
      .def(
          "set_T0_w",
          [](aikido::constraint::dart::TSR* self,
             Eigen::Matrix4d& pose) -> void {
            self->mT0_w = matrixToIsometry(pose);
          })
      .def(
          "set_Bw",
          [](aikido::constraint::dart::TSR* self, Eigen::MatrixXf& Bw) -> void {
            self->mBw = Bw.cast<double>();
          })
      .def(
          "get_Tw_e",
          [](aikido::constraint::dart::TSR* self) -> Eigen::Matrix4d {
            return self->mTw_e.matrix();
          })
      .def(
          "set_Tw_e",
          [](aikido::constraint::dart::TSR* self,
             Eigen::Matrix4d& pose) -> void {
            self->mTw_e = matrixToIsometry(pose);
          });
}
