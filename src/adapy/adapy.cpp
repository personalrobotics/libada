#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/cast.h>
#include <pybind11/stl.h>

namespace py = pybind11;

void Ada(pybind11::module& m);
void Aikido(pybind11::module& m);
void Dart(pybind11::module& m);
void Pr_Tsr(pybind11::module& m);
void IK(pybind11::module& m);

PYBIND11_MODULE(adapy, m) {
  Ada(m);

  Aikido(m);

  Dart(m);

  Pr_Tsr(m);

  IK(m);
}
