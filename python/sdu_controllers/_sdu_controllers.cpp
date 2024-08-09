#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "sdu_controllers/sdu_controllers.hpp"

namespace py = pybind11;

namespace sdu_controllers
{

PYBIND11_MODULE(_sdu_controllers, m)
{
  m.doc() = "Python Bindings for sdu_controllers";
  m.def("add_one", &add_one, "Increments an integer value");
}

}  // namespace sdu_controllers
