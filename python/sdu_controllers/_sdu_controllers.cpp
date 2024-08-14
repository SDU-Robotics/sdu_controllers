#include <nanobind/nanobind.h>
#include <sdu_controllers/sdu_controllers.hpp>

namespace nb = nanobind;

namespace sdu_controllers
{

NB_MODULE(_sdu_controllers, m)
{
  m.doc() = "Python Bindings for sdu_controllers";
  m.def("add_one", &add_one, "Increments an integer value");
}

}  // namespace sdu_controllers
