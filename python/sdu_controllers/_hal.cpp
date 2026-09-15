// Python bindings for sdu_controllers hardware abstraction (Eurofusion robot)
#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/array.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

#include <sdu_controllers/hal/eurofusion.hpp>

namespace nb = nanobind;

namespace sdu_controllers
{
  nb::module_ create_hal_module(nb::module_ &main_module)
  {
    nb::module_ m = main_module.def_submodule("hal", "Submodule containing hardware abstraction.");
    m.doc() = "Python bindings for sdu_controllers hardware abstraction.";

    nb::class_<hal::EurofusionRobot>(m, "EurofusionRobot")
        .def(nb::init<>(), "Construct an Eurofusion robot HAL instance.")
        .def(
            "generalized_tau_2_motor_tau",
            &hal::EurofusionRobot::generalized_tau_2_motor_tau,
            nb::arg("tau_g"),
            nb::arg("q"),
            nb::arg("qd"),
            "Convert generalized joint torques to motor torques, given joint positions and velocities.");

    return m;
  }
}  // namespace sdu_controllers