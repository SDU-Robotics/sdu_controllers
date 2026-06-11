// Python bindings for sdu_controller integrator functions

#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>
#include <nanobind/eigen/dense.h>
#include <nanobind/stl/function.h> 
#include <nanobind/stl/vector.h>

#include <sdu_controllers/integrator/integrator.hpp>

namespace nb = nanobind;

namespace sdu_controllers
{
    template <typename T, int32_t DIM_N, int32_t DIM_P>
    void nb_IntegratorClass(nb::module_ & m)
    {
        std::string typestr;
        typestr = "_" + std::to_string(DIM_N) + "x" + std::to_string(DIM_P);

        using Class = integrator::Integrator<T, DIM_N, DIM_P>;
        using State = Eigen::Matrix<T, DIM_N, DIM_P>;

        std::string nbclass_name = std::string("integrate") + typestr;

        m.def(nbclass_name.c_str(),
            &Class::integrate,
            nb::arg("state"),
            nb::arg("get_dydt"),
            nb::arg("delta"),
            nb::arg("method"));
    }

    nb::module_ create_integrator_module(nb::module_ & main_module)
    {
        nb::module_ m = main_module.def_submodule("integrator", "Submodule containing definitions for integrators.");
        m.doc() = "Python bindings for sdu_controllers integrator utilities.";

        nb::enum_<integrator::IntegrationMethod>(m, "IntegrationMethod")
            .value("Euler", integrator::IntegrationMethod::Euler)
            .value("RK2", integrator::IntegrationMethod::RK2)
            .value("RK4", integrator::IntegrationMethod::RK4);
            
        nb_IntegratorClass<double, 3, 1>(m);

        return m;
    }
}