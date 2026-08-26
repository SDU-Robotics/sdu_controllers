#pragma once

#ifndef INTEGRATOR_HPP
#define INTEGRATOR_HPP

#include <Eigen/Core>
#include <functional>
#include <iostream>


namespace sdu_controllers::integrator 
{
    /**
     * @brief Utility functions for integrating different methods.
     *  
     * Inspired by the implementation of the integrators in 
     * https://github.com/sfwa/ukf/blob/master/include/UKF/Integrator.h
     *
     * This file is directly pulled from sdu_estimators
     * https://github.com/SDU-Robotics/sdu_estimators/blob/main/include/sdu_estimators/integrator/integrator.hpp
     */
    enum class IntegrationMethod
    {
      Euler,
      RK2,
      RK4
    };

    template <typename T, int32_t DIM_N, int32_t DIM_P>
    class Integrator
    {
        public:
            using State = Eigen::Matrix<T, DIM_N, DIM_P>;

            static State integrate(
                const State & y,
                const std::function<State(
                    const State&
                )> & get_dydt,
                float delta,
                IntegrationMethod method = IntegrationMethod::RK2
            )
            {
                State outval;

                switch (method) 
                {
                    case IntegrationMethod::Euler:
                        outval = integrate_euler(y, get_dydt, delta);
                        // std::cout << "euler" << std::endl;
                        break;

                    case IntegrationMethod::RK2:
                        outval = integrate_rk2(y, get_dydt, delta);
                        // std::cout << "rk2" << std::endl;
                        break;

                    case IntegrationMethod::RK4:
                        outval = integrate_rk4(y, get_dydt, delta);
                        // std::cout << "rk4" << std::endl;
                        break;
                }

                return outval;
            }

        private:
            static State integrate_euler(
                const State & y,
                const std::function<State(
                    const State&
                )> & get_dydt,
                float delta)
            {
                return y + delta * get_dydt(y);
            }

            static State integrate_rk2(
                const State & y,
                const std::function<State(
                    const State&
                )> & get_dydt,
                float delta)
            {
                State k1 = delta * get_dydt(y);
                State k2 = delta * get_dydt(y + 0.5 * k1);
                return y + k2;
            }

            static State integrate_rk4(
                const State & y,
                const std::function<State(
                    const State&
                )> & get_dydt,
                float delta)
            {
                State k1 = delta * get_dydt(y);
                State k2 = delta * get_dydt(y + 0.5 * k1);
                State k3 = delta * get_dydt(y + 0.5 * k2);
                State k4 = delta * get_dydt(y + k3);

                return y + (k1 + 2 * k2 + 2 * k3 + k4) / 6.;
            }
    };
}

#endif  //INTEGRATOR_HPP
