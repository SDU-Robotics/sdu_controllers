#pragma once
#ifndef SDU_CONTROLLERS_SLIDING_MODE_JOINT_SPACE_HPP
#define SDU_CONTROLLERS_SLIDING_MODE_JOINT_SPACE_HPP

#include <Eigen/Dense>
#include <memory>
#include <sdu_controllers/controllers/controller.hpp>
#include <sdu_controllers/models/robot_model.hpp>

namespace sdu_controllers::controllers
{
    class SlidingModeControllerJoint : public Controller
    {
        public:
            /**
             * @brief Construct a new Sliding Mode Controller Joint object
             * 
             * @param lambda        Parameter used in the definition of the sliding surface;  $s = \tilde{\dot{q}} + \lambda * \tilde{q}$.
             * @param K1            Tuning parameter.
             * @param K2            Tuning parameter.
             * @param order         Order of the sliding mode control (default: 1).
             * @param dt            Time-step for the integration.
             */
            explicit SlidingModeControllerJoint(double lambda, Eigen::MatrixXd K1, Eigen::MatrixXd K2,
                int order = 1, double dt = 0.002);

            /**
                * @brief Step the execution of the controller.
                * @param q_d    Desired joint configuration
                * @param dq_d   Desired joint speed
                * @param ddq_d  Desired joint acceleration
                * @param q      Actual joint configuration
                * @param dq     Actual joint speed
                */
            void step(const Eigen::VectorXd& q_d, const Eigen::VectorXd& dq_d, const Eigen::VectorXd& ddq_d, 
                const Eigen::VectorXd& q, const Eigen::VectorXd& dq);

            /**
                * @brief Get the output of the controller. Updates when the step() function is called.
                */
            Eigen::VectorXd get_output() override;

            /**
                * @brief Reset internal controller variables.
                */
            void reset() override;

        private:
            double lambda_;

            Eigen::VectorXd y_, omega_, domega_;
            Eigen::MatrixXd K1_, K2_;
            double dt_;

            int order_;
    };
}  // namespace sdu_controllers::controllers

#endif  // SDU_CONTROLLERS_SLIDING_MODE_JOINT_SPACE_HPP
