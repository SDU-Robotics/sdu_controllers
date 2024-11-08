#pragma once
#ifndef SDU_CONTROLLERS_OPERATIONAL_SPACE_CONTROLLER_HPP
#define SDU_CONTROLLERS_OPERATIONAL_SPACE_CONTROLLER_HPP

#include <Eigen/Dense>
#include <memory>
#include <sdu_controllers/controllers/controller.hpp>
#include <sdu_controllers/models/robot_model.hpp>

namespace sdu_controllers::controllers
{
  /**
   * Operational (or cartesian) space controller with inverse dynamics control. The
   * controller is implemented according to Eq. (8.114) from page 348, Robotics: Modelling, Planning and Control:
   *
   * \f$ \mathbf{y} = \mathbf{J}_{A}^{-1}(q)\left(\ddot{x}_{d} + \mathbf{K}_{D}\dot{\tilde{x}} +
   * \mathbf{K}_{P}\tilde{x}-\mathbf{\dot{J}}_{A}(q, \dot{q})\dot{q}\right) \f$
   *
   */
  class OperationalSpaceController : public Controller
  {
   public:
    /**
     * @brief Step the execution of the controller.
     */
    explicit
    OperationalSpaceController(Eigen::MatrixXd Kp, Eigen::MatrixXd Kd, std::shared_ptr<models::RobotModel> robot_model);

    /**
     * @brief Step the execution of the controller.
     */
    void step(
        const Eigen::VectorXd &x_d,
        const Eigen::VectorXd &dx_d,
        const Eigen::VectorXd &ddx_d,
        const Eigen::VectorXd &q,
        const Eigen::VectorXd &dq);

    /**
     * @brief Get the output of the controller. Updates when the step() function is called.
     */
    Eigen::VectorXd get_output() override;

    /**
     * @brief Reset internal controller variables.
     */
    void reset() override;

  private:
    Eigen::VectorXd y_;
    Eigen::MatrixXd Kp_, Kd_;
    std::shared_ptr<models::RobotModel> robot_model_;
  };

}  // namespace sdu_controllers::controllers

#endif  // SDU_CONTROLLERS_OPERATIONAL_SPACE_CONTROLLER_HPP
