#ifndef SDU_CONTROLLERS_MOMENTUM_OBSERVER_HPP
#define SDU_CONTROLLERS_MOMENTUM_OBSERVER_HPP

#include <Eigen/Core>
#include <memory>  // for std::shared_ptr
#include <sdu_controllers/models/robot_model.hpp>

namespace sdu_controllers::math
{

  class MomentumObserver
  {
   public:
    MomentumObserver(const std::shared_ptr<sdu_controllers::models::RobotModel>& model, double dt, const Eigen::VectorXd& K);

    void reset();

    void update(const Eigen::VectorXd& q, const Eigen::VectorXd& qd, const Eigen::VectorXd& tau_m);

    void update(const std::vector<double>& q, const std::vector<double>& qd, const std::vector<double>& tau_m);

    Eigen::VectorXd getTauEstimate() const;

    Eigen::VectorXd getFTEstimate_TCP() const;

    Eigen::VectorXd getFTEstimate_Base() const;

    Eigen::VectorXd getAccEstimate(const Eigen::VectorXd& q, const Eigen::VectorXd& qd, const Eigen::VectorXd& tau_m) const;

    void zeroMomentumObserver();

   private:
    std::shared_ptr<sdu_controllers::models::RobotModel> _model;
    double _dt;
    Eigen::MatrixXd _K;  // Gain matrix for the observer
    Eigen::VectorXd _r;
    Eigen::VectorXd _internal_r_sum;  // Internal state for the observer
    Eigen::VectorXd _last_q;
    bool _initialized;
  };
}  // namespace sdu_controllers

#endif