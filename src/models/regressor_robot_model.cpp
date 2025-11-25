#include <sdu_controllers/math/math.hpp>
#include <sdu_controllers/models/regressor_robot_model.hpp>

using namespace sdu_controllers::models;
using namespace sdu_controllers;

RegressorRobotModel::RegressorRobotModel(
    const std::shared_ptr<kinematics::ForwardKinematics>& fkModel,
    const Eigen::Vector3d& g0)
    : fk_model_(fkModel),
      gravity_(g0)
{
  if (!fk_model_)
  {
    throw std::invalid_argument("RegressorRobotModel: No FK model provided");
  }
}

Eigen::VectorXd RegressorRobotModel::inverse_dynamics(
    const Eigen::VectorXd& q,
    const Eigen::VectorXd& dq,
    const Eigen::VectorXd& ddq,
    const Eigen::VectorXd& he)
{
  return get_regressor(q, dq, ddq) * get_parameters() + get_friction_regressor(dq) * get_friction_parameters();
}

Eigen::VectorXd
RegressorRobotModel::forward_dynamics(const Eigen::VectorXd& q, const Eigen::VectorXd& dq, const Eigen::VectorXd& tau)
{
  Eigen::VectorXd zero_vec = Eigen::VectorXd::Zero(q.rows());

  // Get C(q, dq) dq + g(q)
  Eigen::VectorXd tau_bar = inverse_dynamics(q, dq, zero_vec, zero_vec);
  Eigen::MatrixXd B = get_inertia_matrix(q);
  return B.lu().solve(tau - tau_bar);
}

Eigen::MatrixXd RegressorRobotModel::get_inertia_matrix(const Eigen::VectorXd& q)
{
  Eigen::VectorXd zero_vec = Eigen::VectorXd::Zero(q.rows()), he0 = Eigen::VectorXd::Zero(6),
                  ddq_bar = Eigen::VectorXd::Zero(q.rows()), tau_tmp = Eigen::VectorXd::Zero(q.rows());

  Eigen::MatrixXd B = Eigen::MatrixXd::Zero(q.rows(), q.rows());

  for (int i = 0; i < q.rows(); ++i)
  {
    ddq_bar.setZero();
    ddq_bar(i) = 1.0;
    tau_tmp = get_regressor(q, zero_vec, ddq_bar) * get_parameters() - get_gravity(q);
    B(Eigen::all, i) = tau_tmp;
  }

  return B;
}

Eigen::MatrixXd RegressorRobotModel::get_coriolis(const Eigen::VectorXd& q, const Eigen::VectorXd& qd)
{
  if (q.rows() != qd.rows())
  {
    throw std::runtime_error("q and qd must have the same number of rows");
  }

  Eigen::Index n = q.rows();
  Eigen::MatrixXd C = Eigen::MatrixXd::Zero(n, n);
  Eigen::MatrixXd Csq = Eigen::MatrixXd::Zero(n, n);

  // Find the torques that depend on a single finite joint speed,these are due
  // to the squared(centripetal) terms set QD = [1 0 0 ...] then resulting
  // torque is due to qd_1 ^ 2

  Eigen::VectorXd QD = Eigen::VectorXd::Zero(n), zero = Eigen::VectorXd::Zero(n), tau = Eigen::VectorXd::Zero(n);

  for (size_t i = 0; i < n; ++i)
  {
    QD.setZero();
    QD[i] = 1;
    tau = get_regressor(q, QD, zero) * get_parameters() - get_gravity(q);
    Csq.col(i) = Csq.col(i) + tau;
  }

  // Find the torques that depend on a pair of finite joint speeds,
  // these are due to the product(Coriolis) terms
  // set QD = [1 1 0 ...] then resulting torque is due to
  // qd_1 qd_2 + qd_1 ^ 2 + qd_2 ^ 2
  for (size_t i = 0; i < n; ++i)
  {
    for (size_t j = i + 1; j < n; ++j)
    {
      // Find a product term qd_i *qd_j
      QD.setZero();
      QD[i] = 1;
      QD[j] = 1;
      tau = get_regressor(q, QD, zero) * get_parameters() - get_gravity(q);

      C.col(j) = C.col(j) + (tau - Csq.col(j) - Csq.col(i)) * qd[i] / 2;
      C.col(i) = C.col(i) + (tau - Csq.col(j) - Csq.col(i)) * qd[j] / 2;
    }
  }

  C = C + Csq * qd.asDiagonal();

  return C;
}

Eigen::MatrixXd RegressorRobotModel::get_gravity(const Eigen::VectorXd& q)
{
  Eigen::VectorXd zero_vec = Eigen::VectorXd::Zero(q.rows()), he0 = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd grav = Eigen::VectorXd::Zero(q.rows());

  grav = get_regressor(q, zero_vec, zero_vec) * get_parameters();
  return grav;
}

Eigen::MatrixXd RegressorRobotModel::get_gravity(const std::vector<double>& q)
{
  const Eigen::VectorXd q_eigen = Eigen::Map<const Eigen::VectorXd>(q.data(), q.size());
  return get_gravity(q_eigen);
}

Eigen::MatrixXd RegressorRobotModel::get_jacobian(const Eigen::VectorXd& q)
{
  return fk_model_->geometric_jacobian(q);
}

Eigen::MatrixXd RegressorRobotModel::get_jacobian_dot(const Eigen::VectorXd& q, const Eigen::VectorXd& dq)
{
  const double dt = 1e-6;

  // Calculate the Jacobian at the current configuration q
  Eigen::MatrixXd J_current = get_jacobian(q);

  // Estimate the configuration at the next time step (q_next) using Euler integration
  Eigen::VectorXd q_next = q + (dq * dt);

  // Calculate the Jacobian at the predicted next configuration
  Eigen::MatrixXd J_next = get_jacobian(q_next);

  // Compute the numerical derivative
  Eigen::MatrixXd J_dot = (J_next - J_current) / dt;

  return J_dot;
}

uint16_t RegressorRobotModel::get_dof() const
{
  return static_cast<uint16_t>(fk_model_->get_dof());
}

std::vector<double> RegressorRobotModel::get_m()
{
  throw std::runtime_error("RegressorRobotModel: get_m not implemented");
}

Eigen::Vector3d RegressorRobotModel::get_g0()
{
  return gravity_;
}

Eigen::Matrix<double, Eigen::Dynamic, 3> RegressorRobotModel::get_CoM()
{
  throw std::runtime_error("RegressorRobotModel: get_CoM not implemented");
}

std::vector<Eigen::Matrix3d> RegressorRobotModel::get_link_inertia()
{
  throw std::runtime_error("RegressorRobotModel: get_link_inertia not implemented");
}

Eigen::MatrixXd RegressorRobotModel::get_friction_regressor(const std::vector<double>& qd) const
{
  Eigen::VectorXd qd_eigen = Eigen::Map<const Eigen::VectorXd>(qd.data(), qd.size());
  return get_friction_regressor(qd_eigen);
}

const kinematics::ForwardKinematics& RegressorRobotModel::get_fk_solver() const
{
  return *fk_model_;
}


