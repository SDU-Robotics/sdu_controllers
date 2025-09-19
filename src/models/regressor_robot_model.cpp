#include <sdu_controllers/math/math.hpp>
#include <sdu_controllers/models/regressor_robot_model.hpp>

using namespace sdu_controllers::models;

RegressorRobotModel::RegressorRobotModel(std::vector<kinematics::DHParam> dh_parameters, const Eigen::Vector3d& g0)
    : dh_parameters_(std::move(dh_parameters)),
      gravity_(g0)
{
  if (dh_parameters_.empty())
  {
    throw std::runtime_error("RegressorRobotModel: No DH parameters provided");
  }
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

  grav = get_tau(q, zero_vec, zero_vec);
  return grav;
}

Eigen::MatrixXd RegressorRobotModel::get_gravity(const std::vector<double>& q)
{
  const Eigen::VectorXd q_eigen = Eigen::Map<const Eigen::VectorXd>(q.data(), q.size());
  return get_gravity(q_eigen);
}

Eigen::MatrixXd RegressorRobotModel::get_jacobian(const Eigen::VectorXd& q)
{
  return math::geometric_jacobian(q, dh_parameters_);
}

Eigen::MatrixXd RegressorRobotModel::get_jacobian_dot(const Eigen::VectorXd& q, const Eigen::VectorXd& dq)
{
  throw std::runtime_error("RegressorRobotModel: get_jacobian_dot not implemented");
}

uint16_t RegressorRobotModel::get_dof() const
{
  return static_cast<uint16_t>(dh_parameters_.size());
}

std::vector<double> RegressorRobotModel::get_a()
{
  std::vector<double> a;
  for (const auto& param : dh_parameters_)
  {
    a.push_back(param.a);
  }
  return a;
}

std::vector<double> RegressorRobotModel::get_d()
{
  std::vector<double> d;
  for (const auto& param : dh_parameters_)
  {
    d.push_back(param.d);
  }
  return d;
}

std::vector<double> RegressorRobotModel::get_alpha()
{
  std::vector<double> alpha;
  for (const auto& param : dh_parameters_)
  {
    alpha.push_back(param.alpha);
  }
  return alpha;
}

std::vector<double> RegressorRobotModel::get_theta()
{
  std::vector<double> theta;
  for (const auto& param : dh_parameters_)
  {
    theta.push_back(param.theta);
  }
  return theta;
}

std::vector<bool> RegressorRobotModel::get_is_joint_revolute()
{
  std::vector<bool> is_revolute;
  for (const auto& param : dh_parameters_)
  {
    is_revolute.push_back(param.is_joint_revolute);  // true if revolute, false if prismatic
  }
  return is_revolute;
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

Eigen::VectorXd RegressorRobotModel::get_tau(const Eigen::VectorXd& q, const Eigen::VectorXd& qd, const Eigen::VectorXd& qdd)
{
  return get_regressor(q, qd, qdd) * get_parameters() + get_friction_regressor(qd) * get_friction_parameters();
}

Eigen::MatrixXd RegressorRobotModel::get_friction_regressor(const std::vector<double>& qd) const
{
  Eigen::VectorXd qd_eigen = Eigen::Map<const Eigen::VectorXd>(qd.data(), qd.size());
  return get_friction_regressor(qd_eigen);
}
