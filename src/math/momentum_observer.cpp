#include <Eigen/Eigen>
#include <sdu_controllers/kinematics/forward_kinematics.hpp>
#include <sdu_controllers/math/math.hpp>
#include <sdu_controllers/math/momentum_observer.hpp>

using namespace sdu_controllers::math;

MomentumObserver::MomentumObserver(
    const std::shared_ptr<sdu_controllers::models::RobotModel>& model,
    double dt,
    const Eigen::VectorXd& K)
    : _model(model),
      _dt(dt),
      _K(K.asDiagonal()),
      _initialized(false)
{
  if (K.size() != model->get_dof())
  {
    throw std::invalid_argument("Gain matrix K size must match the model DOF.");
  }
  _r = Eigen::VectorXd::Zero(model->get_dof());
  _internal_r_sum = Eigen::VectorXd::Zero(model->get_dof());
  _last_q = Eigen::VectorXd::Zero(model->get_dof());
}

void MomentumObserver::reset()
{
  _r.setZero();
  _internal_r_sum.setZero();
  _initialized = false;
  _last_q.setZero();
}

void MomentumObserver::update(const std::vector<double>& q, const std::vector<double>& qd, const std::vector<double>& tau_m)
{
  const Eigen::VectorXd q_eigen = Eigen::Map<const Eigen::VectorXd>(q.data(), q.size());
  const Eigen::VectorXd qd_eigen = Eigen::Map<const Eigen::VectorXd>(qd.data(), qd.size());
  const Eigen::VectorXd tau_eigen = Eigen::Map<const Eigen::VectorXd>(tau_m.data(), tau_m.size());
  update(q_eigen, qd_eigen, tau_eigen);
}

void MomentumObserver::update(const Eigen::VectorXd& q, const Eigen::VectorXd& qd, const Eigen::VectorXd& tau)
{
  Eigen::MatrixXd B = _model->get_inertia_matrix(q), C = _model->get_coriolis(q, qd), g = _model->get_gravity(q);

  Eigen::MatrixXd friction = _model->get_friction(qd);

  Eigen::VectorXd beta = g - C.transpose() * qd + friction;

  // Compute the equivalent force
  Eigen::MatrixXd Feq = C * qd + g;

  Eigen::VectorXd momentum = B * qd;

  if (!_initialized)
  {
    _initialized = true;
    _internal_r_sum = momentum;
  }
  _internal_r_sum += _dt * (tau - beta + _r);
  _r = _K * (momentum - _internal_r_sum);
  _last_q = q;
}

Eigen::VectorXd MomentumObserver::getTauEstimate() const
{
  return _r;  // Return the external joint torque estimate
}

Eigen::VectorXd MomentumObserver::getFTEstimate_Base() const
{
  const std::vector<double>& a = _model->get_a();
  const std::vector<double>& d = _model->get_d();
  const std::vector<double>& alpha = _model->get_alpha();
  const std::vector<double>& theta = _model->get_theta();
  const std::vector<bool>& is_joint_revolute = _model->get_is_joint_revolute();

  std::vector<kinematics::DHParam> dh_parameters;
  for (size_t i = 0; i < a.size(); ++i)
  {
    kinematics::DHParam dh;
    dh.a = a[i];
    dh.d = d[i];
    dh.alpha = alpha[i];
    dh.theta = theta[i];
    dh.is_joint_revolute = is_joint_revolute[i];
    dh_parameters.push_back(dh);
  }

  Eigen::MatrixXd J = math::jacobian(_last_q, dh_parameters);

  Eigen::VectorXd ft_base = J.transpose().inverse() * getTauEstimate();  // External force/torque at the base frame

  return ft_base;  // Return the external force/torque at the base frame
}

Eigen::VectorXd MomentumObserver::getFTEstimate_TCP() const
{
  const std::vector<double>& a = _model->get_a();
  const std::vector<double>& d = _model->get_d();
  const std::vector<double>& alpha = _model->get_alpha();
  const std::vector<double>& theta = _model->get_theta();
  const std::vector<bool>& is_joint_revolute = _model->get_is_joint_revolute();

  std::vector<kinematics::DHParam> dh_parameters;
  for (size_t i = 0; i < a.size(); ++i)
  {
    kinematics::DHParam dh;
    dh.a = a[i];
    dh.d = d[i];
    dh.alpha = alpha[i];
    dh.theta = theta[i];
    dh.is_joint_revolute = is_joint_revolute[i];
    dh_parameters.push_back(dh);
  }
  std::vector<Eigen::Matrix4d> fk_chain = kinematics::forward_kinematics_all(_last_q, dh_parameters);
  Eigen::MatrixXd J = math::jacobian(fk_chain, dh_parameters);

  Eigen::VectorXd ft_base = J.transpose() * getTauEstimate();  // External force/torque at the base frame

  // separate the force and torque components
  Eigen::Vector3d f_base(ft_base[0], ft_base[1], ft_base[2]);
  Eigen::Vector3d mu_base(ft_base[3], ft_base[4], ft_base[5]);

  Eigen::Affine3d T_base_tcp(fk_chain.back());
  Eigen::Matrix3d R_tcp_base = T_base_tcp.rotation().inverse();  // Rotation from TCP to base frame
  Eigen::Vector3d f_tcp = R_tcp_base * f_base;                   // Transform force to TCP frame
  Eigen::Vector3d mu_tcp = R_tcp_base * mu_base;                 // Transform torque to TCP frame

  Eigen::VectorXd ft_tcp(6);
  ft_tcp << f_tcp, mu_tcp;  // Combine force and torque into a single vector

  return ft_tcp;
}

Eigen::VectorXd
MomentumObserver::getAccEstimate(const Eigen::VectorXd& q, const Eigen::VectorXd& qd, const Eigen::VectorXd& tau) const
{
  Eigen::MatrixXd B = _model->get_inertia_matrix(q), C = _model->get_coriolis(q, qd), g = _model->get_gravity(q);

  Eigen::VectorXd friction = _model->get_friction(qd);

  const Eigen::VectorXd& tau_ext = getTauEstimate();

  Eigen::VectorXd ddq = B.inverse() * (tau - C * qd - g - friction + tau_ext);

  return ddq;
}

void MomentumObserver::zeroMomentumObserver()
{
  _internal_r_sum = _K.inverse() * _r + _internal_r_sum;
}