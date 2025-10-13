#include <sdu_controllers/kinematics/dh_kinematics.hpp>

using namespace sdu_controllers;
using namespace sdu_controllers::kinematics;

DHKinematics::DHKinematics()
    : ForwardKinematics(std::vector<ForwardKinematics::JointType>())
{
}


DHKinematics::DHKinematics(const std::vector<DHParam>& dh_parameters)
    : ForwardKinematics(std::vector<ForwardKinematics::JointType>())
{
  if (dh_parameters.empty())
  {
    throw std::runtime_error("DHKinematics: No DH parameters provided");
  }

  for (const auto& param : dh_parameters)
  {
    a_.push_back(param.a);
    alpha_.push_back(param.alpha);
    d_.push_back(param.d);
    theta_.push_back(param.theta);
    if (param.is_joint_revolute)
      jointType_.push_back(ForwardKinematics::REVOLUTE);
    else
      jointType_.push_back(ForwardKinematics::PRISMATIC);
  }
}

DHKinematics::DHKinematics(
    const std::vector<double>& a,
    const std::vector<double>& alpha,
    const std::vector<double>& d,
    const std::vector<double>& theta,
    const std::vector<bool>& is_joint_revolute)
    : ForwardKinematics(is_joint_revolute)
{
  if (a.size() != alpha.size() || a.size() != d.size() || a.size() != theta.size() || a.size() != is_joint_revolute.size())
  {
    throw std::runtime_error("DHKinematics: DH parameter vectors must have the same size");
  }
  if (a.empty())
  {
    throw std::runtime_error("DHKinematics: No DH parameters provided");
  }

  a_ = a;
  alpha_ = alpha;
  d_ = d;
  theta_ = theta;
}

Eigen::Matrix4d DHKinematics::forward_kinematics(const Eigen::VectorXd& q) const
{
  if (q.size() != a_.size())
  {
    throw std::runtime_error("DHKinematics: Input joint vector has incorrect size");
  }
  bool first = true;
  Eigen::Matrix4d T;

  Eigen::MatrixXd A_i(4, 4);
  double qi;

  // calculate transform matrix for each link
  for (Eigen::Index i = 0; i < q.size(); i++)
  {
    double cos_alpha = std::cos(alpha_[i]);
    double sin_alpha = std::sin(alpha_[i]);

    if (jointType_[i] == ForwardKinematics::REVOLUTE)
    {  // revolute joint
      qi = q[i] + theta_[i];

      double cos_qi = std::cos(qi);
      double sin_qi = std::sin(qi);

      A_i(0, 0) = cos_qi;
      A_i(0, 1) = -sin_qi * cos_alpha;
      A_i(0, 2) = sin_qi * sin_alpha;
      A_i(0, 3) = a_[i] * cos_qi;

      A_i(1, 0) = sin_qi;
      A_i(1, 1) = cos_qi * cos_alpha;
      A_i(1, 2) = -cos_qi * sin_alpha;
      A_i(1, 3) = a_[i] * sin_qi;

      A_i(2, 0) = 0;
      A_i(2, 1) = sin_alpha;
      A_i(2, 2) = cos_alpha;
      A_i(2, 3) = d_[i];

      A_i(3, 0) = 0;
      A_i(3, 1) = 0;
      A_i(3, 2) = 0;
      A_i(3, 3) = 1;
    }
    else if (jointType_[i] == ForwardKinematics::PRISMATIC)
    {  // prismatic joint

      double cos_theta = std::cos(theta_[i]);
      double sin_theta = std::sin(theta_[i]);

      qi = q[i] + d_[i];
      A_i(0, 0) = cos_theta;
      A_i(0, 1) = -sin_theta * cos_alpha;
      A_i(0, 2) = sin_theta * sin_alpha;
      A_i(0, 3) = a_[i] * cos_theta;

      A_i(1, 0) = sin_theta;
      A_i(1, 1) = cos_theta * cos_alpha;
      A_i(1, 2) = -cos_theta * sin_alpha;
      A_i(1, 3) = a_[i] * sin_theta;

      A_i(2, 0) = 0;
      A_i(2, 1) = sin_alpha;
      A_i(2, 2) = cos_alpha;
      A_i(2, 3) = qi;

      A_i(3, 0) = 0;
      A_i(3, 1) = 0;
      A_i(3, 2) = 0;
      A_i(3, 3) = 1;
    }
    else
    {
      throw std::runtime_error("DHKinematics: Unknown joint type");
    }

    if (first)
    {
      T = A_i;
      first = false;
    }
    else
      T *= A_i;
  }

  return T;
}

std::vector<Eigen::Matrix4d> DHKinematics::forward_kinematics_all(const Eigen::VectorXd& q) const
{
  if (q.size() != a_.size())
  {
    throw std::runtime_error("DHKinematics: Input joint vector has incorrect size");
  }
  std::vector<Eigen::Matrix4d> complete_T;

  bool first = true;
  Eigen::Matrix4d T;

  Eigen::MatrixXd A_i(4, 4);
  double qi;

  // calculate transform matrix for each link
  for (Eigen::Index i = 0; i < q.size(); i++)
  {
    double cos_alpha = std::cos(alpha_[i]);
    double sin_alpha = std::sin(alpha_[i]);
    if (jointType_[i] == ForwardKinematics::REVOLUTE)
    {  // revolute joint
      qi = q[i] + theta_[i];

      double cos_qi = std::cos(qi);
      double sin_qi = std::sin(qi);

      A_i(0, 0) = cos_qi;
      A_i(0, 1) = -sin_qi * cos_alpha;
      A_i(0, 2) = sin_qi * sin_alpha;
      A_i(0, 3) = a_[i] * cos_qi;

      A_i(1, 0) = sin_qi;
      A_i(1, 1) = cos_qi * cos_alpha;
      A_i(1, 2) = -cos_qi * sin_alpha;
      A_i(1, 3) = a_[i] * sin_qi;

      A_i(2, 0) = 0;
      A_i(2, 1) = sin_alpha;
      A_i(2, 2) = cos_alpha;
      A_i(2, 3) = d_[i];

      A_i(3, 0) = 0;
      A_i(3, 1) = 0;
      A_i(3, 2) = 0;
      A_i(3, 3) = 1;
    }
    else if (jointType_[i] == ForwardKinematics::PRISMATIC)
    {  // prismatic joint

      double cos_theta = std::cos(theta_[i]);
      double sin_theta = std::sin(theta_[i]);

      qi = q[i] + d_[i];
      A_i(0, 0) = cos_theta;
      A_i(0, 1) = -sin_theta * cos_alpha;
      A_i(0, 2) = sin_theta * sin_alpha;
      A_i(0, 3) = a_[i] * cos_theta;

      A_i(1, 0) = sin_theta;
      A_i(1, 1) = cos_theta * cos_alpha;
      A_i(1, 2) = -cos_theta * sin_alpha;
      A_i(1, 3) = a_[i] * sin_theta;

      A_i(2, 0) = 0;
      A_i(2, 1) = sin_alpha;
      A_i(2, 2) = cos_alpha;
      A_i(2, 3) = qi;

      A_i(3, 0) = 0;
      A_i(3, 1) = 0;
      A_i(3, 2) = 0;
      A_i(3, 3) = 1;
    }
    else
    {
      throw std::runtime_error("DHKinematics: Unknown joint type");
    }

    if (first)
    {
      T = A_i;
      first = false;
    }
    else
      T *= A_i;

    complete_T.push_back(T);
  }

  return complete_T;
}

std::vector<double> DHKinematics::get_a() const
{
  return a_;
}

std::vector<double> DHKinematics::get_alpha() const
{
  return alpha_;
}

std::vector<double> DHKinematics::get_d() const
{
  return d_;
}

std::vector<double> DHKinematics::get_theta() const
{
  return theta_;
}