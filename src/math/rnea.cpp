#include "sdu_controllers/math/rnea.hpp"
#include <sdu_controllers/kinematics/forward_kinematics.hpp>

#include <Eigen/Geometry>

namespace sdu_controllers::math
{
  RecursiveNewtonEuler::RecursiveNewtonEuler(std::shared_ptr<models::RobotModel> robot_model)
    : robot_model(robot_model)
  {
    const int N = robot_model->get_dof();
    omega = Eigen::MatrixXd::Zero(3, N);
    domega = Eigen::MatrixXd::Zero(3, N);
    ddp = Eigen::MatrixXd::Zero(3, N);
    ddpc = Eigen::MatrixXd::Zero(3, N);
    tau = Eigen::VectorXd::Zero(N);
    f = Eigen::MatrixXd::Zero(3, N);
    mu = Eigen::MatrixXd::Zero(3, N);
    ddp0 = -robot_model->get_g0();
    CoM = robot_model->get_CoM();
    link_inertia = robot_model->get_link_inertia();
  }

  Eigen::VectorXd RecursiveNewtonEuler::inverse_dynamics(const Eigen::VectorXd &q, const Eigen::VectorXd &dq,
        const Eigen::VectorXd &ddq, const Eigen::VectorXd &he)
  {
    std::vector<Eigen::Matrix4d> T = kinematics::forward_kinematics_all(q, robot_model);
    forward(dq, ddq, T);
    backward(he, T);
    return tau;
  }

  void RecursiveNewtonEuler::forward(const Eigen::VectorXd &dq, const Eigen::VectorXd &ddq, const std::vector<Eigen::Matrix4d> T)
  {
    Eigen::Vector3d r_, Rci, zi, z0;
    z0 << 0, 0, 1;

    for (int i = 0; i < robot_model->get_dof(); ++i)
    {
      if (i > 0)
        r_ = T[i](Eigen::seqN(0, 3), 3) - T[i - 1](Eigen::seqN(0, 3), 3);
      else
        r_ = T[i](Eigen::seqN(0, 3), 3);

      Rci = T[i].block<3, 3>(0, 0) * CoM(i, Eigen::all).transpose();

      if (robot_model->get_is_joint_revolute().at(i))
      { // revolute joint
        if (i == 0)
        {
          omega(Eigen::all, i) = omega0 + dq(i) * z0;
          domega(Eigen::all, i) = domega0 + ddq(i) * z0 +
            dq(i) * omega0.cross(z0);
          ddp(Eigen::all, i) = ddp0 + domega0.cross(r_) +
              omega(Eigen::all, i).cross(
                omega(Eigen::all, i).cross(r_));
        }
        else
        {
          zi = T[i - 1].block<3, 1>(0, 2);

          // omega(:, i) = omega(:, i - 1) + dq(i) * zi;
          // domega(:, i) = domega(:, i - 1) + ddq(i) * zi + ...
          //     cross(dq(i) * omega(:, i - 1), zi);
          // ddp(:, i) = ddp(:, i - 1) + cross(domega(:, i), r_) + ...
          //     cross(omega(:, i), cross(omega(:, i), r_));
          omega(Eigen::all, i) = omega(Eigen::all, i - 1) + dq(i) * zi;
          domega(Eigen::all, i) = domega(Eigen::all, i - 1) + ddq(i) * zi +
            dq(i) * omega(Eigen::all, i - 1).cross(zi);
          ddp(Eigen::all, i) = ddp(Eigen::all, i - 1) +
            domega(Eigen::all, i).cross(r_) +
              omega(Eigen::all, i).cross(
                omega(Eigen::all, i).cross(r_));
        }
      }
      else
      { // prismatic joint
        if (i == 0)
        {
          omega(Eigen::all, i) = omega0;
          domega(Eigen::all, i) = domega0;
          ddp(Eigen::all, i) = ddp0 + ddq(i) * z0 +
            2 * dq(i) * omega0.cross(z0) + domega0.cross(r_) + omega0.cross(omega0.cross(r_));

        }
        else
        {
          zi = T[i - 1].block<3, 1>(0, 2);

          // omega(:, i) = omega(:, i - 1);
          // domega(:, i) = domega(:, i - 1);
          // ddp(:, i) = ddp(:, i - 1) + ddq(i) * zi +  ...
          //     2 * dq(i) * cross(omega(:, i), zi) + ...
          //     cross(domega(:, i), r_) + ...
          //     cross(omega(:, i), cross(omega(:, i), r_));
          omega(Eigen::all, i) = omega(Eigen::all, i - 1);
          domega(Eigen::all, i) = domega(Eigen::all, i - 1);
          ddp(Eigen::all, i) = ddp(Eigen::all, i - 1) + ddq(i) * zi +
            2 * dq(i) * omega(Eigen::all, i).cross(zi) +
              domega(Eigen::all, i).cross(r_) +
                omega(Eigen::all, i).cross(omega(Eigen::all, i).cross(r_));
        }
      }

      ddpc(Eigen::all, i) = ddp(Eigen::all, i) + domega(Eigen::all, i).cross(Rci) +
        omega(Eigen::all, i).cross(
          omega(Eigen::all, i).cross(Rci)
        );
    }
  }

  void RecursiveNewtonEuler::backward(const Eigen::VectorXd &he, const std::vector<Eigen::Matrix4d> T)
  {
    Eigen::Vector3d r_, Rci, zi, z0;
    Eigen::Matrix3d R;
    Eigen::Matrix3d Ibase;
    std::vector<double> m = robot_model->get_m();

    for (int i = robot_model->get_dof() - 1; i >= 0; --i)
    {
      if (i > 0)
        r_ = T[i](Eigen::seqN(0, 3), 3) - T[i - 1](Eigen::seqN(0, 3), 3);
      else
        r_ = T[i](Eigen::seqN(0, 3), 3);

      Rci = T[i].block<3, 3>(0, 0) * CoM(i, Eigen::all).transpose();

      R = T[i].block<3, 3>(0, 0);
      Ibase << R * link_inertia[i] * R.transpose();

      if (i == robot_model->get_dof() - 1)
      {
        Eigen::Vector3d f_e = he(Eigen::seqN(0, 3));
        Eigen::Vector3d mu_e = he(Eigen::seqN(3, 3));

        f(Eigen::all, i) = f_e + m[i] * ddpc(Eigen::all, i);
        mu(Eigen::all, i) = -f(Eigen::all, i).cross(r_ + Rci) +
          mu_e + f_e.cross(Rci) +
            Ibase * domega(Eigen::all, i) +
              omega(Eigen::all, i).cross(Ibase * omega(Eigen::all, i));
      }
      else
      {
        f(Eigen::all, i) = f(Eigen::all, i + 1) + m[i] * ddpc(Eigen::all, i);
        mu(Eigen::all, i) = -f(Eigen::all, i).cross(r_ + Rci) +
          mu(Eigen::all, i + 1) + f(Eigen::all, i + 1).cross(Rci) +
            Ibase * domega(Eigen::all, i) +
              omega(Eigen::all, i).cross(Ibase * omega(Eigen::all, i));
      }

      if (robot_model->get_is_joint_revolute().at(i))
      { // Revolute
        if (i == 0)
        {
          tau(i) = mu(Eigen::all, i).transpose() * z0;
        }
        else
        {
          zi = T[i - 1].block<3, 1>(0, 2);
          tau(i) = mu(Eigen::all, i).transpose() * zi;
        }
      }
      else
      { // Prismatic
        if (i == 0)
        {
          tau(i) = f(Eigen::all, i).transpose() * z0;
        }
        else
        {
          zi = T[i - 1].block<3, 1>(0, 2);
          tau(i) = f(Eigen::all, i).transpose() * zi;
        }
      }
    }
  }
}