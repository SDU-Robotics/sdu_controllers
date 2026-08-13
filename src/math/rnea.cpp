#include <Eigen/Geometry>
#include <sdu_controllers/kinematics/forward_kinematics.hpp>
#include <sdu_controllers/math/rnea.hpp>

namespace Eigen {
  namespace indexing {

  }
  using Eigen::indexing::all;
}

namespace sdu_controllers::math
{
  RecursiveNewtonEuler::RecursiveNewtonEuler(models::RobotModel &robot_model) : robot_model_(robot_model)
  {
    const int N_links = static_cast<int>(robot_model_.get_fk_solver().get_num_links());
    const int N_dof   = static_cast<int>(robot_model_.get_dof());
    omega_ = Eigen::MatrixXd::Zero(3, N_links);
    domega_ = Eigen::MatrixXd::Zero(3, N_links);
    ddp_ = Eigen::MatrixXd::Zero(3, N_links);
    ddpc_ = Eigen::MatrixXd::Zero(3, N_links);
    tau_ = Eigen::VectorXd::Zero(N_dof);
    f_ = Eigen::MatrixXd::Zero(3, N_links);
    mu_ = Eigen::MatrixXd::Zero(3, N_links);
    ddp0_ = -robot_model_.get_g0();
    omega0_ = Eigen::MatrixXd::Zero(3, 1);
    domega0_ = Eigen::MatrixXd::Zero(3, 1);
    CoM_ = robot_model_.get_CoM();
    link_inertia_ = robot_model_.get_link_inertia();
    z0_ << 0, 0, 1;
  }

  void RecursiveNewtonEuler::set_z0(const Eigen::Vector3d &z0)
  {
    this->z0_ = z0;
  }

  Eigen::VectorXd RecursiveNewtonEuler::forward_dynamics(
      const Eigen::VectorXd &q,
      const Eigen::VectorXd &dq,
      const Eigen::VectorXd &tau)
  {
    Eigen::VectorXd zero_vec = Eigen::VectorXd::Zero(q.rows());
    // Get C(q, dq) dq + g(q)
    Eigen::VectorXd tau_bar = inverse_dynamics(q, dq, zero_vec, zero_vec);
    // Eigen::VectorXd tau_bar = velocityProduct(q, dq) + gravity(q);

    Eigen::MatrixXd B = inertia(q);

    // return B.inverse() * (tau - tau_bar);
    return B.lu().solve(tau - tau_bar);
  }

  Eigen::MatrixXd RecursiveNewtonEuler::inertia(const Eigen::VectorXd &q)
  {
    Eigen::VectorXd zero_vec = Eigen::VectorXd::Zero(q.rows()), he0 = Eigen::VectorXd::Zero(6),
                    ddq_bar = Eigen::VectorXd::Zero(q.rows()), tau_tmp = Eigen::VectorXd::Zero(q.rows());

    Eigen::MatrixXd B = Eigen::MatrixXd::Zero(q.rows(), q.rows());

    Eigen::Vector3d ddp0_original = this->ddp0_;
    this->ddp0_ *= 0;

    for (int i = 0; i < q.rows(); ++i)
    {
      ddq_bar.setZero();
      ddq_bar(i) = 1.0;
      tau_tmp = rigid_body_inverse_dynamics(q, zero_vec, ddq_bar, he0);
      B(Eigen::all, i) = tau_tmp;
    }

    this->ddp0_ = ddp0_original;

    return B;
  }

  Eigen::MatrixXd RecursiveNewtonEuler::coriolis(const Eigen::VectorXd &q, const Eigen::VectorXd &dq)
  {
    Eigen::VectorXd ddq0 = Eigen::VectorXd::Zero(q.rows()),
                    he0 = Eigen::VectorXd::Zero(6);

    // Set gravity to zero
    Eigen::Vector3d ddp0_original = this->ddp0_;
    this->ddp0_ *= 0;

    // Based on implementation found in Peter Corke's robotics toolbox
    // https://github.com/petercorke/robotics-toolbox-python/blob/master/roboticstoolbox/robot/Dynamics.py#L768

    // Initialise C and Csq
    Eigen::MatrixXd C = Eigen::MatrixXd::Zero(q.rows(), q.rows());
    Eigen::MatrixXd Csq = Eigen::MatrixXd::Zero(q.rows(), q.rows());

    // Find torques depending on a single joint speed due to the centripetal terms.
    Eigen::VectorXd QD = Eigen::VectorXd::Zero(q.rows());
    Eigen::VectorXd tau;

    for (int i = 0; i < q.rows(); ++i)
    {
      QD = Eigen::VectorXd::Zero(q.rows());
      QD(i) = 1.;

      tau = rigid_body_inverse_dynamics(q, QD, ddq0, he0);

      Csq(Eigen::all, i) = tau;
    }

    // Find torques depending on a pair of speeds due to the coriolis terms.
    for (int i = 0; i < q.rows(); ++i)
    {
      for (int j = i + 1; j < q.rows(); ++j)
      {
        QD = Eigen::VectorXd::Zero(q.rows());
        QD(i) = 1.;
        QD(j) = 1.;

        tau = rigid_body_inverse_dynamics(q, QD, ddq0, he0);

        C(Eigen::all, j) = C(Eigen::all, j) + (tau - Csq(Eigen::all, j) - Csq(Eigen::all, i)) * dq(i) / 2.;
        C(Eigen::all, i) = C(Eigen::all, i) + (tau - Csq(Eigen::all, j) - Csq(Eigen::all, i)) * dq(j) / 2.;
      }
    }

    C = C + Csq * dq.asDiagonal();

    // Restore gravity
    this->ddp0_ = ddp0_original;

    return C;
  }

  Eigen::VectorXd RecursiveNewtonEuler::velocity_product(const Eigen::VectorXd &q, const Eigen::VectorXd &dq)
  {
    Eigen::VectorXd zero_vec = Eigen::VectorXd::Zero(q.rows()), he0 = Eigen::VectorXd::Zero(6);
    Eigen::MatrixXd Cdq = Eigen::MatrixXd::Zero(q.rows(), q.rows());

    Eigen::Vector3d ddp0_original = this->ddp0_;
    this->ddp0_ *= 0;

    Cdq = rigid_body_inverse_dynamics(q, dq, zero_vec, zero_vec);

    this->ddp0_ = ddp0_original;

    return Cdq;
  }

  Eigen::VectorXd RecursiveNewtonEuler::gravity(const Eigen::VectorXd &q)
  {
    Eigen::VectorXd zero_vec = Eigen::VectorXd::Zero(q.rows()), he0 = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd grav = Eigen::VectorXd::Zero(q.rows());

    grav = rigid_body_inverse_dynamics(q, zero_vec, zero_vec, he0);
    return grav;
  }

  Eigen::VectorXd RecursiveNewtonEuler::inverse_dynamics(
      const Eigen::VectorXd &q,
      const Eigen::VectorXd &dq,
      const Eigen::VectorXd &ddq,
      const Eigen::VectorXd &he)
  {
    // Rigid-body dynamics plus joint friction
    return rigid_body_inverse_dynamics(q, dq, ddq, he) + friction(dq);
  }

  Eigen::VectorXd RecursiveNewtonEuler::friction(const Eigen::VectorXd &dq)
  {
    return robot_model_.get_friction(dq);
  }

  Eigen::VectorXd RecursiveNewtonEuler::rigid_body_inverse_dynamics(
      const Eigen::VectorXd &q,
      const Eigen::VectorXd &dq,
      const Eigen::VectorXd &ddq,
      const Eigen::VectorXd &he)
  {
    std::vector<Eigen::Matrix4d> T = robot_model_.get_fk_solver().forward_kinematics_all(q);
    forward(dq, ddq, T);
    backward(he, T);
    return tau_;
  }

  void
  RecursiveNewtonEuler::forward(const Eigen::VectorXd &dq, const Eigen::VectorXd &ddq, const std::vector<Eigen::Matrix4d> T)
  {
    Eigen::Vector3d r_, Rci, zi;

    std::vector<kinematics::ForwardKinematics::JointType> joint_type = robot_model_.get_fk_solver().get_joint_types();

    int q_idx = 0;  // index into actuated joint variables (dq, ddq)
    for (int i = 0; i < static_cast<int>(robot_model_.get_fk_solver().get_num_links()); ++i)
    {
      if (i > 0)
      {
        r_ = T[i](Eigen::seqN(0, 3), 3) - T[i - 1](Eigen::seqN(0, 3), 3);
      }
      else
      {
        r_ = T[i](Eigen::seqN(0, 3), 3);
      }

      Rci = T[i].block<3, 3>(0, 0) * CoM_(i, Eigen::all).transpose();

      // Assign the appropriate kinematic quantities for each joint type.
      // These formulas are from Spong.
      if (joint_type.at(i) == kinematics::ForwardKinematics::JointType::FIXED)
      {  // fixed joint: kinematics pass through, no velocity/acceleration contribution
        if (i == 0)
        {
          omega_(Eigen::all, i) = omega0_;
          domega_(Eigen::all, i) = domega0_;
          ddp_(Eigen::all, i) = ddp0_ + domega0_.cross(r_) + omega0_.cross(omega0_.cross(r_));
        }
        else
        {
          omega_(Eigen::all, i) = omega_(Eigen::all, i - 1);
          domega_(Eigen::all, i) = domega_(Eigen::all, i - 1);
          ddp_(Eigen::all, i) = ddp_(Eigen::all, i - 1) + domega_(Eigen::all, i).cross(r_) +
                               omega_(Eigen::all, i).cross(omega_(Eigen::all, i).cross(r_));
        }
        ++q_idx;
      }
      else if (joint_type.at(i) == kinematics::ForwardKinematics::JointType::REVOLUTE)
      {  // revolute joint
        if (i == 0)
        {
          omega_(Eigen::all, i) = omega0_ + dq(q_idx) * z0_;
          domega_(Eigen::all, i) = domega0_ + ddq(q_idx) * z0_ + dq(q_idx) * omega0_.cross(z0_);
          ddp_(Eigen::all, i) = ddp0_ + domega0_.cross(r_) + omega0_.cross(omega0_.cross(r_));
        }
        else
        {
          zi = T[i - 1](Eigen::seqN(0, 3), 2);
          omega_(Eigen::all, i) = omega_(Eigen::all, i - 1) + dq(q_idx) * zi;
          domega_(Eigen::all, i) = domega_(Eigen::all, i - 1) + ddq(q_idx) * zi + dq(q_idx) * omega_(Eigen::all, i - 1).cross(zi);
          ddp_(Eigen::all, i) = ddp_(Eigen::all, i - 1) + domega_(Eigen::all, i).cross(r_) +
                               omega_(Eigen::all, i).cross(omega_(Eigen::all, i).cross(r_));
        }
        ++q_idx;
      }
      else if (joint_type.at(i) == kinematics::ForwardKinematics::JointType::PRISMATIC)
      {  // prismatic joint
        if (i == 0)
        {
          omega_(Eigen::all, i) = omega0_;
          domega_(Eigen::all, i) = domega0_;
          ddp_(Eigen::all, i) =
              ddp0_ + ddq(q_idx) * z0_ + 2 * dq(q_idx) * omega0_.cross(z0_) + domega0_.cross(r_) + omega0_.cross(omega0_.cross(r_));
        }
        else
        {
          zi = T[i - 1](Eigen::seqN(0, 3), 2);
          omega_(Eigen::all, i) = omega_(Eigen::all, i - 1);
          domega_(Eigen::all, i) = domega_(Eigen::all, i - 1);
          ddp_(Eigen::all, i) = ddp_(Eigen::all, i - 1) + ddq(q_idx) * zi + 2 * dq(q_idx) * omega_(Eigen::all, i).cross(zi) +
                               domega_(Eigen::all, i).cross(r_) + omega_(Eigen::all, i).cross(omega_(Eigen::all, i).cross(r_));
        }
        ++q_idx;
      }
      else
      {
        throw std::runtime_error("Unknown joint type");
      }

      ddpc_(Eigen::all, i) = ddp_(Eigen::all, i) + domega_(Eigen::all, i).cross(Rci) +
                            omega_(Eigen::all, i).cross(omega_(Eigen::all, i).cross(Rci));
    }
  }

  void RecursiveNewtonEuler::backward(const Eigen::VectorXd &he, const std::vector<Eigen::Matrix4d> T)
  {
    Eigen::Vector3d r_, Rci, zi;
    Eigen::Matrix3d R = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d Ibase = Eigen::Matrix3d::Zero();
    std::vector<double> m = robot_model_.get_m();

    std::vector<kinematics::ForwardKinematics::JointType> joint_type = robot_model_.get_fk_solver().get_joint_types();
    const int N_links = static_cast<int>(robot_model_.get_fk_solver().get_num_links());

    // Count actuated (non-fixed) joints descending so we can index tau_ correctly
    int q_idx = static_cast<int>(robot_model_.get_dof()) - 1;

    for (int i = N_links - 1; i >= 0; --i)
    {
      if (i > 0)
      {
        r_ = T[i](Eigen::seqN(0, 3), 3) - T[i - 1](Eigen::seqN(0, 3), 3);
      }
      else
      {
        r_ = T[i](Eigen::seqN(0, 3), 3);
      }

      R = T[i].block<3, 3>(0, 0);
      Rci = R * CoM_(i, Eigen::all).transpose();

      Ibase << R * link_inertia_[i] * R.transpose();
      if (i == (N_links - 1))
      {
        Eigen::Vector3d f_e = he(Eigen::seqN(0, 3));
        Eigen::Vector3d mu_e = he(Eigen::seqN(3, 3));

        f_(Eigen::all, i) = f_e + m[i] * ddpc_(Eigen::all, i);
        mu_(Eigen::all, i) = -f_(Eigen::all, i).cross(r_ + Rci) + mu_e + f_e.cross(Rci) + Ibase * domega_(Eigen::all, i) +
                            omega_(Eigen::all, i).cross(Ibase * omega_(Eigen::all, i));
      }
      else
      {
        f_(Eigen::all, i) = f_(Eigen::all, i + 1) + m[i] * ddpc_(Eigen::all, i);
        mu_(Eigen::all, i) = -f_(Eigen::all, i).cross(r_ + Rci) + mu_(Eigen::all, i + 1) + f_(Eigen::all, i + 1).cross(Rci) +
                            Ibase * domega_(Eigen::all, i) + omega_(Eigen::all, i).cross(Ibase * omega_(Eigen::all, i));
      }

      if (joint_type.at(i) == kinematics::ForwardKinematics::JointType::FIXED)
      {
        // Fixed joints carry no torque output, skip tau assignment
        tau_(q_idx) = 0;

        --q_idx;
      }
      
      else if (joint_type.at(i) == kinematics::ForwardKinematics::JointType::REVOLUTE)
      {  // Revolute
        if (i == 0)
        {
          tau_(q_idx) = mu_(Eigen::all, i).transpose() * z0_;
        }
        else
        {
          zi = T[i - 1](Eigen::seqN(0, 3), 2);
          tau_(q_idx) = mu_(Eigen::all, i).transpose() * zi;
        }
        --q_idx;
      }
      else if (joint_type.at(i) == kinematics::ForwardKinematics::JointType::PRISMATIC)
      {  // Prismatic
        if (i == 0)
        {
          tau_(q_idx) = f_(Eigen::all, i).transpose() * z0_;
        }
        else
        {
          zi = T[i - 1](Eigen::seqN(0, 3), 2);
          tau_(q_idx) = f_(Eigen::all, i).transpose() * zi;
        }
        --q_idx;
      }
      else
      {
        throw std::runtime_error("Unknown joint type");
      }
    }
  }
}  // namespace sdu_controllers::math
