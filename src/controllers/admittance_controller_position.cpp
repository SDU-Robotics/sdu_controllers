#include <sdu_controllers/controllers/admittance_controller_position.hpp>
#include <sdu_controllers/math/math.hpp>


namespace sdu_controllers::controllers
{
  AdmittanceControllerPosition::AdmittanceControllerPosition(const double frequency)
  {
    // Specification of impedance-parameters
    const Eigen::Vector3d M_vec{ 22.5, 22.5, 22.5 };  // Positional mass
    const Eigen::Vector3d K_vec{ 0, 0, 0 };           // Positional stiffness
    const Eigen::Vector3d D_vec{ 70, 70, 70 };        // Positional damping
    M_ = M_vec.asDiagonal();
    K_ = K_vec.asDiagonal();
    D_ = D_vec.asDiagonal();
    const Eigen::Vector3d Mo_vec{ 0.25, 0.25, 0.25 };  // Orientation mass
    const Eigen::Vector3d Ko_vec{ 0, 0, 0 };           // Orientation stiffness
    const Eigen::Vector3d Do_vec{ 3, 3, 3 };           // Orientation damping
    Mo_ = Mo_vec.asDiagonal();
    Ko_ = Ko_vec.asDiagonal();
    Do_ = Do_vec.asDiagonal();

    dt_ = 1. / frequency;                           // Time step

    rot_identity_ = Eigen::Matrix3d::Identity();

    AdmittanceControllerPosition::reset();
  }

  void AdmittanceControllerPosition::step(const Eigen::Vector3d &input_force, const Eigen::Vector3d &input_torque, const Eigen::Vector3d &x_desired, const Eigen::Quaterniond &quat_desired)
  {
    // -- Compute positional error --

    // Acceleration error
    Eigen::Vector3d ddx_e_ = M_.inverse() * (input_force - K_ * x_e_ - D_ * dx_e_);

    // Integrate velocity and position error
    dx_e_ += ddx_e_ * dt_;
    x_e_ += dx_e_ * dt_;

    // -- Compute rotational error --
    Eigen::Matrix3d E = (quat_e_.w() * rot_identity_) - math::skew(quat_e_.vec());
    Eigen::Matrix3d Ko_mark = 2 * E.transpose() * Ko_;

    // Compute angular acceleration error domega_e
    Eigen::Vector3d domega_e_ = Mo_.inverse() * (input_torque - Ko_mark * quat_e_.vec() - Do_ * omega_e_);

    // Integrate to angular velocity error omega_e
    omega_e_ += domega_e_ * dt_;

    // Integrate to quaternion error quat_e
    Eigen::Vector3d half_omega_e_dt = 0.5 * omega_e_ * dt_;
    Eigen::Quaterniond omega_quat = math::exp(Eigen::Quaterniond(0, half_omega_e_dt[0], half_omega_e_dt[1], half_omega_e_dt[2]));
    quat_e_ = omega_quat * quat_e_;

    // Calculate new position
    Eigen::Vector3d x_c_ = x_desired + x_e_;
    Eigen::Quaterniond quat_c_ = quat_desired * quat_e_;

    u_ << x_c_[0], x_c_[1], x_c_[2], quat_c_.w(), quat_c_.x(), quat_c_.y(), quat_c_.z();
  }

  void AdmittanceControllerPosition::reset()
  {
    dx_e_.setZero();
    x_e_.setZero();

    omega_e_.setZero();
    quat_e_.setIdentity();

    u_.setZero();
  }

  Eigen::VectorXd AdmittanceControllerPosition::get_output()
  {
    return u_;
  }

  void AdmittanceControllerPosition::set_mass_matrix_position(const Eigen::Matrix3d &mass)
  {
    M_ = mass;
  }

  void AdmittanceControllerPosition::set_mass_matrix_position(const Eigen::Vector3d &mass)
  {
    M_ = mass.asDiagonal();
  }

  void AdmittanceControllerPosition::set_stiffness_matrix_position(const Eigen::Matrix3d &stiffness)
  {
    K_ = stiffness;
  }

  void AdmittanceControllerPosition::set_stiffness_matrix_position(const Eigen::Vector3d &stiffness)
  {
    K_ = stiffness.asDiagonal();
  }

  void AdmittanceControllerPosition::set_damping_matrix_position(const Eigen::Matrix3d &damping)
  {
    D_ = damping;
  }

  void AdmittanceControllerPosition::set_damping_matrix_position(const Eigen::Vector3d &damping)
  {
    D_ = damping.asDiagonal();
  }

  void AdmittanceControllerPosition::set_mass_matrix_orientation(const Eigen::Matrix3d &mass)
  {
    Mo_ = mass;
  }

  void AdmittanceControllerPosition::set_mass_matrix_orientation(const Eigen::Vector3d &mass)
  {
    Mo_ = mass.asDiagonal();
  }

  void AdmittanceControllerPosition::set_stiffness_matrix_orientation(const Eigen::Matrix3d &stiffness)
  {
    Ko_ = stiffness;
  }

  void AdmittanceControllerPosition::set_stiffness_matrix_orientation(const Eigen::Vector3d &stiffness)
  {
    Ko_ = stiffness.asDiagonal();
  }

  void AdmittanceControllerPosition::set_damping_matrix_orientation(const Eigen::Matrix3d &damping)
  {
    Do_ = damping;
  }

  void AdmittanceControllerPosition::set_damping_matrix_orientation(const Eigen::Vector3d &damping)
  {
    Do_ = damping.asDiagonal();
  }

}  // namespace sdu_controllers::controllers
