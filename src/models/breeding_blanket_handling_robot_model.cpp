#include <sdu_controllers/models/breeding_blanket_handling_robot_model.hpp>

using namespace Eigen;

namespace sdu_controllers::models
{
  BreedingBlanketHandlingRobotModel::BreedingBlanketHandlingRobotModel() : bb_robot_(), RobotModel()
  {
    VectorXd q_low(dof_);
    VectorXd q_high(dof_);
    VectorXd dq_low(dof_);
    VectorXd dq_high(dof_);
    VectorXd ddq_low(dof_);
    VectorXd ddq_high(dof_);
    VectorXd torque_low(dof_);
    VectorXd torque_high(dof_);

    constexpr double pi = 3.14159265358979323846;

    q_low << -25, -pi/2, 0.935, -2 * pi, -2 * pi, -2 * pi, -5;
    q_high << 8, pi/2, 4.66, 2 * pi, 2 * pi, 2 * pi, 5;
    dq_low << -pi, -pi, -pi, -pi, -pi, -pi, -pi;
    dq_high << pi, pi, pi, pi, pi, pi, pi;
    ddq_low << -40.0, -40.0, -40.0, -40.0, -40.0, -40.0, -40.0;
    ddq_high << 40.0, 40.0, 40.0, 40.0, 40.0, 40.0, 40.0;
    torque_low << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    torque_high << 150.0, 150.0, 150.0, 28.0, 28.0, 28.0, 28.0;

    joint_pos_bounds_ = { q_low, q_high};
    joint_vel_bounds_ = { dq_low, dq_high};
    joint_acc_bounds_ = { ddq_low, ddq_high};
    joint_torque_bounds_ = {torque_low, torque_high};

    /* BEATRIZ standard */
    a_ = {0.0, 0.0, 0.0,0.0,0.388,0.0,0.0};
    d_ = {-0.4450, 0.0, 0.93, 0.0, 3.3873, 0.0, 0.0};
    alpha_ = {0.0, -(pi)/2.0, 0.0, -(pi)/2, 0.0, -(pi)/2,0.0};
    theta_ = {-(pi)/2.0, 0.0, 0.0,0.0, -(pi)/2, 0.0, 0.0};


    is_joint_revolute_ = {false, true, false, true, true, true, true};
  
    std::vector<std::vector<std::vector<double>>> temp_inertia_;
    std::vector<std::vector<double>> temp_com_;
    m_ = { 3.368400e+03, 10552, 5.283800e+03, 3.821706e+03, 4.481349e+03, 2.517583e+03, 1.640486e+03};
    temp_com_ = { { -3.724500e-08, 4.776900e+00, 6.052400e-01, }, 
    {-1.341100e-13, -5.751800e-01, 3.108600e+00, }, 
    {-1.364100e-02, -4.624800e-01, -9.510100e-04, }, 
    {5.329071e-15, -3.730349e-14, 3.735431e-01, }, 
    {-2.302816e-01, 6.403606e-03, -2.012886e+00, }, 
    {6.014917e-09, 6.908411e-01, 3.829872e-09, }, 
    {-3.069726e-07, -6.430674e-01, -5.370716e-07, }}; 
    temp_inertia_ = { { { 4.900500e+02, -6.832200e-07, -2.052600e-07, }, 
     { -6.832200e-07, 1.677100e+03, -4.885600e-01, }, 
     { -2.052600e-07, -4.885600e-01, 1.227300e+03, }}, 
      
     {{ 19753, 1.472600e-09, 5.050000e-01, }, 
     { 1.472600e-09, 19645, 9.987400e-11, }, 
     { 5.050000e-01, 9.987400e-11, 2.298700e+03, }}, 
      
     {{ 24429, -3.165800e+00, 6.854400e-02, }, 
     { -3.165800e+00, 2.207700e+03, 1.259700e+00, }, 
     { 6.854400e-02, 1.259700e+00, 23579, }}, 
      
     {{ 8.803301e+02, -4.831691e-13, 2.949622e-11, }, 
     { -4.831691e-13, 5.638622e+02, -2.577024e-12, }, 
     { 2.949622e-11, -2.577024e-12, 9.672435e+02, }}, 
      
     {{ 2.332060e+03, 1.101176e-08, 2.420844e+01, }, 
     { 1.101176e-08, 2.327892e+03, -1.364343e-01, }, 
     { 2.420844e+01, -1.364343e-01, 1.999736e+02, }}, 
      
     {{ 9.459399e+02, 7.135083e-06, -4.799660e-06, }, 
     { 7.135083e-06, 1.358611e+02, -3.506231e-07, }, 
     { -4.799660e-06, -3.506231e-07, 1.014328e+03, }}, 
     
     {{ 2.070205e+02, 6.599078e-05, -1.354377e-05, }, 
     { 6.599078e-05, 1.491122e+02, 3.501925e-05, }, 
     { -1.354377e-05, 3.501925e-05, 2.551460e+02, }},}; 
    
    // Eigen::Matrix<double, Eigen::Dynamic, 3>
    com_ = Eigen::Matrix<double, Eigen::Dynamic, 3>::Zero(dof_, 3);
    for (size_t i = 0; i < dof_; i++)
    {
      for (size_t j = 0; j < 3; j++)
      {
        com_(i, j) = temp_com_[i][j];
      }
    }

    //  std::vector<Eigen::Matrix3d>
    for (size_t i = 0; i < dof_; i++)
    {
      link_inertia_.push_back(Eigen::Matrix3d::Zero());
      for (size_t j = 0; j < 3; j++)
      {
        for (size_t k = 0; k < 3; k++)
        {
          link_inertia_[i](j, k) = temp_inertia_[i][j][k];
        }
      }
    }

    g << 0, 0, -9.82;
  }

  MatrixXd BreedingBlanketHandlingRobotModel::get_inertia_matrix(const VectorXd& q)
  {
    return bb_robot_.inertia(q);
  }

  MatrixXd BreedingBlanketHandlingRobotModel::get_coriolis(const VectorXd& q, const VectorXd& qd)
  {
    return bb_robot_.coriolis(q, qd);
  }

  MatrixXd BreedingBlanketHandlingRobotModel::get_gravity(const VectorXd& q)
  {
    return bb_robot_.gravity(q);
  }

  MatrixXd BreedingBlanketHandlingRobotModel::get_jacobian(const VectorXd& q)
  {
    return bb_robot_.jacobian(q);
  }

  MatrixXd BreedingBlanketHandlingRobotModel::get_jacobian_dot(const VectorXd& q, const VectorXd& dq)
  {
    return bb_robot_.jacobianDot(q, dq);
  }

  std::pair<VectorXd, VectorXd> BreedingBlanketHandlingRobotModel::get_joint_pos_bounds()
  {
    return joint_pos_bounds_;
  }

  std::pair<VectorXd, VectorXd> BreedingBlanketHandlingRobotModel::get_joint_vel_bounds()
  {
    return joint_vel_bounds_;
  }

  std::pair<VectorXd, VectorXd> BreedingBlanketHandlingRobotModel::get_joint_acc_bounds()
  {
    return joint_acc_bounds_;
  }

  std::pair<VectorXd, VectorXd> BreedingBlanketHandlingRobotModel::get_joint_torque_bounds()
  {
    return joint_torque_bounds_;
  }

  uint16_t BreedingBlanketHandlingRobotModel::get_dof() const
  {
    return dof_;
  }

  std::vector<double> BreedingBlanketHandlingRobotModel::get_a()
  {
    return a_;
  }
  std::vector<double> BreedingBlanketHandlingRobotModel::get_d()
  {
    return d_;
  }
  std::vector<double> BreedingBlanketHandlingRobotModel::get_alpha()
  {
    return alpha_;
  }
  std::vector<double> BreedingBlanketHandlingRobotModel::get_theta()
  {
    return theta_;
  }
  std::vector<double> BreedingBlanketHandlingRobotModel::get_m()
  {
    return m_;
  }
  Eigen::Vector3d BreedingBlanketHandlingRobotModel::get_g0()
  {
    return g;
  }
  Eigen::Matrix<double, Eigen::Dynamic, 3> BreedingBlanketHandlingRobotModel::get_CoM()
  {
    return com_;
  }
  std::vector<Eigen::Matrix3d> BreedingBlanketHandlingRobotModel::get_link_inertia()
  {
    return link_inertia_;
  }
  std::vector<bool> BreedingBlanketHandlingRobotModel::get_is_joint_revolute() 
  {
    return is_joint_revolute_;
  }

}  // namespace sdu_controllers::math
