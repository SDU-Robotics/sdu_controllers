#include <sdu_controllers/models/breeding_blanket_handling_robot_model.hpp>

using namespace Eigen;

namespace sdu_controllers::models
{
  BreedingBlanketHandlingRobotModel::BreedingBlanketHandlingRobotModel() : RobotModel()
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

    /* EMIL
     *a_ = {0.0, 0.0, 0.0, 0.0, 0.39, 0.0, 0.0};
    d_ = {-0.78, 0.0, 0.0, 0.0, 0.0, 3.05, 0.0};
    alpha_ = {0.0, 0.0, pi/2., 0.0, -pi/2., 0.0, -pi/2.};
    theta_ = {0.0, -pi/2., 0.0, 0.0, 0.0, 0.0, 0.0};*/

    /* PRIER standard
    a_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.3880, 0.0};
    d_ = {0.0, 0.0, 0.0, 0.0, 0.0, 4.5822, 0.0};
    alpha_ = {0.0, 0.0, pi/2.0, 0.0, -pi/2.0, 0.0, -pi/2.0};
    theta_ = {0.0, 0.0, 0.0, 0.0, 0.0, -pi/2.0, 0.0};*/

    /* BEATRIZ standard */
    a_ = {0.0, 0.0, 0.0, 0.0, 0.3880, 0.0, 0.0};
    d_ = {0.0, 0.0, 0.0, 0.0, 3.05, 0.0, 1.85};
    alpha_ = {0.0, -pi/2.0, 0.0, -pi/2.0, 0.0, -pi/2.0, 0.0};
    theta_ = {0.0, -pi/2.0, 0.0, 0.0, pi/2.0, 0.0, 0.0};

    /* BEATRIZ modified
    a_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.3880, 0.0};
    d_ = {-0.78, 0.0, 0.0, 0.0, 0.0, 3.05, 0.0};
    alpha_ = {0.0, 0.0, -pi/2.0, 0.0, -pi/2.0, 0.0, -pi/2.0};
    theta_ = {0.0, -pi/2.0, 0.0, 0.0, -pi/2.0, 0.0, 0.0}; */

    is_joint_revolute_ = {false, true, false, true, true, true, true};

    m_ = {2741.42545562447,
          10065.7834870542,
          7165.97697748845,
          2214.61694486676,
          3222.02650315727,
          1880.28170116907,
          2180.39160250515};

    std::vector<std::vector<std::vector<double>>> temp_inertia_;
    std::vector<std::vector<double>> temp_com_;
    // Eigen::Matrix<double, Eigen::Dynamic, 3> com_
    temp_com_ =  {  { -0.0477560614569301, 4.70907354838998, 0.77426341044415 },
                    { -0.000133480333530223, 3.0442060215219, 0.82067266190261 },
                    { -0.0513016142902631, -0.806572112320407, -0.000622937038291305 },
                    {-0.0118978752865194, 0.00517600339741509, 0.00186348383747914},
                    { 0.19447661364143, 1.32016616021247E-05, 1.05047186769377 },
                    { -5.24025267623074E-14, -2.66453525910038E-15, -0.550437618644357 },
                    {2.48523986279281E-07, -0.861651047912921, 0.0020062647167296}
                  };

    // ixx, ixy, ixz
    // iyx, iyy, iyz
    // izx, iyz, izz
    // 
    // iyx = ixy
    // izx = ixz
    // iyz = izz
    /* Inertia EMIL
    temp_inertia_ = { { { 454.417450064943, 13.2563365046526, -9.76636355809446E-14 },
                        { 13.2563365046526, 1633.01237541584, 6.52150054401012E-05 }, 
                        { -9.76636355809446E-14, 6.52150054401012E-05, 1221.2019949085 } },

                      { { 25868.9325513368, 0.0145719077628428, -1.81065376597345E-09 }, 
                        { 0.0145719077628428, 2100.17874688298, 30.2761307435739 }, 
                        { -1.81065376597345E-09, 30.2761307435739, 25546.0402997148 } },

                      { { 3314.76529365534, -82.5800398026532, -2.45919482072253 }, 
                        { -82.5800398026532, 2888.09444970789, -4.14474795528231 }, 
                        { -2.45919482072253, -4.14474795528231, 2872.07690431493 } },

                      { { 453.81351927542, -0.673337641055126, -3.92180837138767E-07 }, 
                        { -0.673337641055126, 556.749525054131, 4.44377031959428E-13 }, 
                        { -3.92180837138767E-07, 4.44377031959428E-13, 355.428908580776 } },

                      { { 2657.32246973207, -0.00701514649972523, -434.408895521609 }, 
                        { -0.00701514649972523, 2757.85304437171, -0.0513347842736837 }, 
                        { -434.408895521609, -0.0513347842736837, 352.896044035107 } },

                      { { 445.541412938169, -4.2632564145606E-14, 2.34656457898188E-11 }, 
                        { -4.2632564145606E-14, 503.92227149722, -1.53951950533609E-12 }, 
                        { 2.34656457898188E-11, -1.53951950533609E-12, 112.615781631324 } },

                      { { 226.058957958159, 6.73902142191413E-05, 0.000101471683038312 }, 
                        { 6.73902142191413E-05, 180.794332853491, 0.00328761661821927 }, 
                        { 0.000101471683038312, 0.00328761661821927, 293.916105539602 } },
                    };*/

    // all the inertial parameters above are taken from a URDF-file generated from the CAD model.

    /* Inertia BEATRIZ */
    temp_inertia_ = { { { 11760.27, -131.84, 20.76 },
                    { -131.84, 3082.45, -11.68},
                    { 20.76, -11.68, 13869.61 } },

                  { { 35367.60, -3.43, 0.05 },
                    { -3.43, 2163.11, -88.89 },
                    { 0.05, -88.89, 35066.12 } },

                  { { 3334.26, 74.84, 2.45 },
                    { 74.84, 2899.75, 4.15 },
                    { 2.45, 4.15, 2892.53 } },

                  { { 559.66, -2.85, -0.08 },
                    { -2.85, 557.41, 0.55 },
                    { -0.08, 0.55, 461.79 } },

                  { { 3240.15, -0.01, 0.06 },
                    { -0.01, 3126.08, -508.46 },
                    { 0.06, -508.46, 371.49 } },

                  { { 445.54, 0.0, 0.0 },
                    { 0.00, 503.92, 0.0 },
                    { 0.0, 0.0, 112.62 } },

                  { { 546.73, 0.0, 0.0 },
                    { 0.0, 181.31, -0.01 },
                    { 0.0, -0.01, 615.10 } },
                };
    
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
    MatrixXd I = MatrixXd::Identity(ROBOT_DOF, ROBOT_DOF);
    return I;
  }

  MatrixXd BreedingBlanketHandlingRobotModel::get_coriolis(const VectorXd& q, const VectorXd& qd)
  {
    MatrixXd I = MatrixXd::Identity(ROBOT_DOF, ROBOT_DOF);
    return I;
  }

  MatrixXd BreedingBlanketHandlingRobotModel::get_gravity(const VectorXd& q)
  {
    VectorXd v = VectorXd::Zero(ROBOT_DOF);
    return v;
  }

  MatrixXd BreedingBlanketHandlingRobotModel::get_jacobian(const VectorXd& q)
  {
    MatrixXd I = MatrixXd::Identity(ROBOT_DOF, ROBOT_DOF);
    return I;
  }

  MatrixXd BreedingBlanketHandlingRobotModel::get_jacobian_dot(const VectorXd& q, const VectorXd& dq)
  {
    MatrixXd I = MatrixXd::Identity(ROBOT_DOF, ROBOT_DOF);
    return I;
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
