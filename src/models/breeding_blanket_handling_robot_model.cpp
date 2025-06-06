#include <iostream>
#include <sdu_controllers/models/breeding_blanket_handling_robot_model.hpp>

#include "yaml-cpp/yaml.h"

using namespace Eigen;

namespace sdu_controllers::models
{
  BreedingBlanketHandlingRobotModel::BreedingBlanketHandlingRobotModel() : bb_robot_(), RobotModel()
  {
    YAML::Node config = YAML::LoadFile(std::string(CONFIG_FILE_PATH) +
      std::string("/models/breeding_blanket_handling_robot.yaml"));

    // std::cout << config["N"] << std::endl;

    VectorXd q_low(dof_);
    VectorXd q_high(dof_);
    VectorXd dq_low(dof_);
    VectorXd dq_high(dof_);
    VectorXd ddq_low(dof_);
    VectorXd ddq_high(dof_);
    VectorXd torque_low(dof_);
    VectorXd torque_high(dof_);

    constexpr double pi = 3.14159265358979323846;
    constexpr double pi_2 = 1.5707963267948966;

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
    // a_ = {0.0, 0.0, 0.0,0.0,0.388,0.0,0.0};
    // d_ = {-0.4450, 0.0, 0.93, 0.0, 3.3873, 0.0, 0.0};
    // alpha_ = {0.0, -(pi)/2.0, 0.0, -(pi)/2, 0.0, -(pi)/2,0.0};
    // theta_ = {-(pi)/2.0, 0.0, 0.0,0.0, -(pi)/2, 0.0, 0.0};

    a_ = config["kinematics"]["a"].as<std::vector<double>>();
    d_ = config["kinematics"]["d"].as<std::vector<double>>();
    alpha_ = config["kinematics"]["alpha"].as<std::vector<double>>();
    theta_ = config["kinematics"]["theta"].as<std::vector<double>>();
  
    std::vector<std::vector<std::vector<double>>> temp_inertia_;
    std::vector<std::vector<double>> temp_com_;
    // m_ = { 3.368400e+03, 10552, 5.283800e+03, 3.821706e+03, 4.481349e+03, 2.517583e+03, 1.640486e+03};
    m_.reserve(7);
    is_joint_revolute_.reserve(7);
    link_inertia_.reserve(7);

    int i = 0;

    for (YAML::const_iterator it = config["inertial"].begin(); it != config["inertial"].end(); ++it, ++i)
    {
      std::string key = it->first.as<std::string>();
      // std::cout <<  key << std::endl;
      // std::cout <<  config["inertial"][key]["type"] << std::endl;

      m_.push_back(config["inertial"][key]["mass"].as<double>());
      // std::cout << m_[i] << std::endl;

      std::vector<double> tmp = config["inertial"][key]["com"].as<std::vector<double>>();
      com_(i, Eigen::all) = Eigen::Vector3d::Map(tmp.data(), tmp.size());

      Eigen::Matrix3d I;

      double __ixx = config["inertial"][key]["inertia"]["ixx"].as<double>();
      double __iyy = config["inertial"][key]["inertia"]["iyy"].as<double>();
      double __izz = config["inertial"][key]["inertia"]["izz"].as<double>();
      double __ixy = config["inertial"][key]["inertia"]["ixy"].as<double>();
      double __ixz = config["inertial"][key]["inertia"]["ixz"].as<double>();
      double __iyz = config["inertial"][key]["inertia"]["iyz"].as<double>();

      I(0, 0) = __ixx;
      I(1, 1) = __iyy;
      I(2, 2) = __izz;

      I(0, 1) = __ixy;
      I(1, 0) = __ixy;

      I(0, 2) = __ixz;
      I(2, 0) = __ixy;

      I(1, 2) = __iyz;
      I(2, 1) = __iyz;

      link_inertia_.push_back(I);

      std::string jtype = config["inertial"][key]["type"].as<std::string>();
      if (jtype == "revolute")
      {
        is_joint_revolute_.push_back(true);
      }
      else
      {
        is_joint_revolute_.push_back(false);
      }
    }

    // Copy m_, com_, link_inertia_ into default vectors.
    m_default_ = m_;
    com_default_ = com_;
    link_inertia_default_ = link_inertia_;

    g << 0, 0, -9.82;

    // pass variables to bb_robot object
    bb_robot_.set_dh_params(a_[4], d_[0], d_[2], d_[4]);

    double * m_tmp;
    m_tmp = m_.data();
    bb_robot_.set_m(m_tmp);
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

  void BreedingBlanketHandlingRobotModel::set_tcp_mass(double &mass, Eigen::Vector3d &com, Eigen::Matrix3d inertia)
  {
    // self._m[5] = self._m_default[5] + mass
    // com_new = (numpy.asarray(self._com_default[5]) * self._m_default[5] + numpy.asarray(com) * mass)/self._m[5]
    // self._com[5] = com_new.tolist()
    // r = -numpy.asarray(self._com_default[5]) + com_new
    // offset_matrix_a = numpy.asarray([[r[1] ** 2 + r[2] ** 2, -r[0]*r[1],             -r[0]*r[2]],
    //                                  [-r[0]*r[1],             r[0] ** 2 + r[2] ** 2, -r[1]*r[2]],
    //                                  [-r[0]*r[2],            -r[1]*r[2],              r[0] ** 2 + r[1] ** 2]])
    // r = -numpy.asarray(com) + com_new
    // offset_matrix_b = numpy.asarray([[r[1] ** 2 + r[2] ** 2, -r[0]*r[1],             -r[0]*r[2]],
    //                                  [-r[0]*r[1],             r[0] ** 2 + r[2] ** 2, -r[1]*r[2]],
    //                                  [-r[0]*r[2],            -r[1]*r[2],              r[0] ** 2 + r[1] ** 2]])
    // new_inertia = numpy.asarray(self._inertia_default[5]) + self._m_default[5] * offset_matrix_a +\
    //               numpy.asarray(inertia) + mass * offset_matrix_b
    // self._inertia[5] = new_inertia.tolist()
    int N = get_dof();
    m_[N - 1] = m_default_[N - 1] + mass;

    Eigen::Vector3d com_default_N = com_default_(N - 1, Eigen::all);
    Eigen::Vector3d com_new = (com_default_N * m_default_[N - 1] + com * mass) / m_[N - 1];
    com_(N - 1, Eigen::all) = com_new;
    Eigen::Vector3d r;
    r << -com_default_N + com_new;

    Eigen::Matrix3d offset_matrix_a, offset_matrix_b;

    offset_matrix_a << pow(r(1), 2) + pow(r(2), 2),                  -r(0)*r(1),                  -r(0)*r(2),
                                        -r(0)*r(1), pow(r(0), 2) + pow(r(2), 2),                  -r(1)*r(2),
                                        -r(0)*r(2),                  -r(1)*r(2), pow(r(0), 2) + pow(r(1), 2);

    r = -com + com_new;
    offset_matrix_b << pow(r(1), 2) + pow(r(2), 2),                  -r(0)*r(1),                  -r(0)*r(2),
                                        -r(0)*r(1), pow(r(0), 2) + pow(r(2), 2),                  -r(1)*r(2),
                                        -r(0)*r(2),                  -r(1)*r(2), pow(r(0), 2) + pow(r(1), 2);

    Eigen::Matrix3d new_inertia = link_inertia_default_[N - 1] +
      m_default_[N - 1] * offset_matrix_a +
      inertia + mass * offset_matrix_b;

    link_inertia_[N - 1] << new_inertia;

    // set bb_robot variables
    double * m_tmp = &m_[0];
    bb_robot_.set_m(m_tmp);

    // com
    double com_tmp [ROBOT_DOF][3];

    for (size_t i = 0; i < ROBOT_DOF; ++i)
      for (size_t j = 0; j < 3; ++j)
        com_tmp[i][j] = com_(i, j);

    bb_robot_.set_com(com_tmp);

    // inertia
    double inertia_tmp [ROBOT_DOF][3][3];

    for (size_t i = 0; i < ROBOT_DOF; ++i)
      for (size_t j = 0; j < 3; ++j)
        for (size_t k = 0; k < 3; ++k)
          inertia_tmp[i][j][k] = link_inertia_[i](j, k);

    bb_robot_.set_link_inertia(inertia_tmp);
  }
  
} 
