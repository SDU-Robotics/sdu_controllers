#include <yaml-cpp/yaml.h>

#include <memory>
#include <unsupported/Eigen/CXX11/Tensor>
#include <fstream>
#include <iostream>
#include <sdu_controllers/kinematics/dh_kinematics.hpp>
#include <sdu_controllers/kinematics/forward_kinematics.hpp>
#include <sdu_controllers/models/parameter_robot_model.hpp>
#include <sdu_controllers/utils/utility.hpp>
#include <string>

constexpr double pi = 3.14159265358979323846;
using namespace Eigen;

namespace sdu_controllers::models
{
  ParameterRobotModel::ParameterRobotModel(const std::string &yaml_filepath)
  {
    // Load robot parameters using the shared loader
    auto params_opt = load_parameters_from_yaml(yaml_filepath);

    if (!params_opt.has_value())
    {
      std::cerr << "ParameterRobotModel: Failed to load robot parameters from: " << yaml_filepath << std::endl;
      return;
    }
    const RobotParameters &params = *params_opt;
    // Set the loaded parameters into the parameter instance
    set_robot_parameters(params);

    // Initialize RNEA for dynamics
    rnea_ = std::make_shared<math::RecursiveNewtonEuler>(*this);
  }

  ParameterRobotModel::ParameterRobotModel(const RobotParameters &params)
  {
    // Set the parameters into this ParameterRobotModel instance
    set_robot_parameters(params);
    // Initialize RNEA for dynamics
    rnea_ = std::make_shared<math::RecursiveNewtonEuler>(*this);
  }

  std::optional<RobotParameters> ParameterRobotModel::load_parameters_from_yaml(const std::string &filepath)
  {
    RobotParameters params;
    try
    {
      YAML::Node root = YAML::LoadFile(filepath);

      // basic fields
      if (root["dof"])
      {
        params.dof = static_cast<uint16_t>(root["dof"].as<int>());
      }

      // Inertial section: joint_1, joint_2, ...
      if (!root["inertial"])
      {
        std::cerr << "YAML parse error: missing 'inertial' node in " << filepath << std::endl;
        return std::nullopt;
      }

      YAML::Node inertial = root["inertial"];
      // determine number of joints by counting entries
      const std::size_t n_links = inertial.size();
      params.mass.resize(n_links);
      params.com.resize(static_cast<int>(n_links), 3);
      params.link_inertia.clear();
      params.is_joint_revolute.clear();
      params.link_inertia.reserve(n_links);
      params.is_joint_revolute.reserve(n_links);

      // Parse each joint entry and place values according to the joint index found in the key
      // E.g. key "joint_1" -> index 0
      for (YAML::const_iterator it = inertial.begin(); it != inertial.end(); ++it)
      {
        // it->first is key, it->second is node
        const std::string joint_name = it->first.as<std::string>();
        YAML::Node node = it->second;

        // extract index from joint_name
        auto pos = joint_name.find_last_of('_');
        if (pos == std::string::npos)
        {
          std::cerr << "YAML parse error: unexpected joint key '" << joint_name << "' in " << filepath << std::endl;
          return std::nullopt;
        }
        int idx = std::stoi(joint_name.substr(pos + 1)) - 1;
        if (idx < 0 || static_cast<std::size_t>(idx) >= n_links)
        {
          std::cerr << "YAML parse error: invalid joint index for '" << joint_name << "' in " << filepath << std::endl;
          return std::nullopt;
        }

        // mass
        if (node["mass"])
        {
          params.mass[idx] = node["mass"].as<double>();
        }
        else
        {
          params.mass[idx] = 0.0;
        }

        // com: [x,y,z]
        if (node["com"])
        {
          const YAML::Node com_node = node["com"];
          if (com_node.IsSequence() && com_node.size() == 3)
          {
            params.com.row(idx) =
                Eigen::Vector3d(com_node[0].as<double>(), com_node[1].as<double>(), com_node[2].as<double>());
          }
          else
          {
            params.com.row(idx) = Eigen::Vector3d::Zero();
          }
        }
        else
        {
          params.com.row(idx) = Eigen::Vector3d::Zero();
        }

        // inertia: fields ixx, ixy, ixz, iyy, iyz, izz
        Eigen::Matrix3d I = Eigen::Matrix3d::Zero();
        if (node["inertia"])
        {
          YAML::Node I_node = node["inertia"];
          double ixx = I_node["ixx"] ? I_node["ixx"].as<double>() : 0.0;
          double ixy = I_node["ixy"] ? I_node["ixy"].as<double>() : 0.0;
          double ixz = I_node["ixz"] ? I_node["ixz"].as<double>() : 0.0;
          double iyy = I_node["iyy"] ? I_node["iyy"].as<double>() : 0.0;
          double iyz = I_node["iyz"] ? I_node["iyz"].as<double>() : 0.0;
          double izz = I_node["izz"] ? I_node["izz"].as<double>() : 0.0;

          I(0, 0) = ixx;
          I(0, 1) = ixy;
          I(0, 2) = ixz;
          I(1, 0) = ixy;
          I(1, 1) = iyy;
          I(1, 2) = iyz;
          I(2, 0) = ixz;
          I(2, 1) = iyz;
          I(2, 2) = izz;
        }
        params.link_inertia.resize(n_links);  // ensure size
        params.link_inertia[idx] = I;

        // joint type -> revolute/prismatic
        bool is_revolute = true;
        if (node["type"])
        {
          std::string t = node["type"].as<std::string>();
          if (t == "prismatic")
            is_revolute = false;
        }
        if (params.is_joint_revolute.size() != n_links)
          params.is_joint_revolute.resize(n_links);
        params.is_joint_revolute[idx] = is_revolute;
      }

      // Get kinematics type
      const std::string kinematics_type = root["kinematics"]["type"].as<std::string>();
      bool tried_to_use_frames = false;
      if (kinematics_type == "Frames")
      {
        tried_to_use_frames = true;
        std::cerr << "Defining robot as Frames are not supported yet! using DH parameters instead (if available)." << std::endl;
        if (!root["kinematics"]["dh_params"]["a"])
        {
          std::cerr << "Failed to find DH parameters for this robot!" << std::endl;
          return std::nullopt;
        }
        else
        {
          std::cout << "Found the DH parameters using these instead.";
        }
      }
      else if (kinematics_type == "DH" or tried_to_use_frames)
      {
        params.dof = static_cast<uint16_t>(root["dof"].as<int>());
        auto a_opt = utils::yaml_node_to_eigen_vector(root["kinematics"]["dh_params"]["a"]);
        auto d_opt = utils::yaml_node_to_eigen_vector(root["kinematics"]["dh_params"]["d"]);
        auto alpha_opt = utils::yaml_node_to_eigen_vector(root["kinematics"]["dh_params"]["alpha"]);
        auto theta_opt = utils::yaml_node_to_eigen_vector(root["kinematics"]["dh_params"]["theta"]);
        if (!a_opt || !d_opt || !alpha_opt || !theta_opt)
        {
          std::cerr << "Failed to parse DH parameters!" << std::endl;
          return std::nullopt;
        }
        Eigen::VectorXd a = *a_opt;
        Eigen::VectorXd d = *d_opt;
        Eigen::VectorXd alpha = *alpha_opt;
        Eigen::VectorXd theta = *theta_opt;

        // initialize forward kinematics with DH-parameters
        params.fk_model = std::make_shared<kinematics::DHKinematics>(a, alpha, d, theta, params.is_joint_revolute);
      }
      else
      {
        std::cerr << "YAML parse error: kinematics_type must either be 'DH' or 'Frames'" << std::endl;
        return std::nullopt;
      }

      // Joint limits: collect into vectors sized by n_links
      params.joint_position_bounds.first = Eigen::VectorXd::Zero(static_cast<int>(n_links));
      params.joint_position_bounds.second = Eigen::VectorXd::Zero(static_cast<int>(n_links));
      params.joint_max_velocity = Eigen::VectorXd::Zero(static_cast<int>(n_links));
      params.joint_max_acceleration = Eigen::VectorXd::Zero(static_cast<int>(n_links));
      params.joint_max_torque = Eigen::VectorXd::Zero(static_cast<int>(n_links));

      if (root["joint_limits"])
      {
        YAML::Node limits = root["joint_limits"];
        for (YAML::const_iterator it = limits.begin(); it != limits.end(); ++it)
        {
          const std::string joint_name = it->first.as<std::string>();
          YAML::Node node = it->second;

          auto pos = joint_name.find_last_of('_');
          if (pos == std::string::npos)
          {
            std::cerr << "YAML parse error: unexpected joint_limits key '" << joint_name << "' in " << filepath << std::endl;
            return std::nullopt;
          }
          int idx = std::stoi(joint_name.substr(pos + 1)) - 1;
          if (idx < 0 || static_cast<std::size_t>(idx) >= n_links)
          {
            std::cerr << "YAML parse error: invalid joint index for '" << joint_name << "' in " << filepath << std::endl;
            return std::nullopt;
          }

          if (node["max_position"])
            params.joint_position_bounds.second(idx) = node["max_position"].as<double>();
          if (node["min_position"])
            params.joint_position_bounds.first(idx) = node["min_position"].as<double>();
          if (node["max_velocity"])
            params.joint_max_velocity(idx) = node["max_velocity"].as<double>();
          if (node["max_acceleration"])
            params.joint_max_acceleration(idx) = node["max_acceleration"].as<double>();
          if (node["max_torque"])
            params.joint_max_torque(idx) = node["max_torque"].as<double>();
        }
      }

      // Optional gravity vector at top-level
      if (root["gravity"])
      {
        YAML::Node gnode = root["gravity"];
        if (gnode.IsSequence() && gnode.size() == 3)
        {
          params.g0 = Eigen::Vector3d(gnode[0].as<double>(), gnode[1].as<double>(), gnode[2].as<double>());
        }
      }
      else {
        params.g0 = Eigen::Vector3d(0.0, 0.0, -9.81);
      }

      // Basic validation: sizes must match dof if provided
      if (params.dof != 0 && params.dof != static_cast<uint16_t>(n_links))
      {
        std::cerr << "YAML validation error: declared dof (" << params.dof << ") "
                  << "does not match number of inertial entries (" << n_links << ") in " << filepath << std::endl;
        return std::nullopt;
      }
      // set dof if not set
      if (params.dof == 0)
        params.dof = static_cast<uint16_t>(n_links);

      return params;
    }
    catch (const YAML::Exception &e)
    {
      std::cerr << "YAML exception while loading " << filepath << ": " << e.what() << std::endl;
      return std::nullopt;
    }
    catch (const std::exception &e)
    {
      std::cerr << "Exception while loading " << filepath << ": " << e.what() << std::endl;
      return std::nullopt;
    }
  }

  void ParameterRobotModel::set_robot_parameters(const RobotParameters &p)
  {
    dof_ = p.dof;
    // inertial
    mass_ = p.mass;
    com_ = p.com;
    link_inertia_ = p.link_inertia;
    is_joint_revolute_ = p.is_joint_revolute;
    g0_ = p.g0;

    // joint limits
    joint_pos_bounds_ = p.joint_position_bounds;
    joint_max_velocity_ = p.joint_max_velocity;
    joint_max_acceleration_ = p.joint_max_acceleration;
    joint_max_torque_ = p.joint_max_torque;
    fk_model_ = p.fk_model;
  }

  Eigen::VectorXd ParameterRobotModel::inverse_dynamics(const Eigen::VectorXd &q, const Eigen::VectorXd &dq,
    const Eigen::VectorXd &ddq, const Eigen::VectorXd &he)
  {
    return rnea_->inverse_dynamics(q, dq, ddq, he);
  }

  Eigen::VectorXd ParameterRobotModel::forward_dynamics(const Eigen::VectorXd &q, const Eigen::VectorXd &dq, const Eigen::VectorXd &tau)
  {
    return rnea_->forward_dynamics(q, dq, tau);
  }

  MatrixXd ParameterRobotModel::get_inertia_matrix(const VectorXd& q)
  {
    return rnea_->inertia(q);
  }

  MatrixXd ParameterRobotModel::get_coriolis(const VectorXd& q, const VectorXd& qd)
  {
    return rnea_->coriolis(q, qd);
  }

  MatrixXd ParameterRobotModel::get_gravity(const VectorXd& q)
  {
    return rnea_->gravity(q);
  }

  MatrixXd ParameterRobotModel::get_jacobian(const VectorXd& q)
  {
    return fk_model_->geometric_jacobian(q);
  }

  MatrixXd ParameterRobotModel::get_jacobian_dot(const VectorXd& q, const VectorXd& dq)
  {
    const double dt = 1e-6;

    // Calculate the Jacobian at the current configuration q
    MatrixXd J_current = get_jacobian(q);

    // Estimate the configuration at the next time step (q_next) using Euler integration
    VectorXd q_next = q + (dq * dt);

    // Calculate the Jacobian at the predicted next configuration
    MatrixXd J_next = get_jacobian(q_next);

    // Compute the numerical derivative
    MatrixXd J_dot = (J_next - J_current) / dt;

    return J_dot;

    /*Eigen::MatrixXd J = get_jacobian(q);
    Eigen::Tensor<double, 3> H(dof_, 6, dof_);
    H.setZero();

    for (int j = 0; j < dof_; j++)
    {
      for (int i = j; i < dof_; i++)
      {
        // H.block<3, 1>(j * 6, i) = ...
        Eigen::Vector3d cross1 = J.block<3, 1>(3, j).cross(J.block<3, 1>(0, i));
        H(j, 0, i) = cross1(0);
        H(j, 1, i) = cross1(1);
        H(j, 2, i) = cross1(2);

        // H.block<3, 1>(j * 6 + 3, i) = ...
        Eigen::Vector3d cross2 = J.block<3, 1>(3, j).cross(J.block<3, 1>(3, i));
        H(j, 3, i) = cross2(0);
        H(j, 4, i) = cross2(1);
        H(j, 5, i) = cross2(2);

        if (i != j)
        {
          // H.block<3, 1>(i * 6, j) = H.block<3, 1>(j * 6, i);
          H.slice(Eigen::array<Eigen::Index, 3>{i, 0, j}, Eigen::array<Eigen::Index, 3>{1, 3, 1}) =
              H.slice(Eigen::array<Eigen::Index, 3>{j, 0, i}, Eigen::array<Eigen::Index, 3>{1, 3, 1});

          // H.block<3, 1>(i * 6 + 3, j) = Eigen::Vector3d::Zero();
          H.slice(Eigen::array<Eigen::Index, 3>{i, 3, j}, Eigen::array<Eigen::Index, 3>{1, 3, 1}).setZero();
        }
      }
    }

    Eigen::TensorMap<Eigen::Tensor<const double, 1>> dq_tensor(dq.data(), dq.size());
    // H's 3rd dimension (index 2) contracts with dq's 1st dimension (index 0)
    Eigen::array<Eigen::IndexPair<int>, 1> contraction_dims = { Eigen::IndexPair<int>(2, 0)};
    Eigen::Tensor<double, 2> result_tensor = H.contract(dq_tensor, contraction_dims);

    const int rows = result_tensor.dimension(0);
    const int cols = result_tensor.dimension(1);

    // Create a row-major Map of the tensor's data
    Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> row_major_map(result_tensor.data(), rows, cols);

    Eigen::MatrixXd result_matrix = row_major_map;
    return result_matrix */
  }

  std::pair<VectorXd, VectorXd> ParameterRobotModel::get_joint_pos_bounds()
  {
    return joint_pos_bounds_;
  }

  VectorXd ParameterRobotModel::get_joint_max_vel()
  {
    return joint_max_velocity_;
  }

  VectorXd ParameterRobotModel::get_joint_max_acc()
  {
    return joint_max_acceleration_;
  }

  VectorXd ParameterRobotModel::get_joint_max_torque()
  {
    return joint_max_torque_;
  }

  uint16_t ParameterRobotModel::get_dof() const
  {
    return dof_;
  }

  std::vector<double> ParameterRobotModel::get_m()
  {
    return mass_;
  }

  std::vector<bool> ParameterRobotModel::get_is_joint_revolute()
  {
    return is_joint_revolute_;
  }

  Eigen::Vector3d ParameterRobotModel::get_g0()
  {
    return g0_;
  }

  Eigen::Matrix<double, Eigen::Dynamic, 3> ParameterRobotModel::get_CoM()
  {
    return com_;
  }

  std::vector<Eigen::Matrix3d> ParameterRobotModel::get_link_inertia()
  {
    return link_inertia_;
  }

  const kinematics::ForwardKinematics& ParameterRobotModel::get_fk_solver() const
  {
    return *fk_model_;
  }

  void ParameterRobotModel::set_com(const Eigen::Matrix<double, Eigen::Dynamic, 3> &com)
  {
    com_ = com;
  }

  void ParameterRobotModel::set_link_inertia(const std::vector<Eigen::Matrix3d> &link_inertia)
  {
    link_inertia_ = link_inertia;
  }

  void ParameterRobotModel::set_mass(const std::vector<double> &mass)
  {
    mass_ = mass;
  }
}  // namespace sdu_controllers::models
