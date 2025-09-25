#ifndef SDU_CONTROLLERS_FORWARD_KINEMATICS_HPP
#define SDU_CONTROLLERS_FORWARD_KINEMATICS_HPP

#include <Eigen/Dense>
#include <iostream>
#include <memory>
#include <sdu_controllers/models/robot_model.hpp>

namespace sdu_controllers::kinematics
{
  class ForwardKinematics
  {
   public:
    enum JointType
    {
      REVOLUTE,
      PRISMATIC
    };

    /**
     * @brief Get the transformation matrix from base to end-effector
     * @param q [in] Joint configuration
     * @return Homogeneous transformation matrix to end-effector
     */
    virtual Eigen::Matrix4d forward_kinematics(const Eigen::VectorXd& q) const = 0;

    /**
     * @brief Get the transformation matrices from base to each joint frame
     * @param q [in] Joint configuration
     * @return Vector of homogeneous transformation matrices to each joint frame
     */
    virtual std::vector<Eigen::Matrix4d> forward_kinematics_all(const Eigen::VectorXd& q) const = 0;

    /**
     * @brief Get the type of each joint in the kinematic chain
     * @return list of joint types
     */
    virtual const std::vector<JointType>& get_joint_types() const;

    /**
     * @brief Compute the geometric Jacobian at the given joint configuration
     * @param q [in] Joint configuration
     * @return The 6xDOF geometric Jacobian matrix
     */
    virtual Eigen::Matrix<double, 6, Eigen::Dynamic> geometric_jacobian(const Eigen::VectorXd& q) const;

    /**
     * @brief Get the degrees of freedom of the kinematic chain
     * @return Number of degrees of freedom
     */
    size_t get_dof() const;

   protected:
    ForwardKinematics(const std::vector<ForwardKinematics::JointType>& jointType);
    ForwardKinematics(const std::vector<bool>& is_joint_revolute);
    virtual ~ForwardKinematics() = default;

    std::vector<ForwardKinematics::JointType> jointType_;
  };
}  // namespace sdu_controllers::kinematics

#endif  // SDU_CONTROLLERS_FORWARD_KINEMATICS_HPP
