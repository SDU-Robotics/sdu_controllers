#pragma once
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
      PRISMATIC,
      FIXED
    };

    /**
     * @brief Get the transformation matrix from base to end-effector
     * @param q [in] Joint configuration
     * @return Homogeneous transformation matrix to end-effector
     */
    virtual Eigen::Matrix4d forward_kinematics(const Eigen::VectorXd& q) const = 0;

    /**
     * @brief Get the transformation matrix from base to end-effector (overload for std::vector<double>)
     * @param q [in] Joint configuration as std::vector<double>
     * @return Homogeneous transformation matrix to end-effector
     */
    Eigen::Matrix4d forward_kinematics(const std::vector<double>& q) const;

    /**
     * @brief Get the transformation matrices from base to each joint frame
     * @param q [in] Joint configuration
     * @return Vector of homogeneous transformation matrices to each joint frame
     */
    virtual std::vector<Eigen::Matrix4d> forward_kinematics_all(const Eigen::VectorXd& q) const = 0;

    /**
     * @brief Get the transformation matrices from base to each joint frame (overload for std::vector<double>)
     * @param q [in] Joint configuration as std::vector<double>
     * @return Vector of homogeneous transformation matrices to each joint frame
     */
    std::vector<Eigen::Matrix4d> forward_kinematics_all(const std::vector<double>& q) const;

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
     * @brief Compute the geometric Jacobian at the given joint configuration using precomputed forward kinematics matrices
     * @param fk_matrices Precomputed forward kinematics matrices for each joint
     * @return The 6xDOF geometric Jacobian matrix
     */
    virtual Eigen::Matrix<double, 6, Eigen::Dynamic> geometric_jacobian(
        const std::vector<Eigen::Matrix4d>& fk_matrices) const;

    /**
     * @brief Get the degrees of freedom of the kinematic chain (number of actuated joints, excludes FIXED)
     * @return Number of actuated degrees of freedom
     */
    size_t get_dof() const;

    /**
     * @brief Get the total number of links in the kinematic chain (including FIXED joints)
     * @return Total number of links
     */
    size_t get_num_links() const;

    /**
     * @brief Set the Tool Center Point (TCP) transform relative to the last link frame.
     *        Defaults to identity (TCP coincides with last link frame).
     * @param tcp_transform [in] 4x4 homogeneous transform from last link frame to TCP
     */
    void set_tcp(const Eigen::Matrix4d& tcp_transform);

    /**
     * @brief Get the current TCP transform
     * @return 4x4 homogeneous transform from last link frame to TCP
     */
    const Eigen::Matrix4d& get_tcp() const;

    /**
     * @brief Get the TCP pose in the base frame at the given joint configuration.
     *        Equivalent to forward_kinematics(q) * tcp_transform.
     * @param q [in] Joint configuration
     * @return 4x4 homogeneous transformation matrix to TCP
     */
    Eigen::Matrix4d get_tcp_pose(const Eigen::VectorXd& q) const;

    /**
     * @brief Get the TCP pose in the base frame (overload for std::vector<double>)
     * @param q [in] Joint configuration as std::vector<double>
     * @return 4x4 homogeneous transformation matrix to TCP
     */
    Eigen::Matrix4d get_tcp_pose(const std::vector<double>& q) const;

   protected:
    ForwardKinematics(const std::vector<ForwardKinematics::JointType>& joint_type);
    virtual ~ForwardKinematics() = default;

    std::vector<ForwardKinematics::JointType> joint_type_;
    Eigen::Matrix4d tcp_transform_ = Eigen::Matrix4d::Identity();
  };
}  // namespace sdu_controllers::kinematics

#endif  // SDU_CONTROLLERS_FORWARD_KINEMATICS_HPP
