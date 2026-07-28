#pragma once
#ifndef SDU_CONTROLLERS_DH_KINEMATICS_HPP
#define SDU_CONTROLLERS_DH_KINEMATICS_HPP

#include <sdu_controllers/kinematics/forward_kinematics.hpp>
namespace sdu_controllers::kinematics
{
  class DHParam
  {
   public:
    double a;                // link length
    double alpha;            // link twist
    double d;                // link offset
    double theta;            // joint angle
    ForwardKinematics::JointType joint_type = ForwardKinematics::REVOLUTE;  // joint type (REVOLUTE, PRISMATIC, or FIXED)
  };


  /**
   * @class DHKinematics
   * @brief Forward kinematics computation using classical Denavit-Hartenberg (DH) parameters
   *
   * This class implements forward kinematics using the classical (standard) Denavit-Hartenberg 
   * parameter convention, as presented in standard robotics textbooks (Spong, Murray, Sastry).
   */
  class DHKinematics : public ForwardKinematics
  {
   public:

    /**
     * @brief Construct an empty DHKinematics
     */
    DHKinematics();

    /**
     * @brief Construct a DHKinematics object using DH parameters
     * @param dh_parameters [in] Vector of DH parameters for each link
     * @throws std::runtime_error if dh_parameters is empty
     */
    DHKinematics(const std::vector<DHParam>& dh_parameters);

    /**
     * @brief Construct a DHKinematics object using DH parameters
     * @param a [in] Vector of link lengths
     * @param alpha [in] Vector of link twists
     * @param d [in] Vector of link offsets
     * @param theta [in] Vector of joint angles
     * @param joint_types [in] Vector of joint types (REVOLUTE, PRISMATIC, or FIXED)
     * @throws std::runtime_error if input vectors are not of the same size or are empty
     */
    DHKinematics(
        const std::vector<double>& a,
        const std::vector<double>& alpha,
        const std::vector<double>& d,
        const std::vector<double>& theta,
        const std::vector<ForwardKinematics::JointType>& joint_types);

    /**
     * @brief Construct a DHKinematics object using DH parameters
     * @param a [in] Vector of link lengths
     * @param alpha [in] Vector of link twists
     * @param d [in] Vector of link offsets
     * @param theta [in] Vector of joint angles
     * @param joint_types [in] Vector of joint types (REVOLUTE, PRISMATIC, or FIXED)
     * @throws std::runtime_error if input vectors are not of the same size or are empty
     */
    DHKinematics(
        const Eigen::VectorXd& a,
        const Eigen::VectorXd& alpha,
        const Eigen::VectorXd& d,
        const Eigen::VectorXd& theta,
        const std::vector<ForwardKinematics::JointType>& joint_types);

    /**
     * @brief Get the transformation matrix from base to end-effector
     * @param q [in] Joint configuration
     * @return Homogeneous transformation matrix to end-effector
     */
    virtual Eigen::Matrix4d forward_kinematics(const Eigen::VectorXd& q) const;

    /**
     * @brief Get the transformation matrices from base to each joint frame
     * @param q [in] Joint configuration
     * @return Vector of homogeneous transformation matrices to each joint frame
     */
    virtual std::vector<Eigen::Matrix4d> forward_kinematics_all(const Eigen::VectorXd& q) const;

    std::vector<double> get_a() const;
    std::vector<double> get_alpha() const;
    std::vector<double> get_d() const;
    std::vector<double> get_theta() const;

   private:
    std::vector<double> a_;
    std::vector<double> alpha_;
    std::vector<double> d_;
    std::vector<double> theta_;
  };

}  // namespace sdu_controllers::kinematics

#endif
