#pragma once
#ifndef SDU_CONTROLLERS_MATH_POSE_HPP
#define SDU_CONTROLLERS_MATH_POSE_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <array>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

namespace sdu_controllers::math
{
  /**
   * @brief A class representing a 3D pose with position and orientation.
   *
   * This class combines a 3D position vector and a quaternion orientation to
   * represent a complete pose in 3D space. The orientation is stored internally
   * as a quaternion in scalar-first format (w, x, y, z).
   */
  class Pose
  {
   public:
    /**
     * @brief Default constructor.
     *
     * Initializes the pose to identity (zero position and identity rotation).
     */
    Pose() : position_(Eigen::Vector3d::Zero()), orientation_(Eigen::Quaterniond::Identity())
    {
    }

    /**
     * @brief Construct a pose from position and orientation.
     *
     * @param position The 3D position vector
     * @param orientation The orientation as a quaternion in scalar-first format (w, x, y, z)
     */
    Pose(const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation)
        : position_(position),
          orientation_(orientation)
    {
      orientation_.normalize();
    }

    /**
     * @brief Construct a pose from position and angle-axis vector.
     *
     * @param position The 3D position vector
     * @param angle_axis Vector whose direction specifies the rotation axis and whose
     * magnitude specifies the rotation angle in radians.
     */
    Pose(const Eigen::Vector3d& position, const Eigen::Vector3d& angle_axis) : position_(position)
    {
      double angle = angle_axis.norm();
      if (angle < 1e-10)
      {
        orientation_ = Eigen::Quaterniond::Identity();
      }
      else
      {
        Eigen::Vector3d axis = angle_axis / angle;
        orientation_ = Eigen::Quaterniond(Eigen::AngleAxisd(angle, axis));
      }
      orientation_.normalize();
    }

    /**
     * @brief Construct a pose from a vector containing position and orientation the orientation
     * can be represented as either rotation vector (RX, RY, RZ) or as a quaternion in scalar-first format
     * (w, x, y, z).
     *
     * @param pose_vector std::vector containing [x, y, z, RX, RY, RZ] with orientation as rotation vector or a
     *  std::vector containing [x, y, z, qw, qx, qy, qz] with orientation as quaternion in scalar-first format
     * (w, x, y, z).
     * @throw std::invalid_argument if the vector does not contain 6 or 7 elements.
     */
    explicit Pose(const std::vector<double>& pose_vector)
    {
      if (pose_vector.size() == 6)
      {
        position_ << pose_vector[0], pose_vector[1], pose_vector[2];
        Eigen::Vector3d angle_axis(pose_vector[3], pose_vector[4], pose_vector[5]);
        double angle = angle_axis.norm();
        if (angle < 1e-10)
        {
          orientation_ = Eigen::Quaterniond::Identity();
        }
        else
        {
          Eigen::Vector3d axis = angle_axis / angle;
          orientation_ = Eigen::Quaterniond(Eigen::AngleAxisd(angle, axis));
        }
        orientation_.normalize();
      }
      else if (pose_vector.size() == 7)
      {
        position_ = Eigen::Vector3d(pose_vector[0], pose_vector[1], pose_vector[2]);
        orientation_ = Eigen::Quaterniond(pose_vector[3], pose_vector[4], pose_vector[5], pose_vector[6]);
        orientation_.normalize();
      }
      else
      {
        throw std::invalid_argument("Pose vector must have either 6 or 7 elements.");
      }
    }

    /**
     * @brief Construct a pose from an array containing position and orientation.
     *
     * @param pose_array Array containing [x, y, z, qw, qx, qy, qz] where the quaternion
     * is in scalar-first format (w, x, y, z).
     */
    explicit Pose(const std::array<double, 7>& pose_array)
    {
      position_ = Eigen::Vector3d(pose_array[0], pose_array[1], pose_array[2]);
      orientation_ = Eigen::Quaterniond(pose_array[3], pose_array[4], pose_array[5], pose_array[6]);
      orientation_.normalize();
    }

    /**
     * @brief Construct a pose from a homogeneous transformation matrix.
     *
     * @param transform The 4x4 homogeneous transformation matrix
     */
    explicit Pose(const Eigen::Affine3d& transform)
    {
      position_ = transform.translation();
      orientation_ = Eigen::Quaterniond(transform.rotation());
      orientation_.normalize();
    }

    /**
     * @brief Get the position component of the pose.
     * @return The 3D position vector
     */
    Eigen::Vector3d get_position() const
    {
      return position_;
    }

    /**
     * @brief Get the orientation component of the pose.
     * @return The orientation as a quaternion in scalar-first format (w, x, y, z).
     */
    Eigen::Quaterniond get_orientation() const
    {
      return orientation_;
    }

    /**
     * @brief Set the position component of the pose.
     * @param position The new 3D position vector
     */
    void set_position(const Eigen::Vector3d& position)
    {
      position_ = position;
    }

    /**
     * @brief Set the orientation component of the pose.
     * @param orientation The new orientation as a quaternion in scalar-first format (w, x, y, z).
     */
    void set_orientation(const Eigen::Quaterniond& orientation)
    {
      orientation_ = orientation;
      orientation_.normalize();
    }

    /**
     * @brief Convert the orientation to Euler angles in the specified order.
     *
     * @param order String specifying the rotation order (e.g., "ZYZ", "XYZ"). Case insensitive.
     * @return Vector3d containing the Euler angles in radians
     * @throw std::invalid_argument if the order string is invalid
     */
    Eigen::Vector3d to_euler_angles(const std::string& order = "ZYZ") const
    {
      static const std::map<char, int> axis_map = { { 'X', 0 }, { 'Y', 1 }, { 'Z', 2 }, { 'x', 0 }, { 'y', 1 }, { 'z', 2 } };

      if (order.length() != 3)
      {
        throw std::invalid_argument("Order must be a string of length 3");
      }

      try
      {
        int axis1 = axis_map.at(order[0]);
        int axis2 = axis_map.at(order[1]);
        int axis3 = axis_map.at(order[2]);
        return orientation_.toRotationMatrix().eulerAngles(axis1, axis2, axis3);
      }
      catch (const std::out_of_range&)
      {
        throw std::invalid_argument("Invalid rotation axis specified. Use X, Y, or Z");
      }
    }

    /**
     * @brief Convert the orientation to angle-axis representation.
     *
     * @return Vector3d where the direction represents the rotation axis and the magnitude
     * .represents the rotation angle in radians
     */
    Eigen::Vector3d to_angle_axis_vector() const
    {
      Eigen::AngleAxisd aa(orientation_);
      return aa.angle() * aa.axis();
    }

    /**
     * @brief Convert the pose to a 7D vector representation.
     *
     * @return Vector7d containing [x, y, z, qw, qx, qy, qz] where the quaternion
     *         is in scalar-first format (w, x, y, z)
     */
    Eigen::Matrix<double, 7, 1> to_vector7d() const
    {
      Eigen::Matrix<double, 7, 1> result;
      result << position_, orientation_.w(), orientation_.x(), orientation_.y(), orientation_.z();
      return result;
    }

    /**
     * @brief Convert the pose to a std::vector representation.
     *
     * @return Vector containing [x, y, z, qw, qx, qy, qz] where the quaternion
     *         is in scalar-first format (w, x, y, z)
     */
    std::vector<double> to_std_vector() const
    {
      std::vector<double> result(7);
      result[0] = position_.x();
      result[1] = position_.y();
      result[2] = position_.z();
      result[3] = orientation_.w();
      result[4] = orientation_.x();
      result[5] = orientation_.y();
      result[6] = orientation_.z();
      return result;
    }

    /**
     * @brief Get the pose as a homogeneous transformation matrix.
     * @return The 4x4 homogeneous transformation matrix.
     */
    Eigen::Affine3d to_transform() const
    {
      Eigen::Affine3d transform = Eigen::Affine3d::Identity();
      transform.translation() = position_;
      transform.rotate(orientation_);
      return transform;
    }

    /**
     * @brief Implicit conversion operator to Eigen::Affine3d.
     *
     * Allows the Pose to be implicitly converted to a homogeneous transformation matrix
     * when used in assignments or function calls expecting an Affine3d.
     *
     * @return The pose as a 4x4 homogeneous transformation matrix
     */
    operator Eigen::Affine3d() const
    {
      return to_transform();
    }

   private:
    Eigen::Vector3d position_;        ///< The position component of the pose
    Eigen::Quaterniond orientation_;  ///< The orientation component of the pose (stored in scalar-first format)
  };

}  // namespace sdu_controllers::math

#endif  // SDU_CONTROLLERS_MATH_POSE_HPP
