#pragma once
#ifndef SDU_CONTROLLERS_MATH_POSE_HPP
#define SDU_CONTROLLERS_MATH_POSE_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <array>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace sdu_controllers::math
{
  /**
   * @brief Format options for pose string representation.
   */
  enum class PoseFormat
  {
    Default,    ///< Basic format: Position [x, y, z], Orientation [w, x, y, z]
    Compact,    ///< Compact format: [x, y, z, qw, qx, qy, qz]
    Verbose,    ///< Verbose format with labels and separate lines
    Euler,      ///< Position and Euler angles (default ZYZ)
    AngleAxis,  ///< Position and angle-axis representation
    Matrix      ///< Full 4x4 transformation matrix
  };

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
     * @brief Copy constructor.
     *
     * Creates a new pose as a copy of another pose.
     * @param other The pose to copy
     */
    Pose(const Pose& other) : position_(other.position_), orientation_(other.orientation_)
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
     * @brief Construct a pose from a vector containing position and orientation the orientation
     * can be represented as either rotation vector (RX, RY, RZ) or as a quaternion in scalar-first format
     * (w, x, y, z).
     *
     * @param pose_vector std::vector containing [x, y, z, RX, RY, RZ] with orientation as rotation vector or a
     *  std::vector containing [x, y, z, qw, qx, qy, qz] with orientation as quaternion in scalar-first format
     * (w, x, y, z).
     * @throw std::invalid_argument if the vector does not contain 6 or 7 elements.
     */
    explicit Pose(const Eigen::VectorXd& pose_vector)
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
     * @brief Construct a pose from a 4x4 transformation matrix.
     *
     * @param transform The 4x4 homogeneous transformation matrix
     */
    explicit Pose(const Eigen::Matrix4d& transform)
    {
      Eigen::Affine3d affine(transform);
      position_ = affine.translation();
      orientation_ = Eigen::Quaterniond(affine.rotation());
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
     * @brief Get the pose as a 6D vector with position and angle-axis orientation.
     * @return std::vector<double> containing [X, Y, Z, Rx, Ry, Rz] where Rx, Ry, Rz represent
     *         the rotation as an angle-axis (equivalent axis-angle) representation in radians.
     */
    std::vector<double> to_pose6d_std() const
    {
      Eigen::AngleAxisd angle_axis(orientation_);
      Eigen::Vector3d rotation_vector = angle_axis.angle() * angle_axis.axis();
      
      return std::vector<double>{
        position_.x(), position_.y(), position_.z(),
        rotation_vector.x(), rotation_vector.y(), rotation_vector.z()
      };
    }

    /**
     * @brief Get the pose as a 6D Eigen vector with position and angle-axis orientation.
     * @return Eigen::VectorXd containing [X, Y, Z, Rx, Ry, Rz] where Rx, Ry, Rz represent
     *         the rotation as an angle-axis (equivalent axis-angle) representation in radians.
     */
    Eigen::VectorXd to_pose6d_eigen() const
    {
      Eigen::AngleAxisd angle_axis(orientation_);
      Eigen::Vector3d rotation_vector = angle_axis.angle() * angle_axis.axis();
      
      Eigen::VectorXd result(6);
      result << position_, rotation_vector;
      return result;
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

    /**
     * @brief Assignment operator to set the pose from another Pose object.
     *
     * This allows the Pose to be assigned from another Pose object.
     *
     * @param other The Pose object to copy from
     * @return Reference to this Pose object
     */
    Pose& operator=(const Pose& other)
    {

        position_ = other.position_;
        orientation_ = other.orientation_;
        g_pose_io_format_ = other.g_pose_io_format_;
      
      return *this;
    }

    /**
     * @brief Assignment operator to set the pose from a vector.
     *
     * This allows the Pose to be assigned from a std::vector containing either
     * [x, y, z, RX, RY, RZ] (rotation vector) or [x, y, z, qw, qx, qy, qz] (quaternion).
     *
     * @param pose_vector The vector containing the pose data
     * @return Reference to this Pose object
     */
    Pose& operator=(std::vector<double> pose_vector)
    {
      Pose new_pose(pose_vector);
      position_ = new_pose.position_;
      orientation_ = new_pose.orientation_;
      g_pose_io_format_ = new_pose.g_pose_io_format_;
      return *this;
    }

    /**
     * @brief Format the pose as a string according to the specified format.
     *
     * @param format The format to use for string representation
     * @param precision The number of decimal places to use for floating-point values
     * @return A formatted string representation of the pose
     */
    std::string to_string(PoseFormat format = PoseFormat::Default, int precision = 6) const
    {
      std::ostringstream oss;
      oss << std::fixed << std::setprecision(precision);

      switch (format)
      {
        case PoseFormat::Compact:
        {
          oss << "[" << position_.x() << ", " << position_.y() << ", " << position_.z() << ", " << orientation_.w() << ", "
              << orientation_.x() << ", " << orientation_.y() << ", " << orientation_.z() << "]";
          break;
        }
        case PoseFormat::Verbose:
        {
          oss << "Pose:\n"
              << "  Position:\n"
              << "    x: " << position_.x() << "\n"
              << "    y: " << position_.y() << "\n"
              << "    z: " << position_.z() << "\n"
              << "  Orientation (quaternion):\n"
              << "    w: " << orientation_.w() << "\n"
              << "    x: " << orientation_.x() << "\n"
              << "    y: " << orientation_.y() << "\n"
              << "    z: " << orientation_.z();
          break;
        }
        case PoseFormat::Euler:
        {
          Eigen::Vector3d euler = to_euler_angles("ZYZ");
          oss << "Position [" << position_.x() << ", " << position_.y() << ", " << position_.z() << "], " << "Euler ZYZ ["
              << euler.x() << ", " << euler.y() << ", " << euler.z() << "] rad";
          break;
        }
        case PoseFormat::AngleAxis:
        {
          Eigen::Vector3d aa = to_angle_axis_vector();
          oss << "Position [" << position_.x() << ", " << position_.y() << ", " << position_.z() << "], " << "Angle-Axis ["
              << aa.x() << ", " << aa.y() << ", " << aa.z() << "]";
          break;
        }
        case PoseFormat::Matrix:
        {
          Eigen::Affine3d transform = to_transform();
          oss << "Transform Matrix:\n";
          for (int i = 0; i < 4; ++i)
          {
            oss << "  [";
            for (int j = 0; j < 4; ++j)
            {
              oss << transform.matrix()(i, j);
              if (j < 3)
                oss << ", ";
            }
            oss << "]\n";
          }
          break;
        }
        case PoseFormat::Default:
        default:
        {
          oss << "Position [" << position_.x() << ", " << position_.y() << ", " << position_.z() << "], " << "Orientation ["
              << orientation_.w() << ", " << orientation_.x() << ", " << orientation_.y() << ", " << orientation_.z() << "]";
          break;
        }
      }
      return oss.str();
    };

    /**
     * @brief Helper class to set the format for printing poses.
     *
     * Usage: std::cout << PoseIOFormat(PoseFormat::Verbose, 4) << myPose;
     */
    class PoseIOFormat
    {
     public:
      /**
       * @brief Constructor to set formatting options.
       *
       * @param format The format to use for Pose output
       * @param precision The number of decimal places to use
       */
      PoseIOFormat(PoseFormat format = PoseFormat::Default, int precision = 6) : format_(format), precision_(precision)
      {
      }

      PoseFormat format() const
      {
        return format_;
      }
      int precision() const
      {
        return precision_;
      }

     private:
      PoseFormat format_;
      int precision_;
    };

    /**
     * @brief Set the global pose IO format.
     *
     * @param format The new format to use
     * @return The previous format
     */
    PoseIOFormat set_pose_io_format(const PoseIOFormat& format)
    {
      PoseIOFormat old = g_pose_io_format_;
      g_pose_io_format_ = format;
      return old;
    }

    /**
     * @brief Get the current global pose IO format.
     *
     * @return The current format
     */
    PoseIOFormat get_pose_io_format()
    {
      return g_pose_io_format_;
    }

   private:
    Eigen::Vector3d position_;        ///< The position component of the pose
    Eigen::Quaterniond orientation_;  ///< The orientation component of the pose (stored in scalar-first format)

    // format state
    PoseIOFormat g_pose_io_format_ = PoseIOFormat();
  };

}  // namespace sdu_controllers::math

#endif  // SDU_CONTROLLERS_MATH_POSE_HPP
