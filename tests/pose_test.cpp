#define CATCH_CONFIG_MAIN
#include <catch2/catch.hpp>
#include <sdu_controllers/math/pose.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif


using namespace sdu_controllers::math;

// Helper function to compare doubles with tolerance
bool approx_equal(double a, double b, double tolerance = 1e-10)
{
  return std::abs(a - b) < tolerance;
}

// Helper function to compare Vector3d with tolerance
bool approx_equal(const Eigen::Vector3d& a, const Eigen::Vector3d& b, double tolerance = 1e-10)
{
  return (a - b).norm() < tolerance;
}

// Helper function to compare Quaternions with tolerance
bool approx_equal(const Eigen::Quaterniond& a, const Eigen::Quaterniond& b, double tolerance = 1e-10)
{
  // Quaternions q and -q represent the same rotation
  double dot = a.w() * b.w() + a.x() * b.x() + a.y() * b.y() + a.z() * b.z();
  return std::abs(std::abs(dot) - 1.0) < tolerance;
}

TEST_CASE("Pose: Default constructor creates identity pose", "[pose][constructor]")
{
  Pose pose;
  
  REQUIRE(approx_equal(pose.get_position(), Eigen::Vector3d::Zero()));
  REQUIRE(approx_equal(pose.get_orientation(), Eigen::Quaterniond::Identity()));
}

TEST_CASE("Pose: Constructor from position and quaternion", "[pose][constructor]")
{
  Eigen::Vector3d pos(1.0, 2.0, 3.0);
  Eigen::Quaterniond quat(Eigen::AngleAxisd(M_PI / 4, Eigen::Vector3d::UnitZ()));
  
  Pose pose(pos, quat);
  
  REQUIRE(approx_equal(pose.get_position(), pos));
  REQUIRE(approx_equal(pose.get_orientation(), quat));
}

TEST_CASE("Pose: Constructor from position and angle-axis vector", "[pose][constructor]")
{
  Eigen::Vector3d pos(1.0, 2.0, 3.0);
  Eigen::Vector3d angle_axis(0.0, 0.0, M_PI / 4);
  
  Pose pose(pos, angle_axis);
  
  Eigen::Quaterniond expected_quat(Eigen::AngleAxisd(M_PI / 4, Eigen::Vector3d::UnitZ()));
  
  REQUIRE(approx_equal(pose.get_position(), pos));
  REQUIRE(approx_equal(pose.get_orientation(), expected_quat));
}

TEST_CASE("Pose: Constructor from 6D vector (rotation vector)", "[pose][constructor]")
{
  std::vector<double> pose_vec = {1.0, 2.0, 3.0, 0.0, 0.0, M_PI / 4};
  
  Pose pose(pose_vec);
  
  Eigen::Vector3d expected_pos(1.0, 2.0, 3.0);
  Eigen::Quaterniond expected_quat(Eigen::AngleAxisd(M_PI / 4, Eigen::Vector3d::UnitZ()));
  
  REQUIRE(approx_equal(pose.get_position(), expected_pos));
  REQUIRE(approx_equal(pose.get_orientation(), expected_quat));
}

TEST_CASE("Pose: Constructor from 7D vector (quaternion)", "[pose][constructor]")
{
  Eigen::Quaterniond quat(Eigen::AngleAxisd(M_PI / 3, Eigen::Vector3d::UnitX()));
  std::vector<double> pose_vec = {1.0, 2.0, 3.0, quat.w(), quat.x(), quat.y(), quat.z()};
  
  Pose pose(pose_vec);
  
  Eigen::Vector3d expected_pos(1.0, 2.0, 3.0);
  
  REQUIRE(approx_equal(pose.get_position(), expected_pos));
  REQUIRE(approx_equal(pose.get_orientation(), quat));
}

TEST_CASE("Pose: Constructor from Eigen::VectorXd", "[pose][constructor]")
{
  Eigen::VectorXd pose_vec(6);
  pose_vec << 1.0, 2.0, 3.0, 0.0, M_PI / 4, 0.0;
  
  Pose pose(pose_vec);
  
  Eigen::Vector3d expected_pos(1.0, 2.0, 3.0);
  Eigen::Quaterniond expected_quat(Eigen::AngleAxisd(M_PI / 4, Eigen::Vector3d::UnitY()));
  
  REQUIRE(approx_equal(pose.get_position(), expected_pos));
  REQUIRE(approx_equal(pose.get_orientation(), expected_quat));
}

TEST_CASE("Pose: Constructor from std::array", "[pose][constructor]")
{
  Eigen::Quaterniond quat = Eigen::Quaterniond::Identity();
  std::array<double, 7> pose_array = {1.0, 2.0, 3.0, quat.w(), quat.x(), quat.y(), quat.z()};
  
  Pose pose(pose_array);
  
  REQUIRE(approx_equal(pose.get_position(), Eigen::Vector3d(1.0, 2.0, 3.0)));
  REQUIRE(approx_equal(pose.get_orientation(), quat));
}

TEST_CASE("Pose: Constructor from Affine3d transformation", "[pose][constructor]")
{
  Eigen::Affine3d transform = Eigen::Affine3d::Identity();
  transform.translate(Eigen::Vector3d(1.0, 2.0, 3.0));
  transform.rotate(Eigen::AngleAxisd(M_PI / 4, Eigen::Vector3d::UnitZ()));
  
  Pose pose(transform);
  
  REQUIRE(approx_equal(pose.get_position(), Eigen::Vector3d(1.0, 2.0, 3.0)));
  REQUIRE(approx_equal(pose.get_orientation(), Eigen::Quaterniond(transform.rotation())));
}

TEST_CASE("Pose: Constructor from Matrix4d", "[pose][constructor]")
{
  Eigen::Affine3d transform = Eigen::Affine3d::Identity();
  transform.translate(Eigen::Vector3d(1.0, 2.0, 3.0));
  transform.rotate(Eigen::AngleAxisd(M_PI / 4, Eigen::Vector3d::UnitZ()));
  
  Eigen::Matrix4d matrix = transform.matrix();
  Pose pose(matrix);
  
  REQUIRE(approx_equal(pose.get_position(), Eigen::Vector3d(1.0, 2.0, 3.0)));
  REQUIRE(approx_equal(pose.get_orientation(), Eigen::Quaterniond(transform.rotation())));
}

TEST_CASE("Pose: Copy constructor", "[pose][constructor]")
{
  Pose original(Eigen::Vector3d(1.0, 2.0, 3.0), 
                Eigen::Quaterniond(Eigen::AngleAxisd(M_PI / 4, Eigen::Vector3d::UnitZ())));
  
  Pose copy(original);
  
  REQUIRE(approx_equal(copy.get_position(), original.get_position()));
  REQUIRE(approx_equal(copy.get_orientation(), original.get_orientation()));
}

TEST_CASE("Pose: Setters modify pose correctly", "[pose][setters]")
{
  Pose pose;
  
  Eigen::Vector3d new_pos(5.0, 6.0, 7.0);
  pose.set_position(new_pos);
  REQUIRE(approx_equal(pose.get_position(), new_pos));
  
  Eigen::Quaterniond new_quat(Eigen::AngleAxisd(M_PI / 3, Eigen::Vector3d::UnitX()));
  pose.set_orientation(new_quat);
  REQUIRE(approx_equal(pose.get_orientation(), new_quat));
}

TEST_CASE("Pose: to_vector7d conversion", "[pose][conversion]")
{
  Eigen::Vector3d pos(1.0, 2.0, 3.0);
  Eigen::Quaterniond quat(Eigen::AngleAxisd(M_PI / 4, Eigen::Vector3d::UnitZ()));
  Pose pose(pos, quat);
  
  Eigen::Matrix<double, 7, 1> vec = pose.to_vector7d();
  
  REQUIRE(approx_equal(vec(0), pos.x()));
  REQUIRE(approx_equal(vec(1), pos.y()));
  REQUIRE(approx_equal(vec(2), pos.z()));
  REQUIRE(approx_equal(vec(3), quat.w()));
  REQUIRE(approx_equal(vec(4), quat.x()));
  REQUIRE(approx_equal(vec(5), quat.y()));
  REQUIRE(approx_equal(vec(6), quat.z()));
}

TEST_CASE("Pose: to_std_vector conversion", "[pose][conversion]")
{
  Eigen::Vector3d pos(1.0, 2.0, 3.0);
  Eigen::Quaterniond quat(Eigen::AngleAxisd(M_PI / 4, Eigen::Vector3d::UnitZ()));
  Pose pose(pos, quat);
  
  std::vector<double> vec = pose.to_std_vector();
  
  REQUIRE(vec.size() == 7);
  REQUIRE(approx_equal(vec[0], pos.x()));
  REQUIRE(approx_equal(vec[1], pos.y()));
  REQUIRE(approx_equal(vec[2], pos.z()));
  REQUIRE(approx_equal(vec[3], quat.w()));
  REQUIRE(approx_equal(vec[4], quat.x()));
  REQUIRE(approx_equal(vec[5], quat.y()));
  REQUIRE(approx_equal(vec[6], quat.z()));
}

TEST_CASE("Pose: to_transform conversion", "[pose][conversion]")
{
  Eigen::Vector3d pos(1.0, 2.0, 3.0);
  Eigen::Quaterniond quat(Eigen::AngleAxisd(M_PI / 4, Eigen::Vector3d::UnitZ()));
  Pose pose(pos, quat);
  
  Eigen::Affine3d transform = pose.to_transform();
  
  REQUIRE(approx_equal(transform.translation(), pos));
  REQUIRE(approx_equal(Eigen::Quaterniond(transform.rotation()), quat));
}

TEST_CASE("Pose: to_angle_axis_vector conversion", "[pose][conversion]")
{
  double angle = M_PI / 4;
  Eigen::Vector3d axis = Eigen::Vector3d::UnitZ();
  Eigen::Quaterniond quat(Eigen::AngleAxisd(angle, axis));
  Pose pose(Eigen::Vector3d::Zero(), quat);
  
  Eigen::Vector3d aa_vec = pose.to_angle_axis_vector();
  
  REQUIRE(approx_equal(aa_vec.norm(), angle, 1e-6));
  REQUIRE(approx_equal(aa_vec.normalized(), axis, 1e-6));
}

TEST_CASE("Pose: to_pose6d conversions", "[pose][conversion]")
{
  Eigen::Vector3d pos(1.0, 2.0, 3.0);
  double angle = M_PI / 4;
  Eigen::Vector3d axis = Eigen::Vector3d::UnitZ();
  Eigen::Quaterniond quat(Eigen::AngleAxisd(angle, axis));
  Pose pose(pos, quat);
  
  SECTION("to_pose6d_std")
  {
    std::vector<double> vec = pose.to_pose6d_std();
    REQUIRE(vec.size() == 6);
    REQUIRE(approx_equal(vec[0], pos.x()));
    REQUIRE(approx_equal(vec[1], pos.y()));
    REQUIRE(approx_equal(vec[2], pos.z()));
    
    Eigen::Vector3d rotation_vec(vec[3], vec[4], vec[5]);
    REQUIRE(approx_equal(rotation_vec.norm(), angle, 1e-6));
  }
  
  SECTION("to_pose6d_eigen")
  {
    Eigen::VectorXd vec = pose.to_pose6d_eigen();
    REQUIRE(vec.size() == 6);
    REQUIRE(approx_equal(vec(0), pos.x()));
    REQUIRE(approx_equal(vec(1), pos.y()));
    REQUIRE(approx_equal(vec(2), pos.z()));
    
    Eigen::Vector3d rotation_vec = vec.tail<3>();
    REQUIRE(approx_equal(rotation_vec.norm(), angle, 1e-6));
  }
}

TEST_CASE("Pose: Implicit conversion to Affine3d", "[pose][conversion]")
{
  Eigen::Vector3d pos(1.0, 2.0, 3.0);
  Eigen::Quaterniond quat(Eigen::AngleAxisd(M_PI / 4, Eigen::Vector3d::UnitZ()));
  Pose pose(pos, quat);
  
  Eigen::Affine3d transform = pose; // implicit conversion
  
  REQUIRE(approx_equal(transform.translation(), pos));
  REQUIRE(approx_equal(Eigen::Quaterniond(transform.rotation()), quat));
}

TEST_CASE("Pose: Assignment operator from Pose", "[pose][operators]")
{
  Pose pose1(Eigen::Vector3d(1.0, 2.0, 3.0), 
             Eigen::Quaterniond(Eigen::AngleAxisd(M_PI / 4, Eigen::Vector3d::UnitZ())));
  Pose pose2;
  
  pose2 = pose1;
  
  REQUIRE(approx_equal(pose2.get_position(), pose1.get_position()));
  REQUIRE(approx_equal(pose2.get_orientation(), pose1.get_orientation()));
}

TEST_CASE("Pose: Assignment operator from vector", "[pose][operators]")
{
  Pose pose;
  std::vector<double> vec = {1.0, 2.0, 3.0, 0.0, 0.0, M_PI / 4};
  
  pose = vec;
  
  Eigen::Vector3d expected_pos(1.0, 2.0, 3.0);
  Eigen::Quaterniond expected_quat(Eigen::AngleAxisd(M_PI / 4, Eigen::Vector3d::UnitZ()));
  
  REQUIRE(approx_equal(pose.get_position(), expected_pos));
  REQUIRE(approx_equal(pose.get_orientation(), expected_quat));
}

TEST_CASE("Pose: Multiplication operator Pose * Pose", "[pose][operators][multiplication]")
{
  // Create two poses
  Pose pose1(Eigen::Vector3d(1.0, 0.0, 0.0), 
             Eigen::Quaterniond(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ())));
  Pose pose2(Eigen::Vector3d(1.0, 0.0, 0.0), 
             Eigen::Quaterniond(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ())));
  
  Pose result = pose1 * pose2;
  
  // After 90-degree rotation, (1,0,0) becomes (0,1,0), then add (1,0,0)
  Eigen::Vector3d expected_pos(1.0, 1.0, 0.0);
  Eigen::Quaterniond expected_quat(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()));
  
  REQUIRE(approx_equal(result.get_position(), expected_pos, 1e-6));
  REQUIRE(approx_equal(result.get_orientation(), expected_quat, 1e-6));
}

TEST_CASE("Pose: Multiplication operator Pose * Vector3d", "[pose][operators][multiplication]")
{
  // 90-degree rotation around Z-axis and translation
  Pose pose(Eigen::Vector3d(1.0, 2.0, 3.0), 
            Eigen::Quaterniond(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ())));
  
  Eigen::Vector3d point(1.0, 0.0, 0.0);
  Eigen::Vector3d transformed = pose * point;
  
  // (1,0,0) rotated 90° around Z becomes (0,1,0), then add translation
  Eigen::Vector3d expected(1.0, 3.0, 3.0);
  
  REQUIRE(approx_equal(transformed, expected, 1e-6));
}

TEST_CASE("Pose: Multiplication operator Pose * Affine3d", "[pose][operators][multiplication]")
{
  Pose pose(Eigen::Vector3d(1.0, 0.0, 0.0), 
            Eigen::Quaterniond(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ())));
  
  Eigen::Affine3d transform = Eigen::Affine3d::Identity();
  transform.translate(Eigen::Vector3d(1.0, 0.0, 0.0));
  transform.rotate(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ()));
  
  Pose result = pose * transform;
  
  Eigen::Vector3d expected_pos(1.0, 1.0, 0.0);
  Eigen::Quaterniond expected_quat(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()));
  
  REQUIRE(approx_equal(result.get_position(), expected_pos, 1e-6));
  REQUIRE(approx_equal(result.get_orientation(), expected_quat, 1e-6));
}

TEST_CASE("Pose: Multiplication operator Pose * Matrix4d", "[pose][operators][multiplication]")
{
  Pose pose(Eigen::Vector3d(1.0, 0.0, 0.0), 
            Eigen::Quaterniond(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ())));
  
  Eigen::Affine3d transform = Eigen::Affine3d::Identity();
  transform.translate(Eigen::Vector3d(1.0, 0.0, 0.0));
  transform.rotate(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ()));
  
  Eigen::Matrix4d matrix = transform.matrix();
  Pose result = pose * matrix;
  
  Eigen::Vector3d expected_pos(1.0, 1.0, 0.0);
  Eigen::Quaterniond expected_quat(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()));
  
  REQUIRE(approx_equal(result.get_position(), expected_pos, 1e-6));
  REQUIRE(approx_equal(result.get_orientation(), expected_quat, 1e-6));
}

TEST_CASE("Pose: Left multiplication Matrix3d * Pose", "[pose][operators][multiplication]")
{
  Pose pose(Eigen::Vector3d(1.0, 0.0, 0.0), 
            Eigen::Quaterniond::Identity());
  
  Eigen::Matrix3d rotation = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  
  Pose result = rotation * pose;
  
  // Rotation should affect both position and orientation
  Eigen::Vector3d expected_pos(0.0, 1.0, 0.0);
  Eigen::Quaterniond expected_quat(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ()));
  
  REQUIRE(approx_equal(result.get_position(), expected_pos, 1e-6));
  REQUIRE(approx_equal(result.get_orientation(), expected_quat, 1e-6));
}

TEST_CASE("Pose: Left multiplication Affine3d * Pose", "[pose][operators][multiplication]")
{
  Pose pose(Eigen::Vector3d(1.0, 0.0, 0.0), 
            Eigen::Quaterniond::Identity());
  
  Eigen::Affine3d transform = Eigen::Affine3d::Identity();
  transform.translate(Eigen::Vector3d(1.0, 2.0, 3.0));
  transform.rotate(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ()));
  
  Pose result = transform * pose;
  
  // (1,0,0) rotated 90° becomes (0,1,0), then add (1,2,3)
  Eigen::Vector3d expected_pos(1.0, 3.0, 3.0);
  Eigen::Quaterniond expected_quat(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ()));
  
  REQUIRE(approx_equal(result.get_position(), expected_pos, 1e-6));
  REQUIRE(approx_equal(result.get_orientation(), expected_quat, 1e-6));
}

TEST_CASE("Pose: Left multiplication Matrix4d * Pose", "[pose][operators][multiplication]")
{
  Pose pose(Eigen::Vector3d(1.0, 0.0, 0.0), 
            Eigen::Quaterniond::Identity());
  
  Eigen::Affine3d transform = Eigen::Affine3d::Identity();
  transform.translate(Eigen::Vector3d(1.0, 2.0, 3.0));
  transform.rotate(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ()));
  
  Eigen::Matrix4d matrix = transform.matrix();
  Pose result = matrix * pose;
  
  Eigen::Vector3d expected_pos(1.0, 3.0, 3.0);
  Eigen::Quaterniond expected_quat(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ()));
  
  REQUIRE(approx_equal(result.get_position(), expected_pos, 1e-6));
  REQUIRE(approx_equal(result.get_orientation(), expected_quat, 1e-6));
}

TEST_CASE("Pose: Composition with identity pose", "[pose][operators]")
{
  Pose pose(Eigen::Vector3d(1.0, 2.0, 3.0), 
            Eigen::Quaterniond(Eigen::AngleAxisd(M_PI / 4, Eigen::Vector3d::UnitZ())));
  Pose identity;
  
  Pose result1 = pose * identity;
  Pose result2 = identity * pose;
  
  REQUIRE(approx_equal(result1.get_position(), pose.get_position(), 1e-6));
  REQUIRE(approx_equal(result1.get_orientation(), pose.get_orientation(), 1e-6));
  
  REQUIRE(approx_equal(result2.get_position(), pose.get_position(), 1e-6));
  REQUIRE(approx_equal(result2.get_orientation(), pose.get_orientation(), 1e-6));
}

TEST_CASE("Pose: Round-trip conversions", "[pose][conversion]")
{
  Eigen::Vector3d original_pos(1.0, 2.0, 3.0);
  Eigen::Quaterniond original_quat(Eigen::AngleAxisd(M_PI / 3, Eigen::Vector3d(1, 1, 1).normalized()));
  Pose original(original_pos, original_quat);
  
  SECTION("Through to_transform")
  {
    Eigen::Affine3d transform = original.to_transform();
    Pose reconstructed(transform);
    
    REQUIRE(approx_equal(reconstructed.get_position(), original_pos, 1e-6));
    REQUIRE(approx_equal(reconstructed.get_orientation(), original_quat, 1e-6));
  }
  
  SECTION("Through to_std_vector")
  {
    std::vector<double> vec = original.to_std_vector();
    Pose reconstructed(vec);
    
    REQUIRE(approx_equal(reconstructed.get_position(), original_pos, 1e-6));
    REQUIRE(approx_equal(reconstructed.get_orientation(), original_quat, 1e-6));
  }
  
  SECTION("Through to_pose6d_std")
  {
    std::vector<double> vec = original.to_pose6d_std();
    Pose reconstructed(vec);
    
    REQUIRE(approx_equal(reconstructed.get_position(), original_pos, 1e-6));
    REQUIRE(approx_equal(reconstructed.get_orientation(), original_quat, 1e-6));
  }
}

TEST_CASE("Pose: Euler angle conversion", "[pose][euler]")
{
  // Create a pose with a known rotation
  double angle = M_PI / 4;
  Eigen::Quaterniond quat(Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ()));
  Pose pose(Eigen::Vector3d::Zero(), quat);
  
  Eigen::Vector3d euler_zyz = pose.to_euler_angles("ZYZ");
  
  // Reconstruct quaternion from Euler angles
  Eigen::Matrix3d rotation = (Eigen::AngleAxisd(euler_zyz(0), Eigen::Vector3d::UnitZ()) *
                               Eigen::AngleAxisd(euler_zyz(1), Eigen::Vector3d::UnitY()) *
                               Eigen::AngleAxisd(euler_zyz(2), Eigen::Vector3d::UnitZ())).toRotationMatrix();
  Eigen::Quaterniond reconstructed_quat(rotation);
  
  REQUIRE(approx_equal(quat, reconstructed_quat, 1e-6));
}

TEST_CASE("Pose: String representation formats", "[pose][string]")
{
  Eigen::Vector3d pos(1.0, 2.0, 3.0);
  Eigen::Quaterniond quat(Eigen::AngleAxisd(M_PI / 4, Eigen::Vector3d::UnitZ()));
  Pose pose(pos, quat);
  
  SECTION("Default format")
  {
    std::string str = pose.to_string(PoseFormat::Default);
    REQUIRE(str.find("Position") != std::string::npos);
    REQUIRE(str.find("Orientation") != std::string::npos);
  }
  
  SECTION("Compact format")
  {
    std::string str = pose.to_string(PoseFormat::Compact);
    REQUIRE(str.find("[") != std::string::npos);
    REQUIRE(str.find("]") != std::string::npos);
  }
  
  SECTION("Verbose format")
  {
    std::string str = pose.to_string(PoseFormat::Verbose);
    REQUIRE(str.find("Pose:") != std::string::npos);
    REQUIRE(str.find("x:") != std::string::npos);
  }
  
  SECTION("Euler format")
  {
    std::string str = pose.to_string(PoseFormat::Euler);
    REQUIRE(str.find("Euler") != std::string::npos);
  }
  
  SECTION("AngleAxis format")
  {
    std::string str = pose.to_string(PoseFormat::AngleAxis);
    REQUIRE(str.find("Angle-Axis") != std::string::npos);
  }
  
  SECTION("Matrix format")
  {
    std::string str = pose.to_string(PoseFormat::Matrix);
    REQUIRE(str.find("Transform Matrix") != std::string::npos);
  }
}

TEST_CASE("Pose: Edge case - zero rotation vector", "[pose][edge_cases]")
{
  std::vector<double> pose_vec = {1.0, 2.0, 3.0, 0.0, 0.0, 0.0};
  Pose pose(pose_vec);
  
  REQUIRE(approx_equal(pose.get_position(), Eigen::Vector3d(1.0, 2.0, 3.0)));
  REQUIRE(approx_equal(pose.get_orientation(), Eigen::Quaterniond::Identity()));
}

TEST_CASE("Pose: Edge case - very small rotation vector", "[pose][edge_cases]")
{
  std::vector<double> pose_vec = {1.0, 2.0, 3.0, 1e-12, 1e-12, 1e-12};
  Pose pose(pose_vec);
  
  REQUIRE(approx_equal(pose.get_position(), Eigen::Vector3d(1.0, 2.0, 3.0)));
  REQUIRE(approx_equal(pose.get_orientation(), Eigen::Quaterniond::Identity()));
}

TEST_CASE("Pose: Chain of transformations", "[pose][integration]")
{
  // Create a chain: translate(1,0,0) -> rotate(90°,Z) -> translate(0,1,0) -> rotate(90°,Z)
  Pose pose1(Eigen::Vector3d(1.0, 0.0, 0.0), Eigen::Quaterniond::Identity());
  Pose pose2(Eigen::Vector3d::Zero(), 
             Eigen::Quaterniond(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ())));
  Pose pose3(Eigen::Vector3d(0.0, 1.0, 0.0), Eigen::Quaterniond::Identity());
  Pose pose4(Eigen::Vector3d::Zero(), 
             Eigen::Quaterniond(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ())));
  
  Pose result = pose1 * pose2 * pose3 * pose4;
  
  // Verify the result makes sense
  REQUIRE(result.get_position().norm() > 0);
  REQUIRE(approx_equal(result.get_orientation().norm(), 1.0, 1e-6));
}
