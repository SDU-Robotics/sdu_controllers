import sdu_controllers
import numpy as np
import pytest


def test_sdu_controllers():
  pass

def test_ur_model_loads():
    robot_model = sdu_controllers.models.URRobotModel(sdu_controllers.models.ur5e)

def test_ur5e_forward_kinematics():
    """Test forward kinematics for UR5e robot
    
    This test checks that the forward kinematics solver is accessible
    and can compute valid transformation matrices.
    """
    robot_model = sdu_controllers.models.URRobotModel(sdu_controllers.models.ur5e)
    
    # Get the forward kinematics solver
    fk_solver = robot_model.get_fk_solver()
    
    # Test with zero configuration
    q_zero = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    T_zero = fk_solver.forward_kinematics(q_zero)
    
    # The end-effector should have a valid transformation matrix (4x4)
    assert T_zero.shape == (4, 4), f"Expected shape (4, 4), got {T_zero.shape}"
    
    # Check that it's a valid homogeneous transformation matrix
    # Bottom row should be [0, 0, 0, 1]
    assert np.allclose(T_zero[3, :], [0, 0, 0, 1]), "Bottom row should be [0, 0, 0, 1]"
    
    # Test with a different configuration
    q_test = np.array([0.5, -0.5, 1.0, 0.2, -0.1, 0.3])
    T_test = fk_solver.forward_kinematics(q_test)
    
    # Should also be a valid transformation matrix
    assert T_test.shape == (4, 4), f"Expected shape (4, 4), got {T_test.shape}"
    assert np.allclose(T_test[3, :], [0, 0, 0, 1]), "Bottom row should be [0, 0, 0, 1]"
    
    # Test that different joint configurations yield different end-effector poses
    assert not np.allclose(T_zero[:3, 3], T_test[:3, 3]), "Different joint configs should give different positions"


def test_ur5e_gravity_compensation():
    """Test gravity compensation computation for UR5e robot
    
    This test verifies that the UR5e model can compute gravity torques
    needed to hold the robot in various configurations.
    """
    robot_model = sdu_controllers.models.URRobotModel(sdu_controllers.models.ur5e)
    
    # Test zero configuration for gravity
    q = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    dq = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    ddq = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    he = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    
    # Compute gravity torques at zero configuration
    tau_gravity = robot_model.inverse_dynamics(q, dq, ddq, he)
    
    # Should be 6D vector
    assert len(tau_gravity) == 6, f"Expected 6 joint torques, got {len(tau_gravity)}"
    
    # Torques should be finite
    assert np.all(np.isfinite(tau_gravity)), "Computed torques contain NaN or Inf values"
    
    # At zero configuration, some significant gravity torques should be present
    # (at least one joint should need non-zero torque to hold the arm up)
    assert not np.allclose(tau_gravity, 0.0), "Expected non-zero gravity torques at zero configuration"

