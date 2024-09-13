#include <sdu_controllers/controllers/admittance_controller_position.hpp>

using namespace Eigen;

namespace sdu_controllers::controllers
{
  AdmittanceControllerPosition::AdmittanceControllerPosition()
  {
    // Specification of impedance-parameters
    const Vector3d M_vec{ 22.5, 22.5, 22.5 };  // Positional mass
    const Vector3d K_vec{ 0, 0, 0 };           // Positional stiffness
    const Vector3d D_vec{ 70, 70, 70 };        // Positional damping
    M_ = M_vec.asDiagonal();
    K_ = K_vec.asDiagonal();
    D_ = D_vec.asDiagonal();
    const Vector3d Mo_vec{ 0.25, 0.25, 0.25 };  // Orientation mass
    const Vector3d Ko_vec{ 0, 0, 0 };           // Orientation stiffness
    const Vector3d Do_vec{ 3, 3, 3 };           // Orientation damping
    Mo_ = Mo_vec.asDiagonal();
    Ko_ = Ko_vec.asDiagonal();
    Do_ = Do_vec.asDiagonal();
  }

  AdmittanceControllerPosition::~AdmittanceControllerPosition() = default;

  void AdmittanceControllerPosition::step()
  {
  }

  void AdmittanceControllerPosition::reset()
  {
  }

  VectorXd AdmittanceControllerPosition::get_output()
  {
  }

}  // namespace sdu_controllers::controllers
