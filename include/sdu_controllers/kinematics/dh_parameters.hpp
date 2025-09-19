#ifndef SDU_CONTROLLERS_DH_PARAMETERS_HPP
#define SDU_CONTROLLERS_DH_PARAMETERS_HPP

namespace sdu_controllers::kinematics
{
  class DHParam
  {
   public:
    double a;                // link length
    double alpha;            // link twist
    double d;                // link offset
    double theta;            // joint angle
    bool is_joint_revolute;  // true if the joint is revolute, false if prismatic
  };
}  // namespace sdu_controllers::kinematics

#endif