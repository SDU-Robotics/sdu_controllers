#include <Eigen/Dense>
#include <cmath>
#include <array>
#include <sdu_controllers/hal/eurofusion.hpp>
#include <sdu_controllers/utils/utility.hpp>
#include <vector>

namespace sdu_controllers::hal
{

constexpr double PI = 3.14159265358979323846;
constexpr double kJoint4MinGearRatioMagnitude = 1e-6;

EurofusionRobot::EurofusionRobot()
{

    // Do some init stuff? 
}

Eigen::VectorXd EurofusionRobot::generalized_tau_2_motor_tau(const Eigen::VectorXd& tau_g, const Eigen::VectorXd& q, const Eigen::VectorXd& qd)
{
    Eigen::VectorXd motor_torques(8);

    motor_torques(0) = joint1_gearing(tau_g(0));
    motor_torques(1) = joint2_gearing(tau_g(1), q(1));
    motor_torques(2) = joint3_gearing(tau_g(2));
    motor_torques(3) = joint4_gearing(tau_g(3), q(3));
    motor_torques(4) = joint5_gearing(tau_g(4));
    motor_torques(5) = joint6_gearing(tau_g(5));

    const Eigen::Vector2d motor_7 = joint7_gearing(tau_g(6), q(6), qd(6));
    motor_torques(6) = motor_7(0);  // left
    motor_torques(7) = motor_7(1);  // right

    return motor_torques;

}


double EurofusionRobot::joint1_gearing(double tau_1)
{
    // TODO
    return tau_1;

}
double EurofusionRobot::joint2_gearing(double tau_2,double q_2)
{
    // Treating this as a triangle the following relationg can be derived:
    // x = A tan(q_2)    // Position along lead skrew
    // x_dot = A sec^2(q_2) q_2_dot     // Velocity along lead skrew
    // F_l = tau_2 / ( A sec^2(q_2) )   // linear force along lead skrew given generalized torque
    // tau_m = F_l * alpha / ( 2 * pi * N )     // Motor torque gien the the lead (alpha) and the motor gearing (N)

    double A = 5.523;        // Base lenght of the triangle [m]
    double alpha = 20.0 / 1000.0;     // lead [m/rev]
    double N = 1.0;          // Motor gearing 
    
    double sec = 1 / std::cos(q_2);

    double F_l = tau_2 / ( A * sec * sec);

    return F_l * alpha / ( 2 * PI * N );

}
double EurofusionRobot::joint3_gearing(double tau_3)
{
    // This is a simple gearing + lead skrew
    double alpha = 20.0 / 1000.0;  // [m/rev]
    double N = 1.0;

    return tau_3 * alpha / ( 2 * PI * N);
}
double EurofusionRobot::joint4_gearing(double tau_4, double q_4)
{
    // Tilting mechanism: a crank (driven by joint 4, angle q_4) moves a
    // vertical slider, which in turn drives a two-bar linkage ending in the
    // tilt output. The instantaneous gear ratio
    //     N(q_4) = d(theta_tilt)/d(q_4)
    // was derived symbolically (Symbolic Math Toolbox: chain rule through
    // the two-stage kinematics, then reduced via common-subexpression
    // elimination) with every mechanism dimension substituted numerically:
    //     l1=0.035 m, l2=1.96 m, d=0.0 m, l3=0.17 m, l4=1.0 m,
    //     px=-0.9835 m, py=-1.925 m, d_hinge=0.21 m
    // q_4 [rad] is the only remaining variable. Valid for the linkage
    // branch that stays active across the full 0..2*pi crank rotation for
    // these dimensions (verified numerically; re-check if the geometry
    // above changes).
    //
    // Power balance for an ideal (lossless, rigid) linkage:
    //     tau_4 * qd_4 = tau_m * theta_tilt_dot  =>  tau_m = tau_4 / N(q_4)
    // N(q_4) passes through ~0 twice per revolution (mechanism dead
    // points); guard against dividing by (near) zero there.
 
    const double x0 = std::cos(q_4);
    const double x1 = 2 * q_4;
    const double x2 = std::sin(x1) / std::sqrt(1.0001594896331738 - 0.0001594896331738437 * std::cos(x1));
    const double x3 = std::sin(q_4);
    const double x4 = 0.035000000000000003 * x3 - 1.96 * std::sqrt(0.00031887755102040814 * std::pow(x3, 2) + 0.99968112244897955) + 1.7150000000000001;
    const double x5 = std::pow(x4, 2);
    const double x6 = x5 + 0.96727225000000006;
    const double x7 = 1.0 / x6;
 
    const double N = x4 * (0.070000000000000007 * x0 - 0.00062509967307295658 * x2) *
                          (-x7 * (0.25 * x5 + 0.48459306250000006) + 0.5) /
                          (std::sqrt(x6) * std::sqrt(-0.25 * x7 * std::pow(x5 + 1.9383722500000002, 2) + 1)) +
                      x7 * (0.034422500000000002 * x0 - 0.00030739276423362639 * x2);
 
    if (std::fabs(N) < kJoint4MinGearRatioMagnitude)
    {
        // At/near a dead point: no finite motor torque corresponds to this
        // generalized torque at this instant.
        // TODO: decide what the controller should do here (e.g. hold last
        // command, saturate, or flag a fault) rather than silently returning 0.
        return 0.0;
    }
 
    return tau_4 * N;
}

double EurofusionRobot::joint5_gearing(double tau_5)
{
    // Just gearing ( pulley + motor gearing )
    double N = 1.0; // Combined gearing

    return tau_5 / N;
}
double EurofusionRobot::joint6_gearing(double tau_6)
{
    // Just gearing ( motor gearing )
    double N = 1.0; // motor gearing

    return tau_6 / N;
}
Eigen::Vector2d EurofusionRobot::joint7_gearing(double tau_7, double q_7, double qd_7)
{
    // Treat a triangle to derive the following relations
    // c = sqrt( a^2 + b^2 - 2abcos(beta) )    
    // beta = C + q_7
    // F_l = tau_7 * c / ( ab sin(beta) )
    // tau_m = F_l * alpha / ( 2 * pi * N )     // Motor torque gien the the lead (alpha) and the motor gearing (N)

    double alpha = 15.0 / 1000.0;     // lead [m/rev]
    double N = 1.0;          // Motor gearing 

    double a = 0.66022799092435; // [m]
    double b = 0.8311015576034;  // [m]
    double beta = 1.13883 + q_7;    // [rad]

    double c = std::sqrt(a*a + b*b - 2*a*b*std::cos(beta));

    double F_l = tau_7 * c / ( a*b*std::sin(beta));

    double tau_m = F_l * alpha / ( 2 * PI * N );
    
    double mL = 0; // motor left
    double mR = 0; // motor right
    // Determine which motor should run, since they are only allowed to run when pushing not pulling
    if (qd_7 > 0) 
    {
        mL = tau_m;
    } 
    else
    {
        // TODO: What should actaully happen at qd_7 = 0?
        mR = tau_m;
    } 

    return Eigen::Vector2d(mL,mR);

}

} // namespace sdu_controllers::hal