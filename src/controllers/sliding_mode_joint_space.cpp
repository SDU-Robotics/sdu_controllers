#include <sdu_controllers/controllers/sliding_mode_joint_space.hpp>
#include <sdu_controllers/math/math.hpp>
#include <utility>
#include <iostream>

namespace sdu_controllers::controllers
{
    SlidingModeControllerJoint::SlidingModeControllerJoint(
        double lambda, 
        Eigen::MatrixXd K1, 
        Eigen::MatrixXd K2,
        int order, 
        double dt)
        : lambda_(std::move(lambda)), 
          K1_(std::move(K2)), 
          K2_(std::move(K2)), 
          order_(std::move(order)),
          dt_(std::move(dt))
    {
        y_.setZero();
        omega_.setZero();
        domega_.setZero();
    }

    void SlidingModeControllerJoint::step(
        const Eigen::VectorXd& q_d, 
        const Eigen::VectorXd& dq_d, 
        const Eigen::VectorXd& ddq_d, 
        const Eigen::VectorXd& q, 
        const Eigen::VectorXd& dq)
    {
        // Define configuration and speed error
        Eigen::VectorXd q_err = q - q_d;   
        Eigen::VectorXd dq_err = dq - dq_d;

        // Define the sliding surface
        Eigen::VectorXd s = dq_err + lambda_ * q_err;
        Eigen::VectorXd s_sat = s;
        // domega_ = -K2_ * sdu_controllers::math::saturation(s, 0.01);

        for (int i = 0; i < s.size(); ++i)
        {
            s_sat[i] = sdu_controllers::math::saturation(s[i], 0.01);
        }

        domega_ = -K2_ * s_sat;

        // calculate acceleration output
        y_ = ddq_d - lambda_ * dq_err - K1_ * s.cwiseAbs().cwiseSqrt().asDiagonal() * s_sat + omega_;

        omega_ += dt_ * domega_;
    }

    Eigen::VectorXd SlidingModeControllerJoint::get_output()
    {
        return y_;
    }

}
