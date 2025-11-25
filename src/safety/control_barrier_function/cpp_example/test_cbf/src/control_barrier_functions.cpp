/*
 * control_barrier_functions.cpp
 *
 *  Created on: Feb 20, 2023
 *      Author: yik
 */

#include "control_barrier_functions.h"

ControlBarrierFunctions::ControlBarrierFunctions(double control_time)
{
  control_time_ = control_time;
  simulated_contact_force_ = 0;
  safe_contact_force_ = 0;

  //env
  mass_ = 1;
  b_model_ = 25;
  k_model_ = 50;
  uncertain_k_ = 0;
  uncertain_b_ = 0;
  real_k_ = k_model_ + uncertain_k_;
  real_b_ = b_model_ + uncertain_b_;
  theta_model_ << k_model_,b_model_;

  theta_real_(0,0) = real_k_;
  theta_real_(1,0) = real_b_;

  this->initialize();
}
ControlBarrierFunctions::~ControlBarrierFunctions()
{
}
void ControlBarrierFunctions::initialize()
{
  x_ = 0;
  dx_= 0;
  ddx_= 0;
  h_CBF_ = 0;
  f_min_ = 0;
  f_max_ = 0;
  states_.fill(0);
  delta_h_CBF_.fill(0);
  f_h_CBF_.fill(0);
  g_h_CBF_.fill(0);
  A_.fill(0);
  B_.fill(0);
  temp_B_.fill(0);

  n = 1;
  G.resize(n, n);
  g0.resize(n);

  CE.resize(n,0);
  ce0.resize(0);

  CI.resize(n,n);
  ci0.resize(n);

  u.resize(n);

  G[0][0] = 1;
  //equality constraints
  CE[0][0] = 0.0;

  // adaptive control barrier function
  delta_h_aCBF_.fill(0);
  delta_param_h_aCBF_.fill(0);
  f_h_aCBF_.fill(0);
  g_h_aCBF_.fill(0);
  F_h_aCBF_.fill(0);


  adaptive_gain_.fill(0);
  adaptive_gain_<< 1,0,
      0,1;


  alpha_aCBF_ = 0;

  //Robust adaptive control barrier function
  h_RaCBF_ = 0;
  alpha_RaCBF_ = 0;
  adaptive_gain_RaCBF_.fill(0);
  adaptive_gain_RaCBF_<< 1,0,
      0,1;
  delta_h_RaCBF_.fill(0);
  delta_param_h_RaCBF_.fill(0);
  f_h_RaCBF_.fill(0);
  g_h_RaCBF_.fill(0);
  F_h_RaCBF_.fill(0);
}

void ControlBarrierFunctions::CBFKelvinVoigtContactModel(double x, double dot_x, double nominal_ctrl_input,double alpha)
{
  alpha_ = alpha;
  //contact model

  h_CBF_ = (-b_model_*dot_x-k_model_*x-f_min_);
  delta_h_CBF_ << -k_model_,-b_model_;

  f_h_CBF_ << dot_x,
      (1/mass_)*(-b_model_*dot_x-k_model_*x);
  g_h_CBF_ << 0,
      (1/mass_);


  // Ax <= b
  A_ = -delta_h_CBF_*g_h_CBF_;
  temp_B_(0,0) = alpha_*h_CBF_;
  B_ = (delta_h_CBF_*f_h_CBF_+temp_B_);

  // Ax >= b
  A_ = -A_;
  B_ = -B_;

  g0[0] = -nominal_ctrl_input;

  //inequality constraints
  CI[0][0] = A_(0,0);
  ci0[0] = -B_(0,0);

  //solve_quadprog
  quadprogpp::solve_quadprog(G, g0, CE, ce0, CI, ci0, u);

  safe_contact_force_ = u[0];
  this->CalculateKelvinVoigtContactModelSimulatedForce(safe_contact_force_);
  //std::cout << "safe_contact_force_! \n" << safe_contact_force_ <<std::endl;
}
void ControlBarrierFunctions::ACBFKelvinVoigtContactModel(double x, double dot_x, double nominal_ctrl_input,double alpha_aCBF)
{
  //contact model
  static Eigen::Matrix<double, 2 ,1> theta_hat ={k_model_,b_model_};
  static Eigen::Matrix<double, 2 ,1> error_estimated_kb = {0,0};
  static Eigen::Matrix<double, 2 ,1> lambda = {k_model_,b_model_};
  static Eigen::Matrix<double, 2 ,1> tau = {0,0};
  static Eigen::Matrix<double, 2 ,1> dot_kb = {0,0};
  static double norm_error_estimated_kb = 0;
  static double model_acbf_contact_force = 0;
  static double minimum_eigen = 0;

  alpha_aCBF_ = alpha_aCBF;

  theta_real_(0,0) = real_k_;
  theta_real_(1,0) = real_b_;

  static bool flag_init_ = true;


  //init adaptive gain
  if(flag_init_)
  {
    //minimum adaptive gain
    error_estimated_kb = (theta_real_-theta_hat);
    norm_error_estimated_kb = error_estimated_kb.norm();
    model_acbf_contact_force = -theta_hat(0,0)*x-theta_hat(1,0)*dot_x;
    h_aCBF_ = (model_acbf_contact_force-f_min_);
    minimum_eigen = (norm_error_estimated_kb*norm_error_estimated_kb)/(2*h_aCBF_);

    adaptive_gain_ = adaptive_gain_*minimum_eigen;

    std::cout << "adaptive_gain_! \n" << adaptive_gain_ <<std::endl;
    flag_init_ = false;
  }

  // define cbf variables
  model_acbf_contact_force = -theta_hat(0,0)*x-theta_hat(1,0)*dot_x;
  h_aCBF_ = (model_acbf_contact_force-f_min_);
  delta_h_aCBF_ << -k_model_,-b_model_;

  delta_param_h_aCBF_ << -x, -dot_x;

  f_h_aCBF_ << dot_x,
      0;
  g_h_aCBF_ << 0,
      (1/mass_);
  F_h_aCBF_ <<0,0,
      (1/mass_)*(x),(1/mass_)*(dot_x);

  //adaptation law
  lambda = theta_hat - adaptive_gain_*(delta_param_h_aCBF_.transpose());

  if(h_aCBF_>= 0 && h_aCBF_<= 5)
  {
    tau = -(delta_h_aCBF_*F_h_aCBF_).transpose();
    dot_kb = adaptive_gain_*tau;
    theta_hat = theta_hat + dot_kb*control_time_;

    if(theta_hat(0,0) < 0)
      theta_hat(0,0) =  0;

    if(theta_hat(1,0) < 0)
      theta_hat(1,0) = 0;
  }

  //optimization
  // Ax <= b
  A_ = -delta_h_aCBF_*g_h_aCBF_;
  temp_B_(0,0) = alpha_aCBF_*h_aCBF_;
  B_ = (delta_h_aCBF_*f_h_aCBF_+delta_h_aCBF_*F_h_aCBF_*lambda+temp_B_);

  // Ax >= b
  A_ = -A_;
  B_ = -B_;

  //inequality constraints
  CI[0][0] = A_(0,0);
  ci0[0] = -B_(0,0);
  g0[0] = -nominal_ctrl_input;

  //solve_quadprog
  quadprogpp::solve_quadprog(G, g0, CE, ce0, CI, ci0, u);
  //

  safe_contact_force_ = u[0];
  this->CalculateKelvinVoigtContactModelSimulatedForce(safe_contact_force_);
}

void ControlBarrierFunctions::RACBFKelvinVoigtContactModel(double x, double dot_x, double nominal_ctrl_input,double alpha_RaCBF,double mp_error)
{
  //contact model
  static Eigen::Matrix<double, 2 ,1> theta_hat ={k_model_,b_model_};
  static Eigen::Matrix<double, 2 ,1> error_estimated_kb = {0,0};
  static Eigen::Matrix<double, 2 ,1> lambda = {k_model_,b_model_};
  static Eigen::Matrix<double, 2 ,1> tau = {0,0};
  static Eigen::Matrix<double, 2 ,1> dot_kb = {0,0};
  static Eigen::Matrix<double, 2 ,1> maximum_possible_error = {mp_error,mp_error};
  static double norm_maximum_possible_error = 0;
  static double model_acbf_contact_force = 0;
  static double minimum_eigen = 0;

  alpha_RaCBF_ = alpha_RaCBF;

  theta_real_(0,0) = real_k_;
  theta_real_(1,0) = real_b_;

  static bool flag_init_ = true;


  //init adaptive gain
  if(flag_init_)
  {
    //minimum adaptive gain
    norm_maximum_possible_error = maximum_possible_error.norm();
    model_acbf_contact_force = -theta_hat(0,0)*x-theta_hat(1,0)*dot_x;
    h_RaCBF_ = (model_acbf_contact_force-f_min_);
    minimum_eigen = (norm_maximum_possible_error*norm_maximum_possible_error)/(2*h_RaCBF_);

    adaptive_gain_RaCBF_ = adaptive_gain_RaCBF_*minimum_eigen;

    std::cout << "adaptive_gain_racbf_! \n" << adaptive_gain_RaCBF_ <<std::endl;
    flag_init_ = false;
  }

  // define cbf variables
  model_acbf_contact_force = -theta_hat(0,0)*x-theta_hat(1,0)*dot_x;
  h_RaCBF_ = (model_acbf_contact_force-f_min_);
  delta_h_RaCBF_ << -k_model_,-b_model_;

  delta_param_h_RaCBF_ << -x, -dot_x;

  f_h_RaCBF_ << dot_x,
      0;
  g_h_RaCBF_ << 0,
      (1/mass_);
  F_h_RaCBF_ <<0,0,
      (1/mass_)*(x),(1/mass_)*(dot_x);

  //adaptation law
  lambda = theta_hat - adaptive_gain_RaCBF_*(delta_param_h_RaCBF_.transpose());

  if(theta_hat(0,0) < 0)
    theta_hat(0,0) =  0;

  if(theta_hat(1,0) < 0)
    theta_hat(1,0) = 0;

  //optimization
  // Ax <= b
  A_ = -delta_h_RaCBF_*g_h_RaCBF_;
  temp_B_(0,0) = alpha_RaCBF_*(h_RaCBF_-(1/2)*maximum_possible_error.transpose()*adaptive_gain_RaCBF_*maximum_possible_error);
  B_ = (delta_h_RaCBF_*f_h_RaCBF_+delta_h_RaCBF_*F_h_RaCBF_*lambda+temp_B_);

  // Ax >= b
  A_ = -A_;
  B_ = -B_;

  //inequality constraints
  CI[0][0] = A_(0,0);
  ci0[0] = -B_(0,0);
  g0[0] = -nominal_ctrl_input;

  //solve_quadprog
  quadprogpp::solve_quadprog(G, g0, CE, ce0, CI, ci0, u);
  //

  safe_contact_force_ = u[0];
  this->CalculateKelvinVoigtContactModelSimulatedForce(safe_contact_force_);
}
void ControlBarrierFunctions::CalculateKelvinVoigtContactModelSimulatedForce(double safe_contact_force)
{
  ddx_ = (1/mass_)*(-theta_real_(0,0)*x_-theta_real_(1,0)*dx_+safe_contact_force);
  dx_ = dx_ + ddx_*control_time_;
  x_ = x_ + dx_*control_time_;

  simulated_contact_force_ = -theta_real_(0,0)*x_-theta_real_(1,0)*dx_;
}

double ControlBarrierFunctions::GetSimulatedRealForce()
{
  return simulated_contact_force_;
}
double ControlBarrierFunctions::GetSafeControlForce()
{
  return safe_contact_force_;
}
void ControlBarrierFunctions::SetSafetyForce(double force_min, double force_max)
{
  f_min_ = force_min;
  f_max_ = force_max;
}
Eigen::Matrix<double, 2 ,1> ControlBarrierFunctions::GetSimulatedStates()
{
  states_(0,0) = x_;
  states_(1,0) = dx_;

  return states_;
}


