/*
 * control_barrier_functions.h
 *
 *  Created on: Feb 20, 2023
 *      Author: yik
 */

#ifndef SDU_MOSEK_TEST_TEST_MOSEK_INCLUDE_TEST_MOSEK_CONTROL_BARRIER_FUNCTIONS_H_
#define SDU_MOSEK_TEST_TEST_MOSEK_INCLUDE_TEST_MOSEK_CONTROL_BARRIER_FUNCTIONS_H_

#include <Eigen/Dense>
#include <iostream>
#include "QuadProg++/Array.hh"
#include "QuadProg++/QuadProg++.hh"
//#include "osqp++.h"


////      //mosek
//#include "fusion.h"
//
//using namespace mosek::fusion;
//using namespace monty;

// CGAL
//#include <cassert>
//#include <CGAL/QP_models.h>
//#include <CGAL/QP_functions.h>
//// choose exact integral type
//#ifdef CGAL_USE_GMP
//#include <CGAL/Gmpz.h>
//typedef CGAL::Gmpz ET;
//#else
//#include <CGAL/MP_Float.h>
//typedef CGAL::MP_Float ET;
//#endif
//// program and solution types
//typedef CGAL::Quadratic_program<double> Program;
//typedef CGAL::Quadratic_program_solution<ET> Solution;

class ControlBarrierFunctions
{
public:
  ControlBarrierFunctions(double control_time);
  ~ControlBarrierFunctions();
  void CalculateKelvinVoigtContactModelSimulatedForce(double safe_contact_force);
  void CBFKelvinVoigtContactModel(double x, double dot_x, double nominal_ctrl_input,double alpha_CBF);
  void ACBFKelvinVoigtContactModel(double x, double dot_x, double nominal_ctrl_input,double alpha_aCBF);
  void RACBFKelvinVoigtContactModel(double x, double dot_x, double nominal_ctrl_input,double alpha_RaCBF,double mp_error);


  double GetSimulatedRealForce();
  double GetSafeControlForce();
  Eigen::Matrix<double, 2 ,1> GetSimulatedStates();

  void SetSafetyForce(double force_min, double force_max);
  void initialize();


private:
  double control_time_;

  //theta
  Eigen::Matrix<double, 2 ,1> theta_real_;


  //modeled_force
  double simulated_contact_force_;

  //force limit
  Eigen::Matrix<double, 2 ,1> safety_limit_; //force min max

  //control output
  double safe_contact_force_;

  // control barrier function
  double h_CBF_;
  double f_min_;
  double f_max_;

  Eigen::Matrix<double, 1 ,2> delta_h_CBF_;
  Eigen::Matrix<double, 2 ,1> f_h_CBF_;
  Eigen::Matrix<double, 2 ,1> g_h_CBF_;
  Eigen::Matrix<double, 2 ,2> F_h_CBF_;

  Eigen::Matrix<double, 1 ,1> A_;
  Eigen::Matrix<double, 1 ,1> B_;
  Eigen::Matrix<double, 1 ,1> temp_B_;

  quadprogpp::Matrix<double> G, CE, CI;
  quadprogpp::Vector<double> g0, ce0, ci0, u;
  int n, m, p;
  double sum;
  char ch;

  double alpha_;

  // adaptive control barrier function
  double h_aCBF_;
  double alpha_aCBF_;
  Eigen::Matrix<double, 2 ,2> adaptive_gain_;

  Eigen::Matrix<double, 1 ,2> delta_h_aCBF_;
  Eigen::Matrix<double, 1 ,2> delta_param_h_aCBF_;
  Eigen::Matrix<double, 2 ,1> f_h_aCBF_;
  Eigen::Matrix<double, 2 ,1> g_h_aCBF_;
  Eigen::Matrix<double, 2 ,2> F_h_aCBF_;

  // robust adaptive control barrier function
  double h_RaCBF_;
  double alpha_RaCBF_;
  Eigen::Matrix<double, 2 ,2> adaptive_gain_RaCBF_;

  Eigen::Matrix<double, 1 ,2> delta_h_RaCBF_;
  Eigen::Matrix<double, 1 ,2> delta_param_h_RaCBF_;
  Eigen::Matrix<double, 2 ,1> f_h_RaCBF_;
  Eigen::Matrix<double, 2 ,1> g_h_RaCBF_;
  Eigen::Matrix<double, 2 ,2> F_h_RaCBF_;



  //simulation states
  double b_model_;
  double k_model_;
  double uncertain_k_;
  double uncertain_b_;
  double real_k_;
  double real_b_;
  Eigen::Matrix<double, 2 ,1> theta_model_;

  double mass_;
  double x_;
  double dx_;
  double ddx_;
  Eigen::Matrix<double, 2 ,1>  states_;
};





#endif /* SDU_MOSEK_TEST_TEST_MOSEK_INCLUDE_TEST_MOSEK_CONTROL_BARRIER_FUNCTIONS_H_ */
