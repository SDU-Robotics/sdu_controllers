/*
 * test_mosek.h
 *
 *  Created on: Feb 20, 2023
 *      Author: yik
 */

#ifndef SDU_MOSEK_TEST_TEST_MOSEK_INCLUDE_TEST_MOSEK_TEST_MOSEK_H_
#define SDU_MOSEK_TEST_TEST_MOSEK_INCLUDE_TEST_MOSEK_TEST_MOSEK_H_

#define LOOP_PERIOD 2e6 //Expressed in ticks // 2ms control time
#define TIMESPEC_ADD(A,B) /* A += B */ \
    do {                                   \
      (A).tv_sec += (B).tv_sec;          \
      (A).tv_nsec += (B).tv_nsec;        \
      if ( (A).tv_nsec >= 1000000000 ) { \
        (A).tv_sec++;                  \
        (A).tv_nsec -= 1000000000;     \
      }                                  \
    } while (0)

#define TIMESPEC_SUB(A,B) /* A -= B */ \
    do {                                   \
      (A).tv_sec -= (B).tv_sec;          \
      (A).tv_nsec -= (B).tv_nsec;        \
      if ( (A).tv_nsec < 0 ) {           \
        (A).tv_sec--;                  \
        (A).tv_nsec += 1000000000;     \
      }                                  \
    } while (0)
      //typedef actionlib::SimpleActionClient<test_client::belt_task_actionAction> Client;

#include <iostream>
#include <fstream>
#include <vector>
#include <sstream>
#include <map>
#include "control_barrier_functions.h"
#include "sdu_math/control_function.h"
//      //osqp
//#include "osqp.h"
//

//preempt rt system
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <pthread.h>
#include <sys/mman.h>
#include <limits.h>
#include <sched.h>
#include <signal.h>


//memory lock
#include <mutex>

#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"

#include <ur_rtde/rtde_receive_interface.h> // 로봇으로 데이터를 받을때 쓰는 헤더
#include <ur_rtde/rtde_control_interface.h> // 로봇으로 데이터를 줄때 쓰는 헤더 (control)

using std::cout; using std::cerr;
using std::endl; using std::string;
using std::ifstream; using std::ostringstream;
using std::istringstream;

#include <Eigen/Dense>

std::mutex m;
volatile bool exit_program = false;
//ros
ros::Publisher raw_force_torque_pub_;
std_msgs::Float64MultiArray raw_force_torque_msg_;

//control
double control_time;
std::shared_ptr<ControlBarrierFunctions> cbf_x;
std::shared_ptr<PID_function> force_x_compensator_;

double nominal_ctrl_output;
double desired_force;

#endif /* SDU_MOSEK_TEST_TEST_MOSEK_INCLUDE_TEST_MOSEK_TEST_MOSEK_H_ */
