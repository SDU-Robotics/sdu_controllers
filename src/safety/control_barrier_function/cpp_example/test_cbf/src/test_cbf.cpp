/*
 * test_mosek.cpp
 *
 *  Created on: Feb 19, 2023
 *      Author: yik
 */

#include "test_cbf.h"

string readFileIntoString(const string& path) {
  auto ss = ostringstream{};
  ifstream input_file(path);
  if (!input_file.is_open()) {
    cerr << "Could not open the file - '"
        << path << "'" << endl;
    exit(EXIT_FAILURE);
  }
  ss << input_file.rdbuf();
  return ss.str();
}

void my_function(int sig)
{
  exit_program = true; // set flag
}

void *thread_func_robot_a ( void *param )
{
  struct timespec t_next, period, t_now, t_prev, t_diff, t_all, t_all_pre;

  /* period = 2 ms * thread_id */
  period.tv_sec = 0;
  period.tv_nsec = LOOP_PERIOD; // a x ms

  int counter = 1;

//  raw_force_torque_msg_.data.push_back(desired_force);
//  raw_force_torque_msg_.data.push_back(0);
//  raw_force_torque_msg_.data.push_back(-10.5);
//  raw_force_torque_pub_.publish(raw_force_torque_msg_);
//  raw_force_torque_msg_.data.clear();
//  ros::spinOnce();

  while(!exit_program)
  {
    clock_gettime ( CLOCK_MONOTONIC, &t_now );
    t_all_pre = t_now;
    m.lock();

    clock_gettime ( CLOCK_MONOTONIC, &t_now );
    t_next = t_now;
    t_prev = t_now;

    counter ++;
    //################################################## todo

    desired_force = 15*cos((counter*control_time)*2*M_PI*0.2) - 15;

//    if ((counter*control_time) < 10)
//      desired_force = -(15/10)*(counter*control_time);
//    else
//      desired_force =  - 15;

    force_x_compensator_->PID_calculate(desired_force,cbf_x->GetSimulatedRealForce(),0);
    nominal_ctrl_output = -(force_x_compensator_->get_final_output()+desired_force);


    cbf_x->CBFKelvinVoigtContactModel(cbf_x->GetSimulatedStates()(0,0),cbf_x->GetSimulatedStates()(1,0),nominal_ctrl_output,50);
    //cbf_x->ACBFKelvinVoigtContactModel(cbf_x->GetSimulatedStates()(0,0),cbf_x->GetSimulatedStates()(1,0),nominal_ctrl_output,30);
    //cbf_x->RACBFKelvinVoigtContactModel(cbf_x->GetSimulatedStates()(0,0),cbf_x->GetSimulatedStates()(1,0),nominal_ctrl_output,600,10);

    std::cout << "Contact Force:: \n" << cbf_x->GetSimulatedRealForce() <<std::endl;
    raw_force_torque_msg_.data.push_back(desired_force);
    raw_force_torque_msg_.data.push_back(cbf_x->GetSimulatedRealForce());
    raw_force_torque_msg_.data.push_back(-10.5);
    raw_force_torque_pub_.publish(raw_force_torque_msg_);
    raw_force_torque_msg_.data.clear();


    //#################################################

    clock_gettime ( CLOCK_MONOTONIC, &t_now );
    TIMESPEC_SUB(t_now,t_prev);
    t_diff  = t_now;

    m.unlock();


    TIMESPEC_ADD (t_next, period);
    clock_nanosleep ( CLOCK_MONOTONIC, TIMER_ABSTIME, &t_next, NULL );

    clock_gettime ( CLOCK_MONOTONIC, &t_now );
    t_all = t_now;

    //if(((t_all.tv_sec - t_all_pre.tv_sec) + (t_all.tv_nsec - t_all_pre.tv_nsec))*0.000001  >= 2.01)
     cout << "  Elapsed time A : "<< ((t_all.tv_sec - t_all_pre.tv_sec) + (t_all.tv_nsec - t_all_pre.tv_nsec))*0.000001 << endl;
    //if(counter > csv_contents.size()-1)
    //  exit_program = true;

  }
  return NULL;
}



int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_mosek");
  ros::NodeHandle nh;
  signal(SIGINT, my_function);

  //ros
  raw_force_torque_pub_ = nh.advertise<std_msgs::Float64MultiArray>("/raw_force_torque_data", 100);

  control_time = 0.002;
  cbf_x = std::make_shared<ControlBarrierFunctions>(control_time);
  force_x_compensator_ = std::make_shared<PID_function>(control_time, 20, -20, 0, 0, 0, 0.0000001, -0.0000001, 0.01);
  force_x_compensator_->set_pid_gain(0.5,0,0); // kp ki kd
  desired_force = -15;
  //nominal_ctrl_output = force_x_compensator_->get_final_output();

  cbf_x->SetSafetyForce(-10.5,0);
  std::cout << "start!" << std::endl;


  //preempt rt
  int policy;
  struct sched_param prio;
  pthread_attr_t attr_robot_a;
  pthread_t loop_robot_a;
  pthread_attr_init( &attr_robot_a);

  pthread_attr_setstacksize(&attr_robot_a, PTHREAD_STACK_MIN);

  policy = SCHED_FIFO;
  pthread_attr_setschedpolicy( &attr_robot_a, policy);

  prio.sched_priority = 90; // priority range should be btw -20 to +19
  pthread_attr_setschedparam(&attr_robot_a,&prio);
  pthread_attr_setinheritsched( &attr_robot_a, PTHREAD_EXPLICIT_SCHED);

  if ( pthread_create(&loop_robot_a, &attr_robot_a, thread_func_robot_a, (void *)(1)) ){
    perror ( "Error: pthread1_create" );
    return 1;
  }

  while(!exit_program)
  {
    usleep(0.1);
  }

  /* wait for threads to finish */
  pthread_join ( loop_robot_a, NULL );


  return 0;


}
