/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: December 11, 2020
 *      Author: Mathilde Badoual
 **********************************************/

#include "pid_controller.h"
#include <vector>
#include <iostream>
#include <math.h>

using namespace std;

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kpi, double Kii, double Kdi, double output_lim_maxi, double output_lim_mini) {
   /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   **/
  kp_ = Kpi;
  ki_ = Kii;
  kd_ = Kdi;
  output_lim_maxi_ = output_lim_maxi;
  output_lim_mini_ = output_lim_mini;
}


void PID::UpdateError(double cte) {
   /**
   * TODO: Update PID errors based on cte.
   **/
  if(dt_ > __DBL_EPSILON__)
    derror_ = (cte - error_)/dt_;
  else
    derror_ = 0.0;
  
  ierror_ += cte * dt_; 
  error_ = cte;
   
}

double PID::TotalError() {
   /**
   * TODO: Calculate and return the total error
    * The code should return a value in the interval [output_lim_mini, output_lim_maxi]
   */
    double control = kp_ * error_ + ki_ * ierror_ + kd_ * derror_;
    if (control > output_lim_maxi_)
      control = output_lim_maxi_;
    else if (control < output_lim_mini_)
        control = output_lim_mini_;
  
    return control;
}

double PID::UpdateDeltaTime(double new_delta_time) {
   /**
   * TODO: Update the delta time with new value
   */
  dt_ = new_delta_time;
}