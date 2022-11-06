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

void PID::Init(double Kpi, double Kii, double Kdi, double output_lim_max, double output_lim_min) {
   /**
   * Initialize PID coefficients (and errors, if needed)
   **/
   this->Kpi = Kpi;
   this->Kii = Kii;
   this->Kdi = Kdi;
  
   this->output_lim_max = output_lim_max;
   this->output_lim_min = output_lim_min;
  
   p_error = 0.0;
   i_error = 0.0;
   d_error = 0.0;
  
   dt = 0.0;
}


void PID::UpdateError(double cte) {
   /**
   * Update PID errors based on cte.
   **/
   p_error = cte;
   if (dt>0)
   {    
     d_error = (cte - p_error) / dt; 
   } else {    
     d_error = 0.0;
   }
  
   p_error = cte;
   i_error += cte * dt;  
  
}

double PID::TotalError() {
   /**
   * Calculate and return the total error
    * The code should return a value in the interval [output_lim_min, output_lim_max]
   */
  
    double control;
    control = (Kpi*p_error + Kii*i_error + Kdi*d_error);
    if (control < output_lim_min) {
       control = output_lim_min;
    } else if (control > output_lim_max) {
       control = output_lim_max;
    }
    return control;
}

double PID::UpdateDeltaTime(double new_delta_time) {
   /**
   * Update the delta time with new value
   */
   dt = new_delta_time;
   return dt;
}