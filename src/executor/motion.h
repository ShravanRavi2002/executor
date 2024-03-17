//========================================================================
//  This software is free: you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License Version 3,
//  as published by the Free Software Foundation.
//
//  This software is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public License
//  Version 3 in the file COPYING that came with this distribution.
//  If not, see <http://www.gnu.org/licenses/>.
//========================================================================
// Copyright 2013 joydeepb@ri.cmu.edu
// Robotics Institute, Carnegie Mellon University
//
// Basic motion computations.

#include "shared/math/cobot_geometry.h"

#ifndef _MOTION_H_
#define _MOTION_H_

// Acceleration limits for 1D trapezoidal motion profiling.
struct AccelLimits{
  double max_accel;  // acceleration limit from 0 to max_vel
  double max_deccel; // acceleration limit from max_vel to 0
  double max_vel;    // maximum velocity along dimension

  AccelLimits() : max_accel(0), max_deccel(0), max_vel(0) {}
  AccelLimits(double a,double d,double v) {
    max_accel = a;
    max_deccel = d;
    max_vel = v;
  }

  // return new limits with all parameters scaled by <f>
  AccelLimits operator*(double f) const
  {AccelLimits r(max_accel*f,max_deccel*f,max_vel*f); return(r);}

  // scale all parameters by <f> in-place
  AccelLimits &operator*=(double f);

  // decide whether two limits are equal or not
  bool operator==(const AccelLimits otherLimits) const;

  // set limits to <al> with all parameters scaled by <f>
  AccelLimits &set(const AccelLimits &al,double f);
};

// Motion model for a 2D omnidirectional robot
struct MotionModel {
  // The translation acceleration limits.
  AccelLimits trans;
  // The angular acceleration limits.
  AccelLimits angular;
  MotionModel() {}
  MotionModel(const AccelLimits& _trans,
              const AccelLimits& _angular) :
      trans(_trans), angular(_angular) {}
};

/**
 * 1D Trapezoidal Motion Profiling
 * Compute the 1D velocity command for the next time step in order to
 * traverse total distance dx starting with velocity v0 at the
 * current time step, and terminating with max velocity v1_max.
 * Pass by ref var time returns the time to execute the complete
 * motion trajectory
 **/
class SubCommand{
public:
  double dx;   // Signed distance to target at the begining of this sub-command.
  double v0;   // Starting velocity at the begining of this sub-command.
  double a;    // Constant acceleration for this sub-command.
  double time; // Time for this sub-command duration.
  double l;    // Signed distance traversed while executing this sub-command.
  const char *name; // Descriptive string for debugging.

public:
  void advance() {
    if(time < 0.0) {
      time = 0.0;
    }
    double v1 = v0 + a * time;
    l = 0.5 * (v0 + v1) * time;
    dx -= l;
    v0 = v1;
  }

  void print() {
    printf("    %10.4f %10.4f %10.4f %8.5f [%s]\n",dx,v0,a,time,name);
  }
};

// Compute 1D trapezoidal profiling with initial velocity v0, delta location dx,
// and maximum final speed v1_max. returns the next command velocity for the
// next frame_period.
double CalcMotion1D(
    const AccelLimits &limits,
    double frame_period,
    double dx_,
    double v0_,
    double v1_max,
    double &time,
    SubCommand* cmd_schedule);

// Compute 2D near time-optimal control with initial state loc0, vel0, to reach
// final state target with max velocity target_vel.
vector2f CalcMotion2D(
    const AccelLimits& limits,
    const float lookahead,
    const vector2f& loc0,
    const vector2f& vel0,
    const vector2f& target,
    const vector2f& max_target_vel,
    double &time,
    vector2f &frame,
    SubCommand* cmd_schedule_x,
    SubCommand* cmd_schedule_y);

// Compute what the next translation velocity command should be, in order to
// reach the target specified in the relative coordinate frame.
void CalcNextTransVelocity(
    const vector2f& target, const vector2f& current_vel,
    const float max_acc, const float max_speed, const float frame_period,
    vector2f* velocity_next);

// Compute what the next rotation velocity command should be, in order to
// reach the angular target specified in the relative coordinate frame.
void CalcNextRotVelocity(
  const float target, const float current_vel,
  const float max_acc, const float max_speed, const float frame_period,
  float* velocity_next);

#endif  // _MOTION_H_c