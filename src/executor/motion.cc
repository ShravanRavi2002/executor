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

#include "motion.h"
#include "shared/math/cobot_geometry.h"

AccelLimits &AccelLimits::operator*=(double f)
{
  max_accel  *= f;
  max_deccel *= f;
  max_vel    *= f;
  return(*this);
}

bool AccelLimits::operator==(const AccelLimits otherLimits) const
{
  return (max_accel == otherLimits.max_accel &&
  max_deccel == otherLimits.max_deccel &&
  max_vel == otherLimits.max_vel);
}


AccelLimits &AccelLimits::set(const AccelLimits &al,double f)
{
  max_accel  = al.max_accel  * f;
  max_deccel = al.max_deccel * f;
  max_vel    = al.max_vel    * f;
  return(*this);
}

double CalcMotion1D(const AccelLimits &limits,
                    double frame_period, double dx_,double v0_,
                    double v1_max, double &time,
                    SubCommand* cmd_schedule) {
  int num;
  SubCommand cur,cmd[6];

  cur.dx = dx_;
  cur.v0 = v0_;
  cur.a = cur.time = 0.0;
  cur.name = "";
  num = 0;

  // stop first if currently going the wrong way
  if(!sign_eq(cur.dx, cur.v0)){
    cur.time = fabs(cur.v0) / limits.max_deccel;
    cur.a    = -sign(cur.v0) * limits.max_deccel;
    cur.name = "stop wrong way";

    cmd[num++] = cur;
    cur.advance();
  }else{
    // stop if we are going to overshoot
    double stop_dist = sq(cur.v0) / ( 2*limits.max_deccel );
    if (stop_dist > fabs(cur.dx) && fabs(v1_max)<0.001)
    {
      cur.time = fabs( cur.v0 ) / limits.max_deccel;
      cur.a    = -sign( cur.v0 ) * limits.max_deccel;
      cur.name = "stop overshoot";

      cmd[num++] = cur;
      cur.advance();
    }
  }

  // slow down if currently exceeding max speed
  if(fabs(cur.v0) > limits.max_vel){
    cur.time = (fabs(cur.v0) - limits.max_vel) / limits.max_deccel;
    cur.a    = -sign(cur.v0) * limits.max_deccel;
    cur.name = "slow overspeed";

    cmd[num++] = cur;
    cur.advance();
  }


  // calculate top speed assuming acceleration and decceleration to reach target

  double vtop2 =
      sq(cur.v0) +
      limits.max_accel/(limits.max_accel + limits.max_deccel) *
      fmax(0, sq(v1_max) - sq(cur.v0) + 2.0 * limits.max_deccel * fabs(cur.dx));
  double vtop = sqrt(vtop2);

  // check if we will hit the speed limit
  if(vtop < limits.max_vel){
    // accelerate and decellerate to reach goal
    if(cur.v0 <= vtop){
      // accel
      cur.time = (vtop - fabs(cur.v0)) / limits.max_accel;
      cur.a    = sign(cur.dx) * limits.max_accel;
      cur.name = "accel triangle";

      cmd[num++] = cur;
      cur.advance();
    }

    // deccel
    cur.time =  (vtop - fabs(v1_max)) / limits.max_deccel;
    cur.a    = -sign(cur.dx) * limits.max_deccel;
    cur.name = "deccel triangle";
    cmd[num++] = cur;
    cur.advance();
  }else{
    // figure out times for all three segments
    double t_accel  = (limits.max_vel - fabs(cur.v0)) / limits.max_accel;
    double t_deccel = (limits.max_vel - fabs(v1_max)) / limits.max_deccel;
    double dx1 = fabs(cur.dx) -
        0.5*(cur.v0 + limits.max_vel) * t_accel -
        0.5*(limits.max_vel + v1_max) * t_deccel;    //distance travelled at cruise speed
    double t_cruise = dx1 / limits.max_vel;

    // generate commands
    // accel
    cur.time = t_accel;
    cur.a    = sign(cur.dx) * limits.max_accel;
    cur.name = "accel trap";

    cmd[num++] = cur;
    cur.advance();

    // cruise
    cur.time = t_cruise;
    cur.a    = 0.0;
    cur.name = "cruise trap";

    cmd[num++] = cur;
    cur.advance();

    // deccel
    cur.time = t_deccel;
    cur.a    = -sign(cur.dx) * limits.max_deccel;
    cur.name = "deccel trap";

    cmd[num++] = cur;
    cur.advance();
  }

  // total time is simply the sum of components
  time = 0.0;
  for(int i=0; i<num; i++){
    time += cmd[i].time;
  }

  // look into the command schedule for the final value
  double t_left = frame_period;
  double t=0.0;
  double v1 = 0.0;

  int i=0;
  while(i<num && t_left>0.0){
    t = cmd[i].time;
    if(t > t_left){
      t = t_left;
      t_left = 0.0;
    }else{
      t_left -= t;
    }
    v1 = cmd[i].v0 + cmd[i].a*t;
    i++;
  }
  if (cmd_schedule != NULL) {
    for(int i = 0; i < num; ++i) {
      cmd_schedule[i] = cmd[i];
    }
    if (num < 6)
      cmd_schedule[num].time = -1.0;
  }
  return(v1 );//+ v1_max);
}

vector2f CalcMotion2D(const AccelLimits& limits,
                      const float lookahead,
                      const vector2f& loc0,
                      const vector2f& vel0,
                      const vector2f& target,
                      const vector2f& max_target_vel,
                      double &time,
                      vector2f &frame,
                      SubCommand* cmd_schedule_x,
                      SubCommand* cmd_schedule_y)
{
  // FunctionTimer timer(__PRETTY_FUNCTION__);
  static const bool debug = false;
  static const float MinTargVel = 1.0;
  AccelLimits lim = limits;

  float ts = max_target_vel.sqlength();
  if(ts > sq(MinTargVel)){
    frame = max_target_vel / sqrt(ts);
  }

  vector2f dir = frame.norm();
  vector2f v  = dir.project_in(vel0);
  vector2f d  = dir.project_in(target - loc0);
  vector2f tv = dir.project_in(max_target_vel);

  if(fabs(tv.x) < MinTargVel) tv.x = 0;
  if(fabs(tv.y) < MinTargVel) tv.y = 0;

  vector2f rv(0.0f, 0.0f);
  double tx(0.0), ty(0.0), err(0.0);
  vector2d w(0.0, 0.0); // normalized x/y balance weights
  double a0(0.0), a1(M_PI_2); // angle of weight vector bracket

  if(debug) printf("Motion2D\n");

  for(int i=0; i<12; i++){
    double am = (a0 + a1) / 2.0;
    w.heading(am);

    // calculate motion command for the components
    lim.set(limits,w.x);
    rv.x = CalcMotion1D(lim, lookahead, d.x, v.x, tv.x, tx, cmd_schedule_x);

    lim.set(limits,w.y);
    rv.y = CalcMotion1D(lim, lookahead, d.y, v.y, tv.y, ty, cmd_schedule_y);

    time = max(tx,ty);
    err = fabs(tx-ty);

    if(debug){
      printf("  %d: a:[%0.4f %0.4f] w:<%f,%f> t:<%f,%f> e=%f\n",
             i,a0,a1,V2COMP(w),tx,ty,err);
    }
    if(err < 0.001) break;

    if(tx < ty){
      a0 = am;
    }else{
      a1 = am;
    }
    am = (a0 + a1) / 2.0;
  }
  // change velocity back into world space
  vector2f dir_p = dir.perp();
  rv = dir*rv.x + dir_p*rv.y;
  return(rv);
}

void CalcNextRotVelocity(
    const float target, const float current_vel, const float max_acc,
    const float max_speed, const float frame_period, float* velocity_next) {
  AccelLimits limits;
  limits.max_accel = max_acc;
  limits.max_deccel = max_acc;
  limits.max_vel = max_speed;
  double time = 0.0;
  *velocity_next =
      CalcMotion1D(limits, frame_period, target, current_vel, 0.0, time, NULL);
}

void CalcNextTransVelocity(
    const vector2f& target, const vector2f& current_vel, const float max_acc,
    const float max_speed, const float frame_period, vector2f* velocity_next) {
  AccelLimits limits;
  limits.max_accel = max_acc;
  limits.max_deccel = max_acc;
  limits.max_vel = max_speed;
  double time = 0.0;
  vector2f frame(1.0, 0.0);
  *velocity_next = CalcMotion2D(limits, frame_period,
                                vector2f(0,0), current_vel, target,
                                vector2f(0,0), time, frame,
                                NULL, NULL);
}