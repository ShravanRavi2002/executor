/*======================================================================
    Util.h : Numerical utility functions
  ----------------------------------------------------------------------
    Copyright (C) 1999-2002  James R. Bruce
    Modified by Joydeep Biswas, 2010
    School of Computer Science, Carnegie Mellon University
  ----------------------------------------------------------------------
    This software is distributed under the GNU General Public License,
    version 2.  If you do not have a copy of this licence, visit
    www.gnu.org, or write: Free Software Foundation, 59 Temple Place,
    Suite 330 Boston, MA 02111-1307 USA.  This program is distributed
    in the hope that it will be useful, but WITHOUT ANY WARRANTY,
    including MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  ======================================================================*/

#ifndef __UTIL_H__
#define __UTIL_H__

#include <stdint.h>
#include <cmath>
#include <string.h>
#include <algorithm>
#include <stdio.h>

// UnusedVar can suppress gcc warnings about unused variables.
// MustCheckReturn lets a function emit a warning if its return value
// isn't used
#if __GNUC__ >= 3
# define UnusedVar __attribute__((unused))
# define MustUseResult __attribute__((warn_unused_result))
#else
# define UnusedVar
# define MustUseResult
#endif

// This seems to be missing from some systems
#ifndef M_2PI
#define M_2PI 6.28318530717958647693
#endif

using std::sort;

#if 1

using std::min;
using std::max;
using std::swap;

#else
template <class num>
inline num max(num a,num b)
{
  return((a > b)? a : b);
}

template <class num>
inline num min(num a,num b)
{
  return((a < b)? a : b);
}

template <class data>
inline void swap(data &a,data &b)
{
  data t;
  t = a;
  a = b;
  b = t;
}
#endif

///Returns the min of a list of variables
template <class num>
inline num minVar(int n,num* list)
{
  num min, a;
  min = list[0];
  for(int i = 1; i <= n; i++) {
    a = list[i];
    if(a < min)
      min = a;
  }
  return min;
}

///Returns the max of a list of variables
template <class num>
inline num maxVar(int n,num* list)
{
  num max, a;
  max = list[0];
  for(int i = 1; i <= n; i++) {
    a = list[i];
    if(a > max)
      max = a;
  }
  return max;
}

///Return a (pseudo)random number between 0.0 and 1.0
#ifndef frand
inline double frand()
{
  return ( double(rand())/double(RAND_MAX) );
}

template <class num>
inline num frand(num minVal,num maxVal)
{
  return ( num(rand())/num(RAND_MAX)*(maxVal-minVal) +minVal );
}
#endif

///Return a (pseudo)random number drawn from the Normal Dsitribution
template <class num>
inline num randn(num stdDev = 1.0, num mean = 0.0)
{
  //Uses Box-Muller transform to turn a pair of uniform random numbers into a pair of gaussian random numbers
  num u1 = ((num)rand())/((num)RAND_MAX);
  num u2 = ((num)rand())/((num)RAND_MAX);
  num z1 = sqrt(-2.0*log(u1)) * sin(M_2PI*u2);
  //float z2 = sqrt(-2*log(u1)) * cos(2*M_PI*u2);
  num x1 = z1 * stdDev + mean;
  //float x2 = z2 * std_dev + mean;
  return x1;
}

///Return two (pseudo)random numbers drawn from the Normal Dsitribution
template <class num>
inline void randn2(num &x1, num &x2)
{
  //Uses Box-Muller transform to turn a pair of uniform random numbers into a pair of gaussian random numbers
  num u1 = ((num)rand())/((num)RAND_MAX);
  num u2 = ((num)rand())/((num)RAND_MAX);
  num r = sqrt(-2.0*log(u1));
  x1 = r * sin(M_2PI*u2);
  x2 = r * cos(M_2PI*u2);
}

template <class num1,class num2>
inline num1 bound(num1 x,num2 low,num2 high) MustUseResult;

template <class num1,class num2>
inline num1 bound(num1 x,num2 low,num2 high)
{
  if(x < low ) x = low;
  if(x > high) x = high;
  return(x);
}

template <class num1,class num2>
inline num1 abs_bound(num1 x,num2 range) MustUseResult;

template <class num1,class num2>
inline num1 abs_bound(num1 x,num2 range)
// bound absolute value x in [-range,range]
{
  if(x < -range) x = -range;
  if(x >  range) x =  range;
  return(x);
}

template <class num>
inline num abs_max(num a,num b)
{
  return((fabs(a) > fabs(b))? a : b);
}

template <class num>
inline num abs_min(num a,num b)
{
  return((fabs(a) < fabs(b))? a : b);
}

template <class num>
inline num max3(num a,num b,num c)
{
  if(a > b){
    return((a > c)? a : c);
  }else{
    return((b > c)? b : c);
  }
}

template <class num>
inline num min3(num a,num b,num c)
{
  if(a < b){
    return((a < c)? a : c);
  }else{
    return((b < c)? b : c);
  }
}

template <class num>
inline num max4(num a,num b,num c,num d)
{
  num x,y;
  x = max(a,b);
  y = max(c,d);
  return(max(x,y));
}

template <class num>
inline num min4(num a,num b,num c,num d)
{
  num x,y;
  x = min(a,b);
  y = min(c,d);
  return(min(x,y));
}

template <class num>
inline num max_abs(num a,num b)
{
  return((fabs(a) > fabs(b))? a : b);
}

template <class num>
inline num min_abs(num a,num b)
{
  return((fabs(a) < fabs(b))? a : b);
}

template <class data_t>
inline int max_idx(data_t *arr,int num)
{
  int mi = 0;
  for(int i=1; i<num; i++){
    if(arr[i] > arr[mi]) mi = i;
  }
  return(mi);
}

template <class data_t>
inline int min_idx(data_t *arr,int num)
{
  int mi = 0;
  for(int i=1; i<num; i++){
    if(arr[i] < arr[mi]) mi = i;
  }
  return(mi);
}

template <class num>
inline void sort(num &a,num &b,num &c)
{
  if(a > b) swap(a,b);
  if(b > c) swap(b,c);
  if(a > b) swap(a,b);
}

template <class num>
inline bool take_min(num &base,num val)
{
  if(val < base){
    base = val;
    return(true);
  }else{
    return(false);
  }
}

template <class num>
inline bool take_max(num &base,num val)
{
  if(val > base){
    base = val;
    return(true);
  }else{
    return(false);
  }
}

template <class real>
inline real sq(real x) MustUseResult;

template <class real>
real sq(real x)
{
  return(x * x);
}

template <class real>
inline real cube(real x) MustUseResult;

template <class real>
real cube(real x)
{
  return(x * x * x);
}

template <class num>
inline num sign_nz(num x)
{
  return((x >= 0)? 1 : -1);
}

template <class num>
inline num sign_eq(num a,num b)
{
  return((a*b)>=0);
}

template <class num>
inline num sign(num x)
{
  return((x >= 0)? (x > 0) : -1);
}

template <class bool_t>
inline void toggle(bool_t &b)
{
  b = !b;
}

template <class num>
inline num gcd(num x,num y)
// returns greatest common divisor of x,y
// NOTE: untested
{
  int w;
  while (y != 0) {
    w = x % y;
    x = y;
    y = w;
  }
  return x;
}

template <class num>
num lcm(num x,num y)
// returns least common multiple of x,y
// NOTE: untested
{
  num d = gcd(x,y);
  num r = (x / d) * (y / d);
  // num r = (x * y) / d; // faster but more likely to overflow
  return(r);
}

template <class num>
inline num mod(num x,num m)
// returns x mod m, where x e [0,m-1]
// note this is different from %, which can return negative numbers
{
  return(((x % m) + m) % m);
}

template <class real>
inline real fmodt(real x,real m)
// Does a real modulus the *right* way, using
// truncation instead of round to zero.
{
  return(x - floor(x / m)*m);
}

template <class real>
inline real ramp(real x,real x0,real x1)
// linear ramp from f(x0)=0 to f(x1)=1, with output bounded to [0,1]
// returns f(x)
{
  if(x < x0) return(0);
  if(x > x1) return(1);
  return((x - x0) / (x1 - x0));
}

template <class real>
inline real ramp(real x, real x0,real y0, real x1,real y1)
// linear ramp from f(x0)=y0 to f(x1)=y1, with output bounded to [y0,y1]
// returns f(x)
{
  if(x < x0) return(y0);
  if(x > x1) return(y1);
  return(y0 + (y1 - y0) * (x - x0) / (x1 - x0));
}

template <class num,class num2>
inline num bool_sat_count(num cnt,num2 min,num2 max,bool val)
// Saturating counter of a boolean value, clipped to the range
// [min,max].  True values increment the counter (up to max), while
// false values decrement the counter (down to min).
{
  if(val){
    return((cnt < max)? cnt+1 : max);
  }else{
    return((cnt > min)? cnt-1 : min);
  }
}


//==== Bitwise Utilities =============================================//

template <class num>
inline bool one_bit_set(num n)
// returns true if num has only one bit set, i.e. n=2^k for some integer k
{
  return(n!=0 && ((-n) & n) == n);
}

template <class num>
inline num num_bits_set(num x)
// returns the number of bits set in x
{
  num numBitsSet = 0;
  for(num i = 0; i<sizeof(num)*8; i++){
    if(x&(1<<i))
      numBitsSet++;
  }
  return numBitsSet;
}

template <class num1,class num2>
inline bool all_bits_set(num1 x,num2 m)
// returns true if all set bits in <m> are present in <x>
{
  return((x & m) == m);
}

template <class num1,class num2>
inline bool any_bits_set(num1 x,num2 m)
// returns true if any set bits in <m> are present in <x>
{
  return((x & m) != 0);
}

//==== Angle Utilities ===============================================//

/// Returns angle within [-PI,PI]
template <class real>
inline real angle_mod(real angle) MustUseResult;

template <class real>
real angle_mod(real angle)
{
  angle -= M_2PI * rint(angle / M_2PI);

  return(angle);
}

/// Returns the secondary angle x such that PI <= fabs(x) <= 2PI
template <class real>
inline real angle_long(real angle)
{
  if (angle < 0.0) {
    return M_2PI + angle;
  } else {
    return -M_2PI + angle;
  }
}

/// Returns angle within [0,2*PI]
template <class real>
inline real angle_pos(real angle)
{
  return(fmod(angle+M_2PI,M_2PI));
}

/// Returns difference of two angles (a-b), within [-PI,PI]
template <class real>
inline real angle_diff(real a,real b)
{
  real d;

  d = a - b;
  d -= M_2PI * rint(d / M_2PI);

  return(d);
}

/// Returns absolute angular distance between two angles
template <class real>
inline real angle_dist(real a,real b)
{
  return(fabs(angle_mod(a-b)));
}

/// return the normalized angle halfway between these two
/// assumes the arguments are already normalized to (-M_PI, M_PI].
template <class real>
inline real avg_angle(real left,real right)
{
  real result;

  if(left < right) left += 2*M_PI;
  result = (left + right)/2;
  if(result > M_PI) result -= 2*M_PI;

  return(result);
}

template <class real>
inline real abs_bound_angle(real bound_angle,real tolerance,real a) MustUseResult;

/// bound angle <a> to interval [bound_angle-tolerance, bound_angle+tolerance]
/// does the appropriate normalization of angles as long as <tolerance>
/// is below M_PI.
template <class real>
real abs_bound_angle(real bound_angle,real tolerance,real a)
{
  real x = angle_mod(a - bound_angle);
  x = abs_bound(x,tolerance);
  return(angle_mod(x + bound_angle));
}

//==== Array Functions ===============================================//

template <class data>
int find_item(const data *arr,int num,data key)
{
  int i = 0;
  while(i<num && !(arr[i]==key)) i++;
  return(i);
}

template <class data,class num>
data *alloc_array(data *arr,num &size,num new_size) MustUseResult;

template <class data,class num>
data *alloc_array(data *arr,num &size,num new_size)
{
  if((arr!=NULL && new_size==size) ||
     (arr==NULL && new_size==0)) return(arr);

  delete[](arr);
  arr = new data[new_size];
  size = (arr != NULL)? new_size : 0;
  return(arr);
}

template <class data,class num>
data *resize_array(data *arr,num &size,num new_size,num cur_used)
{
  if((arr!=NULL && new_size==size) ||
     (arr==NULL && new_size==0)) return(arr);

  data *narr = new data[new_size];
  if(narr){
    // copy existing data
    for(num i=0; i<cur_used; i++) narr[i] = arr[i];
    delete[](arr);
    size = new_size;
    return(narr);
  }else{
    // could not resize
    return(arr);
  }
}

template <class data,class num>
void free_array(data *&arr,num &size)
{
  delete[](arr);
  arr = NULL;
  size = 0;
}

template <class data,class num>
data *alloc_array2(data *arr,num &w,num &h,
                   int new_w,int new_h) MustUseResult;

template <class data,class num>
data *alloc_array2(data *arr,num &w,num &h,
                   int new_w,int new_h)
{
  int size = w * h;
  int new_size = new_w * new_h;

  if((arr!=NULL && new_size==size) ||
     (arr==NULL && new_size==0)) return(arr);

  delete[](arr);
  arr = new data[new_size];
  if(arr){
    w = new_w;
    h = new_h;
  }else{
    w = h = 0;
  }

  return(arr);
}

template <class data>
void set_range(data *arr,int start,int num,const data &val)
{
  num += start;
  for(int i=start; i<num; i++) arr[i] = val;
}

//==== Template-based Memory Operations ==============================//

template <class data>
inline int mcopy(data *dest,data *src,int num)
{
  int i;

  for(i=0; i<num; i++) dest[i] = src[i];

  return(num);
}

template <class data>
inline data mset(data *dest,data val,int num)
{
  int i;

  for(i=0; i<num; i++) dest[i] = val;

  return(val);
}

template <class data>
inline void mzero(data &d)
{
  memset(&d,0,sizeof(d));
}

template <class data>
inline void mzero(data *d,int n)
{
  memset(d,0,sizeof(data)*n);
}

// #if __GNUC__ >= 3
// # define likely(x) __builtin_expect(!!(x),1)
// # define unlikely(x) __builtin_expect(!!(x),0)
// #else
// # define likely(x) (x)
// # define unlikely(x) (x)
// #endif

#endif