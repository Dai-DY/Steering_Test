#ifndef INCLUDE_H
#define INCLUDE_H
#include "performance_test.h"
#include "pid.h"
#include "components.h"
#include "communicate.h"
#include "protocol.h"
#include "comm.h"
#include "os.h"
#include "fifo.h"
#include <math.h>

#define M3508_MAX_WHEEL_RPM 8500 
#define PI          3.1416f
#define RADIAN_COEF 57.3f

#define VAL_LIMIT(val, min, max) \
do {\
if((val) <= (min))\
{\
  (val) = (min);\
}\
else if((val) >= (max))\
{\
  (val) = (max);\
}\
} while(0)\

#define cushionk(a, b, k) ((a) + (k) * ((b) - (a)))

#endif
