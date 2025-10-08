#ifndef _FUNC_LIB_H_
#define _FUNC_LIB_H_

#include "components.h"
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include "pid.h"
#include "fsfc.h"
#include "communicate.h"
#include "protocol.h"
#include "comm.h"
#include "fifo.h"
#include "led.h"
#include "main_control.h"

//define of constant to enhance calculation cost
#define SQUARE_ROOT_OF_2 1.41421356f
#define SQUARE_ROOT_OF_2_DIVIDED_BY_2  0.70710678f

//homemade math functions
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
#define DEG2R(x) ((x)*PI /180.0f)
#define R2DEG(x) ((x)*180.0f /PI)

#endif
