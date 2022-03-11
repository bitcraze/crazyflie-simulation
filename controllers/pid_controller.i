/* File: pid_controller.i */
%module pid_controller

%{
#define SWIG_FILE_WITH_INIT
#include "pid_controller.h"
%}

float constrain(float value, const float minVal, const float maxVal);
