#pragma once
#include<vector>
#include<float.h>
#include<math.h>
#include<assert.h>
using namespace std;
namespace Physics
{
	/* 头文件 */
	///////////////////////////////////////////////////////////////////////////////
	typedef float real;  //定义实数
    #define real_sqrt sqrt
	#define real_pow pow
	#define real_abs abs
	#define real_exp exp
	#define real_sin sin
	#define real_cos cos
	#define real_fmod fmod
	#define InFiniteMass REAL_MAX
	#define REAL_MAX FLT_MAX
	#define REAL_PI 3.14159265358979
	#define CANNOTSLEEPTIMES 20
#define SPHERE 1
#define BOX 2
#define CIRCLE 4
#define PLANE 8

#define SPHERE_SPHERE 2
#define SPHERE_BOX 3
#define SPHERE_CIRCLE 5
#define SPHERE_PLANE 9
#define BOX_BOX 4
#define BOX_CIRCLE 6
#define BOX_PLANE 10
#define CIRCLE_CIRCLE 8
#define CIRCLE_PLANE 12
	extern real DLL_PHYSICS sleepEpsilon;
	void DLL_PHYSICS SetSleepEpsilon(real value);
	real DLL_PHYSICS GetSleepEpsilon();
}