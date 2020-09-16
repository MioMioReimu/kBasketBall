#define DLL_PHYSICS_EXIST
#include"Physics.h"
namespace Physics{
	real sleepEpsilon=((real)1);
	void SetSleepEpsilon(real value)
	{
		sleepEpsilon=value;
	}
	real GetSleepEpsilon()
	{
		return sleepEpsilon;
	}
}