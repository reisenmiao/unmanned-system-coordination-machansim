// File:          form_circle.cpp
// Date:
// Description:
// Author:
// Modifications:

#include "common.h"
#include "Intel_robot.h"


int main(int argc, char **argv)
{
	Intel_robot robot;
	
	
	robot.Initialization();

	robot.Step();
	robot.NetConstruction();

	robot.ConvexHullConstruction();

	robot.Step();
	robot.Determination();

	while (robot.Step()!=-1)
	{
		robot.CircleFormation();
		robot.UniformTransformation();
	}

	return 0;
}




