#pragma once
#include "basic_robot.h"


class Contr_robot :public Basic_robot
{
public:
	
	//aovid obstacles movement
	void BasicMovement();

	//turn to a target direction using PID method
	void PIDTurn(Vector2d target);

	//move to a target position using PID method
	void PIDForward(Vector2d target);

};