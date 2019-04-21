#include "contr_robot.h"


void Contr_robot::BasicMovement()
{
	double d0 = us0->getValue();
	double d1 = us1->getValue();

	if (d0 < 100 || d1 < 100)
	{
		Turn(1.0);
	}

	else
	{
		Forward(1.0);
	}

}

//turn to a target direction using PID method
void Contr_robot::PIDTurn(Vector2d target)
{
	//pre deal
	target = target.normalize();

	//baisc parameters
	double Kp = 0.005, Kd = 0.005, Ki = 0.0005;

	double error =  Dist(Direction(), target);   //cross
	double old_error = error;
	double E = error;
	double error_dot = 0;
	double k = 0;

	if (Direction() % target >= 0)
		Turn(Kp*error);
	else
		Turn(-Kp*error);


	while ( Step() != -1 && error > AngleError)
		{
			//refresh error and move
			error = Dist(Direction(),target);
	
			error_dot = error - old_error;
			E = E + error;
	
			k = Kp*error + Kd*error_dot + Ki*E;

			if (Direction() % target >= 0)
				Turn(k);
			else
				Turn(-k);

			old_error = error;
		}

}

//move to a target position using PID method
void Contr_robot::PIDForward(Vector2d target)
{
	//turn to right direction in the first
	Vector2d position = Position();
	Vector2d right_direction = (target - position).normalize();
	PIDTurn(right_direction);

	//then move to the target position
	double Kp = 0.1, Kd = 0.1, Ki = 0.01;

	double error = Dist(target, Position());
	double old_error = error;
	double E = error;
	double error_dot = 0;
	double k = 0;
	
	if (error > DistError)
		Forward(Kp*error);

	while (Step() != -1 && error > DistError)
	{
		//adjust the direction
		Vector2d position = Position();
		Vector2d right_direction = (target - position).normalize();
		PIDTurn(right_direction);


		//refresh error and move
		error = Dist(target, Position());

		error_dot = error - old_error;
		E = E + error;

		k = Kp*error + Kd * error_dot + Ki * E;
		Forward(k);

		old_error = error;
	}

}