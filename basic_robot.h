#pragma once
#include "common.h"
#include "vector2d.h"


typedef std::vector<Vector2d> SList;

class Basic_robot
{
private:
	//parameters
	const double SPEED = 3.0;
	const double raPos[3] = { 0.0,2.09,4.19 };
	int timeStep;
	double left_speed, rightspeed;

public:
	//the webots robot
	Robot *robot;

	//device
	Compass *compass;
	Motor *leftMotor, *rightMotor;
	Emitter *emitter;
	Receiver *receiver;
	Radar *ra1, *ra2, *ra3;
	DistanceSensor *us0, *us1;
	GPS *gps;

	//parameters and define

	//sensing range
	const double Rs = 1.5;
	//comunicate range
	const double Rc = 2.0;
	//min(Rc,Rs)
	const double Ra = 1.5;

	//distance error(0.01 m)
	const double DistError = 0.01;
	
	//angle error(0.01 rad)
	const double AngleError = 0.01;

	typedef int ID;
	typedef vector<ID> RLIst;
	typedef set<ID> RSet;

	typedef struct { ID r; Vector2d p; } MSG;

	//init
	Basic_robot();

	//name
	string Name();

	//position
	Vector2d Position();

	//direction
	Vector2d Direction();

	//forward
	void Forward(double k);

	//turn
	void Turn(double k);

	//one step
	int Step();

	//step
	void Step(double seconds);

	//stop
	void Stop();

	//surround robots (relativel position)
	SList SurroundRobots_R();

	//surround robots (absolute position)
	SList SurroundRobots_A();
};








