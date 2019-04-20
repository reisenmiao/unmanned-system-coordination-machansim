#include "basic_robot.h"



//init
Basic_robot::Basic_robot()
{
	robot = new Robot();

	timeStep = (int)robot->getBasicTimeStep();


	leftMotor = robot->getMotor("left wheel motor");
	rightMotor = robot->getMotor("right wheel motor");
	leftMotor->setPosition(INFINITY);
	rightMotor->setPosition(INFINITY);
	leftMotor->setVelocity(0.0);
	rightMotor->setVelocity(0.0);


	compass = robot->getCompass("compass");
	compass->enable(timeStep);

	emitter = robot->getEmitter("emitter");

	receiver = robot->getReceiver("receiver");
	receiver->enable(timeStep);

	ra1 = robot->getRadar("radar1");
	ra2 = robot->getRadar("radar2");
	ra3 = robot->getRadar("radar3");

	ra1->enable(timeStep);
	ra2->enable(timeStep);
	ra3->enable(timeStep);

	us0 = robot->getDistanceSensor("us0");
	us1 = robot->getDistanceSensor("us1");
	us0->enable(timeStep);
	us1->enable(timeStep);

	gps = robot->getGPS("gps");
	gps->enable(timeStep);

}

//name
string Basic_robot::Name()
{
	return robot->getName();
}

//position
Vector2d Basic_robot::Position()
{
	const double *position = gps->getValues();
	return Vector2d(position[2], position[0]);
}

//direction
Vector2d Basic_robot::Direction()
{
	const double *north = compass->getValues();
	return Vector2d(-north[0], -north[2]).normalize();
}

//forward
void Basic_robot::Forward(double k)
{
	leftMotor->setVelocity(SPEED*k);
	rightMotor->setVelocity(SPEED*k);
}

//turn
void Basic_robot::Turn(double k)
{
	leftMotor->setVelocity(SPEED *k);
	rightMotor->setVelocity(-SPEED*k);
}

//one step
int Basic_robot::Step()
{
	return robot->step(timeStep);
}

//step
void Basic_robot::Step(double seconds)
{
	const double ms = seconds * 1000;
	int elapsed_time = 0;
	while (elapsed_time < ms)
	{
		robot->step(timeStep);
		elapsed_time += timeStep;
	}
}

//stop
void Basic_robot::Stop()
{
	leftMotor->setVelocity(0.0);
	rightMotor->setVelocity(0.0);
}

//surround robots (relativel position)
SList Basic_robot::SurroundRobots_R()
{
	SList Sr;

	//deal with the radar1
	int targetsNumber = ra1->getNumberOfTargets();
	const RadarTarget* target = ra1->getTargets();

	for (int i = 0; i < targetsNumber; i++)
	{
		double distance = target[i].distance;
		Vector2d azimuth = Direction().rotate(-target[i].azimuth + raPos[0]);

		Vector2d pos = azimuth*distance;

		bool flag = true;

		for (unsigned j = 0; j < Sr.size(); j++)

			if ((pos - Sr[j]).norm() < DistError)	
				flag = false;

		if (flag)
		{
			Sr.push_back(pos);
		}
	}

	//deal with the radar2
	targetsNumber = ra2->getNumberOfTargets();
	target = ra2->getTargets();

	for (int i = 0; i < targetsNumber; i++)
	{
		double distance = target[i].distance;
		Vector2d azimuth = Direction().rotate(-target[i].azimuth + raPos[1]);
	
		Vector2d pos = azimuth * distance;

		bool flag = true;

		for (unsigned j = 0; j < Sr.size(); j++)

			if ((pos - Sr[j]).norm() < DistError)
				flag = false;

		if (flag)
		{
			Sr.push_back(pos);
		}
	}

	//deal with radar3
	targetsNumber = ra3->getNumberOfTargets();
	target = ra3->getTargets();

	for (int i = 0; i < targetsNumber; i++)
	{
		double distance = target[i].distance;
		Vector2d azimuth = Direction().rotate(-target[i].azimuth + raPos[2]);

		Vector2d pos = azimuth * distance;

		bool flag = true;

		for (unsigned j = 0; j < Sr.size(); j++)

			if ((pos - Sr[j]).norm() < DistError)
				flag = false;

		if (flag)
		{
			Sr.push_back(pos);
		}
	}

	//print to debug
	/*
	std::cout << Name() << ": Nearby Robots { ";
	for (SList::iterator iter = Sr.begin(); iter != Sr.end(); iter++)
	{
		cout << *iter << ", ";
	}
	std::cout << "}" << std::endl;
	*/

	return Sr;

}

//surround robots (absolute position)
SList Basic_robot::SurroundRobots_A()
{
	SList Sa;
	SList Sr = SurroundRobots_R();

	for (SList::iterator iter = Sr.begin(); iter != Sr.end(); iter++)
	{
		Vector2d pos = Position();
		Sa.push_back(*iter + pos);
	}

	//print to debug
	cout <<Name()<<": nearby robots  { ";
	for (SList::iterator iter = Sa.begin(); iter != Sa.end(); iter++)
	{
		cout << *iter;
	}
	cout << "}" << endl;
	

	return Sa;
}