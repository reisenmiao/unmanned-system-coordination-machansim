// File:          form_circle.cpp
// Date:
// Description:
// Author:
// Modifications:

#include "common.h"
#include "basic_robot.h"





int main(int argc, char **argv)
{
	Basic_robot robot;

	while (robot.Step() != -1)
	{	
		robot.SurroundRobots_A();
	}


	return 0;
}






//


//
//
//
//
//

//

//
//void basicMovement(void)
//{
//	double d0 = us0->getValue();
//	double d1 = us1->getValue();
//
//	if (d0 < 100 || d1 < 100)
//	{
//		leftMotor->setVelocity(-SPEED);
//		rightMotor->setVelocity(SPEED);
//	}
//	else
//	{
//		leftMotor->setVelocity(SPEED);
//		rightMotor->setVelocity(SPEED);
//	}
//}
//
////move to the target point
//void PIDmovement(Pos amibition)
//{
//
//	Pos p = getPosition();
//	double ed = calDist(amibition, p);
//	double ed_old = ed;
//	double Ed = ed;
//	double ed_dot = 0;
//	if (ed > 0.01)
//		forward(SPEED*0.01*ed);
//
//
//	//use the PID control to reach
//	double Kp = 1.0, Kd = 1.0, Ki = 0.1;
//	while ((robot->step(timeStep) != -1) && ed > 0.05)
//	{
//		//calculate target direction
//		double amibitionAngle = atan((amibition.x - p.x) / (amibition.z - p.z));
//		if ((amibition.z - p.z) > 0)
//			amibitionAngle += 3.14;
//		amibitionAngle = modifyAzimuth(amibitionAngle);
//		//std::cout << robot->getName() << ":" << "ambition angle" << amibitionAngle << std::endl;
//
//		//turn to the ambition angle
//		double error = amibitionAngle - p.theta;
//		double error_old = error;
//		double E = error;
//		double e_dot = 0;
//		double turn_speed = 0;
//		turn(SPEED*(-error));
//
//		//use PID_control to turn
//		while ((robot->step(timeStep) != -1) && abs(error) > 0.5
//			)
//		{
//			//refresh position message and error
//			p = getPosition();
//			error = amibitionAngle - p.theta;
//
//			e_dot = error - error_old;
//			E = E + error;
//
//			turn_speed = 0.005*(Kp*error + Kd*e_dot + Ki*E);
//			if (abs(turn_speed) > 10)
//			{
//				stop(1.0);
//				forward(SPEED);
//				step(1.0);
//			}
//
//			turn(-turn_speed);
//			error_old = error;
//		}
//
//		//std::cout << robot->getName() << ": finish the turn" << std::endl;
//		//reach the target
//		p = getPosition();
//		ed = calDist(amibition, p);
//
//		ed_dot = ed - ed_old;
//		Ed = Ed + ed;
//		forward(SPEED*0.2*(Kp*ed + Kd*ed_dot + Ki *Ed));
//		ed_old = ed;
//	}
//
//	//std::cout << robot->getName() << ": reach the ambition" << std::endl;
//
//
//}
//
////move along the circle to the target
////just do some adjustment to PIDmovement 
//void moveAlongCirlce(Pos amibition,Pos O)
//{
//	Pos p = getPosition();
//	double ed = calDist(amibition, p);
//	double ed_old = ed;
//	double Ed = ed;
//	double ed_dot = 0;
//	if (ed > 0.01)
//		forward(SPEED*0.01*ed);
//
//
//	//use the PID control to reach
//	double Kp = 1.0, Kd = 1.0, Ki = 0.1;
//	while ((robot->step(timeStep) != -1) && ed > 0.05)
//	{
//		
//		//the rad of ray OPi
//		double OPRad = getVectorRad(O, p);
//		double OPRad_re = modifyAzimuth(OPRad + PI);
//
//		//the angle of ray OPtarget
//		double OPtRad = getVectorRad(O, amibition);
//
//		double amibitionAngle;
//		if (OPRad_re <= OPtRad && OPtRad <= OPRad)
//			amibitionAngle = OPRad - PI / 2;
//		else
//			amibitionAngle = OPRad + PI / 2;
//		std::cout << "ambition angle:" << amibitionAngle << std::endl;
//
//		//turn to the ambition angle
//		double error = amibitionAngle - p.theta;
//		double error_old = error;
//		double E = error;
//		double e_dot = 0;
//		double turn_speed = 0;
//		turn(SPEED*(-error));
//
//		//use PID_control to turn
//		while ((robot->step(timeStep) != -1) && abs(error) > 0.5)
//		{
//			//refresh position message and error
//			p = getPosition();
//			error = amibitionAngle - p.theta;
//
//			e_dot = error - error_old;
//			E = E + error;
//
//			turn_speed = 0.005*(Kp*error + Kd*e_dot + Ki*E);
//			
//			//random move to avoid some special pos
//			if (abs(turn_speed) > 10)
//			{
//				stop(10.0);
//				forward(SPEED);
//				step(10.0);
//			}
//
//			turn(-turn_speed);
//			error_old = error;
//		}
//
//		//std::cout << robot->getName() << ": finish the turn" << std::endl;
//		//reach the target
//		p = getPosition();
//		ed = calDist(amibition, p);
//
//		ed_dot = ed - ed_old;
//		Ed = Ed + ed;
//		forward(SPEED*0.2*(Kp*ed + Kd*ed_dot + Ki *Ed));
//		ed_old = ed;
//	}
//
//	std::cout << robot->getName() << ": reach the ambition" << std::endl;
//}
//
////NetWork construction for each robot ri
//RSet NetConstruct(ID id,SList Sr)
//{
//	//Neighborhood list 
//	RSet N;
//	
//	//abosulute position list
//	SList Sa;
//	Pos p = getPosition();
//
//	for (SList::iterator iter = Sr.begin(); iter != Sr.end(); iter++)
//	{
//		Pos temp;
//		temp.x = p.x + iter->x;
//		temp.z = p.z + iter->z;
//		Sa.push_back(temp);
//	}
//
//	/*broadcast Msg of itself*/
//	MSG1 m;
//	m.r = id;
//	m.p = p;
//	void *m_pt = &m;
//	emitter->send(m_pt, sizeof(MSG1));
//
//	while (receiver->getQueueLength() >0) 
//	{
//		//get the receive data
//		const MSG1 *Msgj = (MSG1*)receiver->getData();
//		
//		//if receive the msg from itself,break this rountie
//	
//		//print Msg
//		//std::cout << "Reveive  : ( ID: " << Msgj->r << ", x:" << Msgj->p.x << ", z:" << Msgj->p.z << " )" << std::endl;
//
//
//		//draw the dircle
//		double diameter;
//		Pos center;
//		diameter = calDist(p, Msgj->p);
//
//		if (diameter < Ra) 
//		{
//			center = Midpoint(p, Msgj->p);
//
//			//search if any robot in circle
//			bool flag = true;
//			for (SList::iterator iter = Sa.begin();iter != Sa.end(); ++iter)
//				if (calDist(p, Msgj->p) < diameter)
//					flag = false;
//				
//			if (flag)
//				N.insert(Msgj->r);
//		}
//
//		//pop the msg please!
//		receiver->nextPacket();
//	}
//	
//	//print to debug
//	/*
//	std::cout << "Neighborhood : { ";
//	for (RSet::iterator it = N.begin(); it != N.end(); it++)
//	{
//		std::cout << *it << ", ";
//	}
//	std::cout << "}" << std::endl;
//	*/
//	return N;
//
//}
//
//
//
////Circle Formation for each robot ri
////uncompeleted!!!!!!!!!!!!!!!!!!
//void CircleFormation(Pos O, double Rcir, ID id)
//{
//	Pos p = getPosition();
//	Pos D = calIntersection(p, O, Rcir);
//    //print to debug
//	//std::cout << "D  : ( x:" << D.x << ", z:" << D.z << " )" << std::endl;
//	PIDmovement(D);
//}
//
////Uniform transformation for each robot ri
//void UniformTransformation(double d,Pos O,double R)
//{
//	Pos p = getPosition();
//	SList Sa = getSa(p.theta);
//	
//	//find the most nearby robot clockwise and cal the d+
//	std::vector<double> SListRad;
//	if (Sa.size() < 3)
//	{
//		std::cout << "Waiting: some robot don't reach the circle." << std::endl;
//		return;
//	}
//	for (SList::iterator it = Sa.begin(); it != Sa.end(); it++)
//	{
//		if (abs(calDist(O, *it) - R) > 0.05)
//		{
//			//std::cout << "Waiting: some robot don't reach the circle." << std::endl;
//			return;
//		}
//		double temp_rad = getVectorRad(O, *it);
//		SListRad.push_back(temp_rad);
//	}
//	
//	//print to debug
//	std::cout << "Nearby Robots  : { ";
//	for (std::vector<double>::iterator iter = SListRad.begin(); iter != SListRad.end(); iter++)
//	{
//		std::cout << "( " << *iter << " ), ";
//	}
//	std::cout << "}" << std::endl;
//	
//
//	double minDist = 3*PI;
//	double self_rad = getVectorRad(O, p);
//	std::cout << robot->getName() << ": myPos: " << self_rad << std::endl;
//	for (std::vector<double>::iterator it = SListRad.begin(); it != SListRad.end(); it++)
//	{
//		double dist = modifyAzimuth(self_rad - *it);
//		while (dist < 0)
//			dist += 2 * PI;
//		while (dist > 2 * PI)
//			dist -= 2 * PI;
//		
//		if (dist < minDist)
//			minDist = dist;
//	}
//	
//	
//	//print to debug
//	std::cout << "Most neraby robot:" << modifyAzimuth(self_rad-minDist)  << std::endl;
//	//if d+ > d
//	if (minDist > d)
//	{
//		double targetRad = modifyAzimuth(self_rad + d - minDist) ;
//		Pos target;
//		target.x = R*sin(targetRad) + O.x;
//		target.z = R*cos(targetRad) + O.z;
//		std::cout <<robot->getName() <<"amibition:" << targetRad << std::endl;
//		PIDmovement(target);
//	}
//	
//
//
//}
//*/