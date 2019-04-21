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
	
	
	
	while (robot.Step()!=-1)
	{
		robot.CircleFormation();
		robot.UniformTransformation();
	}

	
	return 0;
}







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