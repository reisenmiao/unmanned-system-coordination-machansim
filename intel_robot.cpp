#include "intel_robot.h"

//init
Intel_robot::Intel_robot(void)
{
	AMOUNT = 4;
	O = Vector2d(1.0, 1.25);
	R = 0.6592;
	d = 2 * PI / AMOUNT;

}

//NetWork construction for each robot ri
RSet Intel_robot::NetConstruction(void)
{
	//Neighborhood list 
	RSet N;
	Vector2d pos = Position();
	
	//abosulute position list
	SList Sa = SurroundRobots_A();

	/*broadcast Msg of itself*/
	MSG m;
	m.r = ID();
	m.p = pos;
	void *m_pt = &m;
	emitter->send(m_pt, sizeof(MSG));

	while (receiver->getQueueLength() >0) 
	{
		//get the receive data
		const MSG *Msgj = (MSG*)receiver->getData();
		
		//if receive the msg from itself,break this rountie
	
		//print Msg
		//std::cout << "Reveive  : ( ID: " << Msgj->r << ", x:" << Msgj->p.x << ", z:" << Msgj->p.z << " )" << std::endl;


		//draw the dircle
		double diameter;
		Vector2d center;
		diameter = Dist(pos, Msgj->p);

		if (diameter < Ra) 
		{
			center = MidPoint(pos, Msgj->p);

			//search if any robot in circle
			bool flag = true;
			for (SList::iterator iter = Sa.begin();iter != Sa.end(); ++iter)
				if (Dist(*iter, center) < diameter/2 - DistError)
					flag = false;
				
			if (flag)
				N.insert(Msgj->r);
		}

		//pop the msg please!
		receiver->nextPacket();
	}
	
	//print to debug
	
	std::cout << "Neighborhood : { ";
	for (RSet::iterator it = N.begin(); it != N.end(); it++)
	{
		std::cout << *it << ", ";
	}
	std::cout << "}" << std::endl;
	
	return N;

}


//Distribute convex hull Constrction for each robot ri
void ConvexHullConstruction(void)
{

}

//Distribute cardinality estimation for each robot ri
void CardinalityEstimation(void)
{

}

//Determination of the center and radius of the cirle to be formed for each robot ri
void Determination(void)
{

}

//Circle Formation for each robot ri
void Intel_robot::CircleFormation(void)
{
	Vector2d pos = Position();
	Vector2d D = Intersection(pos, O, R);
	//print to debug
	//cout << Name() << ": " << D << endl;
	PIDForward(D);
}

//Uniform transformation for each robot ri
void Intel_robot::UniformTransformation(void)
{
	Vector2d pos = Position();
	SList Sa = SurroundRobots_A();	
	
	//find the most nearby robot clockwise and cal the d+
	double minRad = -PI;
	
	if (Sa.size() < AMOUNT - 1)
	{
		cout << "Waiting: some robot don't reach the circle." << endl;
		return;
	}

	for (SList::iterator it = Sa.begin(); it != Sa.end(); it++)
	{
		if (abs(Dist(*it, O) - R) > DistError)
		{
			cout << "Waiting: some robot don't reach the circle." << endl;
			return;
		}

		double rad = Rad(pos, *it, O);

		if (rad < 0 && rad > minRad)
			minRad = rad;

	}

	//print to debug
	cout << Name() <<":Nearby Robots  : { ";
	for (SList::iterator iter = Sa.begin(); iter != Sa.end(); iter++)
	{
		cout << "( " << Rad(pos,*iter,O) << " ), ";
	}
	cout << "} minRad:" << minRad << endl;


	//if d+ > d
	if (abs(minRad) > d && abs(minRad+d)>AngleError)
	{
		Vector2d target_circle = (pos - O).rotate(d + minRad);
		Vector2d target = target_circle + O;
		PIDForward(target);
	}
	//std::vector<double> SListRad;

	//for (SList::iterator it = Sa.begin(); it != Sa.end(); it++)
	//{
	//		if (abs(calDist(O, *it) - R) > 0.05)
			//{
	//			//std::cout << "Waiting: some robot don't reach the circle." << std::endl;
	//			return;
	//		}
	//		double temp_rad = getVectorRad(O, *it);
	//		SListRad.push_back(temp_rad);
	//	}
	//	
	//print to debug
		//std::cout << "Nearby Robots  : { ";
	//for (std::vector<double>::iterator iter = SListRad.begin(); iter != SListRad.end(); iter++)
	//	{
	//		std::cout << "( " << *iter << " ), ";
	//}
	//std::cout << "}" << std::endl;
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
}