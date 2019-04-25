#include "intel_robot.h"

//init
void Intel_robot::Initialization(void)
{
	N.clear();

	conv.Top = -1;
	conv_init = false;

	for (int k = 0; k < K; k++)
		for (int i = 0; i < L; i++)
			x[k][i] = 0;

	CardinalityEstimation();

	AMOUNT = 6;
	O = Vector2d(0, 0);
	R = 0.0;
	d = 2 * PI / AMOUNT;

}

//NetWork construction for each robot ri
void Intel_robot::NetConstruction(void)
{
	Vector2d pos = Position();
	
	//abosulute position list
	SList Sa = SurroundRobots_A();

	/*broadcast Msg of itself*/
	MSG m;
	m.r = ID();
	m.p = pos;
	void *m_pt = &m;
	emitter->send(m_pt, sizeof(MSG));
	
	//waiting 0.5s to receive message
	Step(0.5);

	while (receiver->getQueueLength() >0) 
	{
		//get the receive data
		const MSG *Msgj = (MSG*)receiver->getData();
		
		//draw the dircle
		double diameter;
		Vector2d center;
		diameter = Dist(pos, Msgj->p);

		if (diameter < Ra) 
		{
			center = MidPoint(pos, Msgj->p);

			//search if any robot in circle
			bool flag = true;
			for (SList::iterator iter = Sa.begin(); iter != Sa.end(); ++iter)
			{
				if (Dist(*iter, Msgj->p) < DistError)
					continue;

				if (Dist(*iter, center) < diameter / 2 + DistError)
					flag = false;
			}
				
			if (flag)
				N.insert(Msgj->r);
		}

		//pop the msg please!
		receiver->nextPacket();
	}
	
	//print ro debug
	
	cout <<Name() << ": Neighborhood { ";
	for (RSet::iterator it = N.begin(); it != N.end(); it++)
	{
		cout << *it << ", ";
	}
	cout << "}" << endl;
	
	
}


//Distribute convex hull Constrction for each robot ri
void Intel_robot::ConvexHullConstruction(void)
{
	set<int> Fin;
	vector<MSG2> msglist[16];

	Conv temp;
	while (Step() != -1)
	{
		if (!conv_init)
		{
			ConvexPoint point;
			point.id = ID();
			point.p = Position();

			conv.points[++conv.Top] = point;

			temp = conv;

			//send msg
			MSG2 m;
			m.r = ID();
			m.conv = conv;
			for (int k = 0; k < K; k++)
				for (int i = 0; i < L; i++)
					m.x[k][i] = x[k][i];

			void *m_pt = &m;
			emitter->send(m_pt, sizeof(MSG2));

			//finished the initialization
			conv_init = true;

		}
		else if (receiver->getQueueLength() > 0)
		{
			//receive the message
			const MSG2 *Msgj = (MSG2*)receiver->getData();
			
			//if receive the message from robot which is not in N,ignore it 
			if (N.find(Msgj->r) == N.end())
			{
				receiver->nextPacket();
				continue;
			}

			if (Fin.find(Msgj->r) != Fin.end())
			{
				msglist[Msgj->r].push_back(*Msgj);
			}
			else
			{
				temp = ConvexHullMerge(temp, Msgj->conv);
				for (int k = 0; k < K; k++)
					for (int i = 0; i < L; i++)
						x[k][i] = x[k][i] || Msgj->x[k][i];

				Fin.insert(Msgj->r);

				if (Fin.size() == N.size())
				{
					if (temp == conv)
					{
						cout << Name() << ": {";
						for (int i = 0; i <= conv.Top; i++)
							cout << conv.points[i].p;
						cout << "}, (";
						for (int k = 0; k < K; k++)
						{
							for (int i = 0; i < L; i++)
								cout << x[k][i];
							cout << ", ";
						}
						cout << " )" << endl;

						//send msg
						MSG2 m;
						m.r = ID();
						m.conv = conv;
						for (int k = 0; k < K; k++)
							for (int i = 0; i < L; i++)
								m.x[k][i] = x[k][i];

						void *m_pt = &m;
						emitter->send(m_pt, sizeof(MSG2));

						return;
					}

					conv = temp;

					//send msg
					MSG2 m;
					m.r = ID();
					m.conv = conv;
					for (int k = 0; k < K; k++)
						for (int i = 0; i < L; i++)
							m.x[k][i] = x[k][i];

					void *m_pt = &m;
					emitter->send(m_pt, sizeof(MSG2));

					Fin.clear();
					for (RSet::iterator it = N.begin(); it != N.end(); it++)
					{
						if (!msglist[*it].empty())
						{
							Conv conv_k = msglist[*it].back().conv;
							temp = ConvexHullMerge(conv, conv_k);
							for (int k = 0; k < K; k++)
								for (int i = 0; i < L; i++)
									x[k][i] =x[k][i] || msglist[*it].back().x[k][i];

							Fin.insert(*it);
							msglist[*it].pop_back();
						}
					}

				}//end if have received msg from all robots in N 

			}//end deal with the msg

			//pop the msg please!
			receiver->nextPacket();

		}//end if init or not

	}//end while of ConvexHullConstruction

	
	
	
}

//Distribute cardinality estimation for each robot ri
void Intel_robot::CardinalityEstimation(void)
{
	int r[L - 1];

	srand((int)time(0) + ID());

	for (int k = 0; k < K; k++)
	{
		int y = 0;

		
		for (int i = 0; i < L - 1; i++)
			r[i] = rand() % 2;

		for (int i = 0; i < L - 1; i++)
		{
			if (r[i] == 1)
				break;
			else
				y++;
		}

		x[k][y] = 1;
	}
}

//Determination of the center and radius of the cirle to be formed for each robot ri
void Intel_robot::Determination(void)
{
	int sum_y = 0;
	for (int k = 0; k < K; k++)
	{
		int y = 0;
		for (int i = 0; i < L; i++)
		{
			if (x[k][i] == 0)
				break;
			y++;
		}
		sum_y += y;
	}
	double p = 1.2897*pow(2, sum_y / K);

	for (int i = 0; i <= conv.Top; i++)
	{
		Vector2d temp = conv.points[i].p*(1.0 / (conv.Top + 1.0));
		O = O + temp;
	}

	double area = 0;
	for (int i = 0; i < conv.Top; i++)
	{
		area += conv.points[i].p % conv.points[i + 1].p;
	}
	area += conv.points[conv.Top].p % conv.points[0].p;
	area = fabs(area / 2.0);

	double q = gamma * area / PI / size / size;
	
	double n = min(p, q);

	R = size / sin(PI / n);

	if (R > 0.70)
	{
		R = 0.70;
	}
	else if(R < 0.6)
	{
		R = 0.6;
	}
	cout << Name() << " O:" << O << ", R:" << R << endl;

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
void  Intel_robot::UniformTransformation(void)
{
	Vector2d pos = Position();
	SList Sa = SurroundRobots_A();	
	
	//find the most nearby robot clockwise and cal the d+
	double minRad = -PI;
	
	for (SList::iterator it = Sa.begin(); it != Sa.end(); it++)
	{
		//if (fabs(Dist(*it, O) - R) > DistError)
		//{
			//cout<< Name() << ": Waiting: some robot don't reach the circle." << endl;
			//return;
		//}

		double rad = Rad(pos, *it, O);

		if (rad < 0 && rad > minRad)
			minRad = rad;

	}

	//print to debug
	/*cout << Name() <<":Nearby Robots  : { ";
	for (SList::iterator iter = Sa.begin(); iter != Sa.end(); iter++)
	{
		cout << "( " << Rad(pos,*iter,O) << " ), ";
	}
	cout << "} minRad:" << minRad << endl;
	*/

	//if d+ > d
	if (abs(minRad) > d && fabs(minRad+d)>2*AngleError)
	{
		Vector2d target_circle = (pos - O).rotate(d + minRad);
		Vector2d target = target_circle + O;
		PIDForward(target);
	}

}