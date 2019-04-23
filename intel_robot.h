#pragma once
#include "contr_robot.h"

const int L = 5;
const int K = 4;

class Intel_robot :public Contr_robot
{
private:
	
	RSet N;

	Conv conv;
	bool conv_init;

	int x[K][L] = { 0 };

	const double gamma = 1.5;
	const double size = 0.5;

	Vector2d O;
	int AMOUNT;
	double R;
	double d;

	typedef struct { int r; Vector2d p; } MSG;
	typedef struct { int r; Conv conv; int x[K][L]; } MSG2;

public:

	//initialization
	void Initialization(void);

	//NetWork construction for each robot ri
	void NetConstruction(void);

	//Distribute convex hull Constrction for each robot ri
	void ConvexHullConstruction(void);

	//Distribute cardinality estimation for each robot ri
	void CardinalityEstimation(void);

	//Determination of the center and radius of the cirle to be formed for each robot ri
	void Determination(void);

	//Circle Formation for each robot ri
	void CircleFormation(void);

	//Uniform transformation for each robot ri
	void UniformTransformation(void);

}; 