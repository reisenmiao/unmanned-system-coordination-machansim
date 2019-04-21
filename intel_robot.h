#pragma once
#include "contr_robot.h"


class Intel_robot :public Contr_robot
{
private:
	int AMOUNT;
	Vector2d O;
	double R;
	double d;

public:
	//init
	Intel_robot(void);

	//NetWork construction for each robot ri
	RSet NetConstruction(void);

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