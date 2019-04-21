#pragma once
#include "common.h"

//
//This is the mathmatic functions for form_circle project
//And there are some things that you should pay attention
//1.when you calculate angle ,make sure the vector has been normalized
//

class Vector2d
{
private:

	double z;
	double x;

public:
	Vector2d(double z = 0.0 ,double x = 0.0);

	//basic calculation
	Vector2d operator +(Vector2d &v);
	Vector2d operator -(Vector2d &v);
	Vector2d operator*(double scaler);
	double operator*(Vector2d &v);
	
	//cross(just return one signed value)
	double operator%(Vector2d &v);

	friend ostream &operator<<(ostream &s, Vector2d&v);

	// Euclidean norm
	double norm();

	Vector2d normalize();

	//rad
	double Rad();

	Vector2d rotate(double rad);

};

//cal the distance
double Dist(Vector2d p1, Vector2d p2);

double Rad(Vector2d v1, Vector2d v2);

double Rad(Vector2d v1, Vector2d v2, Vector2d O);

//cal the midpoint
Vector2d MidPoint(Vector2d p1, Vector2d p2);

//cal the intersection of a ray ang a circle
Vector2d Intersection(Vector2d P, Vector2d O, double R, double bias = 0);


