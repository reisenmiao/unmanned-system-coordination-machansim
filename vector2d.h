#pragma once
#include "common.h"

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

	friend ostream &operator<<(ostream &s, Vector2d&v);

	// Euclidean norm
	double norm();

	Vector2d normalize();

	Vector2d rotate(double rad);

};

//cal the distance
double Dist(Vector2d p1, Vector2d p2);

//cal the midpoint
Vector2d Midpoint(Vector2d p1, Vector2d p2);

//cal the intersection of a ray ang a circle
Vector2d Intersection(Vector2d P, Vector2d O, double R, double bias = 0);