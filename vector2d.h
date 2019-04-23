#pragma once
#include "common.h"

//
//This is the mathmatic functions for form_circle project
//And there are some things that you should pay attention
//1.when you calculate angle ,make sure the vector has been normalized
//

class Vector2d
{
public:
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
	double norm(void);

	// squared Euclidean norm
	double sqNorm(void); 

	Vector2d normalize(void);

	//rad
	double Rad(void);

	Vector2d rotate(double rad);

};

//cal the distance
double Dist(Vector2d p1, Vector2d p2);

double Rad(Vector2d v1, Vector2d v2);

double Rad(Vector2d v1, Vector2d v2, Vector2d O);

//cal the midpoint
Vector2d MidPoint(Vector2d p1, Vector2d p2);

//convex hull construction
typedef struct { int id; Vector2d p; } ConvexPoint;
typedef struct Conv{ 
	ConvexPoint points[16]; int Top;
	bool friend operator == (Conv c1, Conv c2)
	{
		set<int> s1, s2;
		if (c1.Top != c2.Top)return false;
		for (int i = 0; i < c1.Top; i++)
		{
			s1.insert(c1.points[i].id);
			s2.insert(c2.points[i].id);
		}

		return s1 == s2;
	}
} Conv;
Conv ConvexHullMerge(Conv conv1, Conv conv2);

//cal the intersection of a ray ang a circle
Vector2d Intersection(Vector2d P, Vector2d O, double R, double bias = 0);



