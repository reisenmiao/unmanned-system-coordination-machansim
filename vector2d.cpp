#include "vector2d.h"

Vector2d::Vector2d(double z , double x)
{
	this->z = z;
	this->x = x;
}

Vector2d Vector2d::operator+(Vector2d &v)
{
	z = z + v.z;
	x = x + v.x;
	return *this;
}

Vector2d Vector2d::operator-(Vector2d &v)
{
	return Vector2d(z - v.z, x - v.x);
}

Vector2d Vector2d::operator*(double scaler)
{
	return Vector2d(z*scaler, x*scaler);
}

double Vector2d::operator*(Vector2d &v)
{
	return z * v.z + x * v.x;
}

//cross(just return one signed value)
double Vector2d::operator%(Vector2d &v)
{
	return z * v.x - x * v.z;
}

ostream &operator <<(ostream &s, Vector2d &v)
{
	s << "(" << v.z << ", " << v.x << ")";
	return s;
}

double Vector2d::norm()
{
	return sqrt(z * z + x * x);
}

double Vector2d::sqNorm()
{
	return z * z + x * x;
}

Vector2d Vector2d::normalize() 
{  
	//Non int
	double len = norm();

	if (len != 0) 
	{
		len = 1 / len;
		z *= len;
		x *= len;
	}

	return *this;
}

//rad
double Vector2d::Rad()
{
	Vector2d z_axis(1.0, 0.0);

	if (z_axis % (*this) >= 0)
		return acos(z / norm());
	else
		return -acos(z / norm());

}

Vector2d Vector2d::rotate(double rad)
{
	double z_ = z * cos(rad) - x * sin(rad);
	double x_ = z * sin(rad) + x * cos(rad);

	return Vector2d(z_, x_);
}

//calculate the distance
double Dist(Vector2d v1, Vector2d v2)
{
	return (v1 - v2).norm();
}


double Rad(Vector2d v1, Vector2d v2)
{
	double rad = v2.Rad() - v1.Rad();

	while (rad > PI)
		rad -= 2 * PI;

	while (rad <= -PI)
		rad += 2 * PI;

	return rad;
}

double Rad(Vector2d v1, Vector2d v2, Vector2d O)
{
	return Rad(v1 - O, v2 - O);
}

//cal the midpoint
Vector2d MidPoint(Vector2d v1, Vector2d v2)
{
	return (v1 + v2) * 0.5;
}

//convex hull construction
bool GrahamSort(ConvexPoint p1, ConvexPoint p2)
{
	double cross = p1.p % p2.p;
	if (cross > 0)
		return true;
	else if (abs(cross) <= 0.01 && p1.p.sqNorm() < p2.p.sqNorm())
		return true;
	else
		return 0;
}

Conv ConvexHullMerge(Conv conv1, Conv conv2)
{
	set<int> points;
	ConvexPoint lowest_point = conv1.points[0];

	//merge the points in two ConvexHulls and find the lowest point
	for (int i=0; i<= conv1.Top; i++)
	{
		points.insert(conv1.points[i].id);

		if (conv1.points[i].p.x < lowest_point.p.x || (conv1.points[i].p.x == lowest_point.p.x && conv1.points[i].p.z < lowest_point.p.z))
			lowest_point = conv1.points[i];
	}

	for (int i=0; i<=conv2.Top; i++)
	{
		if (points.find(conv2.points[i].id) == points.end())
		{
			conv1.points[++conv1.Top] = conv2.points[i];
			points.insert(conv2.points[i].id);
		}

		if (conv2.points[i].p.x < lowest_point.p.x || (conv2.points[i].p.x == lowest_point.p.x && conv2.points[i].p.z < lowest_point.p.z))
			lowest_point = conv2.points[i];

	}

	//transfer the points to the relative position to lowest point
	for (int i=0; i<=conv1.Top; i++)
	{
		conv1.points[i].p = conv1.points[i].p - lowest_point.p;
	}

	//arrange points Counterclockwise
	sort(conv1.points, conv1.points+conv1.Top+1, GrahamSort);

	//constuct the Convex Hull
	Conv conv;
	conv.Top = -1;
	conv.points[++conv.Top] = conv1.points[0];
	conv.points[++conv.Top] = conv1.points[1];

	for (int i=2; i <= conv1.Top; i++)
	{
		Vector2d v1, v2;
		v1 = conv.points[conv.Top].p - conv.points[conv.Top-1].p;
		v2 = conv1.points[i].p - conv.points[conv.Top - 1].p;
		while (conv.Top >= 1 && v1%v2 <= 0)
		{
			conv.Top--;
			v1 = conv.points[conv.Top].p - conv.points[conv.Top - 1].p;
			v2 = conv1.points[i].p - conv.points[conv.Top - 1].p;
		}
		conv.points[++conv.Top] = conv1.points[i];
	}

	//transfer relative position of points to the absolute position
	for (int i=0; i <= conv.Top; i++)
	{
		conv.points[i].p = conv.points[i].p + lowest_point.p;
	}

	return conv;

}

//cal the intersection of a ray ang a circle
Vector2d Intersection(Vector2d P, Vector2d O, double R, double bias)
{

	Vector2d ray_OP = (P - O).normalize();

	if (bias)
		ray_OP.rotate(bias);

	return ray_OP * R + O;

}