#include "vector2d.h"

Vector2d::Vector2d(double z , double x)
{
	this->z = z;
	this->x = x;
}

Vector2d Vector2d::operator+(Vector2d &v)
{
	return Vector2d(z+v.z,x+v.x);
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

//cal the intersection of a ray ang a circle
Vector2d Intersection(Vector2d P, Vector2d O, double R, double bias)
{

	Vector2d ray_OP = (P - O).normalize();

	if (bias)
		ray_OP.rotate(bias);

	return ray_OP * R + O;

}