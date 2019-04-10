/*
 * Vector3D.hpp
 *
 *  Created on: Mar 4, 2019
 *      Author: johannes
 */

#ifndef SRC_MATH_VECTOR2D_HPP_
#define SRC_MATH_VECTOR2D_HPP_

#include "Transformable.hpp"
#include <ctime>

class Vector2D : public ITransformable {
public:
	Vector2D(Vector2D & old);
	Vector2D(double, double);
	double getX() 				{ return x; }
	Vector2D setX(double newX) 	{ x = newX; return this; }
	double getY() 				{ return y; }
	Vector2D setY(double newY) 	{ y = newY; return this; }
	Vector2D add(Vector2D & v);
	Vector2D add(double, double);
	Vector2D subtract(Vector2D & v);
	Vector2D subtract(double, double);
	Vector2D multiply(Vector2D & v);
	Vector2D multiply(double, double);
	Vector2D multiply(double n) { return multiply(n,n); }
	Vector2D divide(Vector2D & v);
	Vector2D divide(double, double);
	Vector2D divide(double n) 	{ return divide(n,n); }

	Vector2D normalize() 		{ return divide(length()); }
	double length();

	virtual void transform(MotionData & data);

private:
	double x, y;
	time_t timestamp;
};



#endif /* SRC_MATH_VECTOR2D_HPP_ */
