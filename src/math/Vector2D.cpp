/*
 * Vector3D.cpp
 *
 *  Created on: Mar 4, 2019
 *      Author: johannes
 */
#include "Vector2D.hpp"

Vector2D::Vector2D(Vector2D & old) {
	x = old.getX();
	y = old.getY();
	timestamp = time(NULL);
}

Vector2D::Vector2D(double oldX, double oldY) {
	x = oldX;
	y = oldY;
	timestamp = time(NULL);
}

Vector2D* Vector2D::add(Vector2D& v) {
	x += v.getX();
	y += v.getY();
	return this;
}

Vector2D* Vector2D::add(double px, double py) {
	x += px;
	y += py;
	return this;
}

Vector2D* Vector2D::divide(Vector2D& v) {
	x /= v.getX();
	y /= v.getY();
	return this;
}

Vector2D* Vector2D::divide(double px, double py) {
	x /= px;
	y /= py;
	return this;
}

