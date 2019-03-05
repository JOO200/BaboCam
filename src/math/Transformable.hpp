/*
 * Transformable.hpp
 *
 *  Created on: Mar 5, 2019
 *      Author: johannes
 */

#ifndef SRC_MATH_TRANSFORMABLE_HPP_
#define SRC_MATH_TRANSFORMABLE_HPP_

#include "Vector2D.hpp"
#include <ctime>

class MotionData {
	Vector2D diff;
	time_t start;
	time_t end;
};


class ITransformable {
public:
	virtual ~ITransformable();
	virtual void transform(MotionData & data) = 0;
};



#endif /* SRC_MATH_TRANSFORMABLE_HPP_ */
