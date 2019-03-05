/*
 * Context.hpp
 *
 *  Created on: Mar 4, 2019
 *      Author: johannes
 */

#ifndef SRC_STRUCT_CONTEXT_HPP_
#define SRC_STRUCT_CONTEXT_HPP_

#include <vector>

struct ball_pos {
	double x, z;
	double distance;

} ;

struct context {
	ball_pos curr_ball;
};



#endif /* SRC_STRUCT_CONTEXT_HPP_ */
