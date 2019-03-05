/*
 * PathFinder.hpp
 *
 *  Created on: Mar 4, 2019
 *      Author: johannes
 */

#ifndef SRC_PATHFINDING_PATHFINDER_HPP_
#define SRC_PATHFINDING_PATHFINDER_HPP_

#include <mutex>
#include <condition_variable>
#include "../struct/Context.hpp"

class PathFinder {
public:
	PathFinder(context * context, std::mutex * data_access, std::condition_variable * cond_var);
	~PathFinder();
private:
	void run();

	std::mutex * data_access;
	std::condition_variable * cond_var;
	context * context;
};



#endif /* SRC_PATHFINDING_PATHFINDER_HPP_ */
