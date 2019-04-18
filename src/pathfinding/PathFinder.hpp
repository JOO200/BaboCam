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
#include "../interfaces/IRunnable.hpp"
#include "../extern/kobuki_driver/include/kobuki_driver/kobuki.hpp"

class PathFinder : public IRunnable {
public:
	PathFinder(Context * context, kobuki::Kobuki * device)
	    :context(context),device(device) {}

protected:
	void run() override;
private:
	kobuki::Kobuki * device;
	Context * context;
};



#endif /* SRC_PATHFINDING_PATHFINDER_HPP_ */
