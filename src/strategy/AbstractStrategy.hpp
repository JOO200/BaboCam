/*
 * AbstractStrategy.hpp
 *
 *  Created on: Mar 4, 2019
 *      Author: johannes
 */

#ifndef SRC_STRATEGY_ABSTRACTSTRATEGY_HPP_
#define SRC_STRATEGY_ABSTRACTSTRATEGY_HPP_

#include "../struct/Context.hpp"
#include "../interfaces/MovableDevice.hpp"

class AbstractStrategy {
public:
	AbstractStrategy() = default;
	virtual ~AbstractStrategy() = default;

	virtual void step(Context * curr_context, MovableDevice * kobuki, std::atomic<bool>& m_stop) = 0;
};



#endif /* SRC_STRATEGY_ABSTRACTSTRATEGY_HPP_ */
