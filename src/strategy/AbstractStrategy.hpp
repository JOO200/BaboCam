/*
 * AbstractStrategy.hpp
 *
 *  Created on: Mar 4, 2019
 *      Author: johannes
 */

#ifndef SRC_STRATEGY_ABSTRACTSTRATEGY_HPP_
#define SRC_STRATEGY_ABSTRACTSTRATEGY_HPP_

class AbstractStrategy {
public:
	AbstractStrategy();
	virtual ~AbstractStrategy();

	virtual void step() = 0;
	virtual AbstractStrategy* changeCheck() = 0;
};



#endif /* SRC_STRATEGY_ABSTRACTSTRATEGY_HPP_ */
