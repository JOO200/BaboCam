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

/**
 * abstrakte Klasse einer Strategie.
 * Vererbende Klassen müssen die Methode step(..) überschreiben.
 *
 * Bei der Erstellung dieses Systems war die Idee, dass man eine Liste von Strategien speichert und bei jedem Zyklus
 * durch eine Iteration über alle Strategien die best "geeignete" Strategie detektiert werden. Dazu fehlt noch eine
 * Methode, mit welcher sich die Dringlichkeit der Strategie abrufen lässt.
 * So würde ein "notstop" eine hohe Priorität haben und alle anderen Strategien sofort überschreiben.
 */
class AbstractStrategy {
public:
	AbstractStrategy() = default;
	virtual ~AbstractStrategy() = default;

	/**
	 *
	 * @param curr_context Verwendeter Kontext
	 * @param kobuki Treiber des selbstfahrenden Roboters, z.B. Kobuki Turtlebot2
	 * @param m_stop Anweisung, womit der Roboter vollständig herunterfährt.
	 */
	virtual void step(Context * curr_context, MovableDevice * kobuki, std::atomic<bool>& m_stop) = 0;
};



#endif /* SRC_STRATEGY_ABSTRACTSTRATEGY_HPP_ */
