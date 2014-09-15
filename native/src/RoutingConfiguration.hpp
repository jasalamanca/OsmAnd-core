/*
 * RoutingConfiguration.hpp
 *
 *  Created on: 28/08/2014
 *      Author: javier
 */

#ifndef ROUTINGCONFIGURATION_HPP_
#define ROUTINGCONFIGURATION_HPP_

#include "generalRouter.h"

struct RoutingConfiguration
{
	typedef UNORDERED(map)<std::string, std::string> MAP_STR_STR;

	GeneralRouter router;

	int memoryLimitation;
	float initialDirection;

	int zoomToLoad;
	float heurCoefficient;
	int planRoadDirection;

	void initParams(MAP_STR_STR& attributes) {
		planRoadDirection = (int) parseFloat(attributes, "planRoadDirection", 0);
		heurCoefficient = parseFloat(attributes, "heuristicCoefficient", 1);
		// don't use file limitations?
		memoryLimitation = (int)parseFloat(attributes, "nativeMemoryLimitInMB", memoryLimitation);
		zoomToLoad = (int)parseFloat(attributes, "zoomToLoadTiles", 16);
	}

	RoutingConfiguration(float initDirection = -360, int memLimit = 64) :
			memoryLimitation(memLimit), initialDirection(initDirection) {
	}
};

#endif /* ROUTINGCONFIGURATION_HPP_ */
