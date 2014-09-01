/*
 * PrecalculatedRouteDirection.hpp
 *
 *  Created on: 28/08/2014
 *      Author: javier
 */

#ifndef PRECALCULATEDROUTEDIRECTION_HPP_
#define PRECALCULATEDROUTEDIRECTION_HPP_

#include <cstdint>
#include "common2.h"

struct PrecalculatedRouteDirection {
	std::vector<uint32_t> pointsX;
	std::vector<uint32_t> pointsY;
	std::vector<float> times;
	float minSpeed;
	float maxSpeed;
	float startFinishTime;
	float endFinishTime;
	bool followNext;
	static int SHIFT;
	static int SHIFTS[];
	bool empty;

	uint64_t startPoint;
	uint64_t endPoint;
	quad_tree<int> quadTree;

 	inline uint64_t calc(int x31, int y31) {
		return (((uint64_t) x31) << 3l) + ((uint64_t)y31);
	}

 	float getDeviationTime(int x31, int y31)
 	{
 		return getDeviationDistance(x31, y31) / maxSpeed;
 	}
	float getDeviationDistance(int x31, int y31)
	{
		int ind = getIndex(x31, y31);
		if(ind == -1) {
			return 0;
		}
		return getDeviationDistance(x31, y31, ind);
	}
	float getDeviationDistance(int x31, int y31, int ind);
	int getIndex(int x31, int y31);
	float timeEstimate(int begX, int begY, int endX, int endY);
};

#endif /* PRECALCULATEDROUTEDIRECTION_HPP_ */
