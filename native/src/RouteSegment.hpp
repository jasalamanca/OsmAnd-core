/*
 * RouteSegment.hpp
 *
 *  Created on: 28/08/2014
 *      Author: javier
 */

#ifndef ROUTESEGMENT_HPP_
#define ROUTESEGMENT_HPP_

#include <Common.h>

struct RouteSegment {
	int segmentStart;
	SHARED_PTR<RouteDataObject> road;

	// needed to store intersection of routes
	SHARED_PTR<RouteSegment> next;

	// search context (needed for searching route)
	// Initially it should be null (!) because it checks was it segment visited before
	SHARED_PTR<RouteSegment> parentRoute;
	int parentSegmentEnd;

	// distance measured in time (seconds)
	float distanceFromStart;
	float distanceToEnd;

	inline SHARED_PTR<RouteDataObject> const & getRoad() const {
		return road;
	}

	inline int getSegmentStart() const {
		return segmentStart;
	}

	inline float f() const
	{
		return distanceFromStart + distanceToEnd;
	}

	inline float g() const
	{
		return distanceFromStart;
	}

	inline float h() const
	{
		return distanceToEnd;
	}

	RouteSegment(SHARED_PTR<RouteDataObject> const & road, int segmentStart)
	: segmentStart(segmentStart), road(road),
	  next(), parentRoute(), parentSegmentEnd(0),
	  distanceFromStart(0), distanceToEnd(0){
	}
	~RouteSegment(){
	}
};

struct RouteSegmentResult {
	SHARED_PTR<RouteDataObject> object;
	int startPointIndex;
	int endPointIndex;
	float routingTime;
	std::vector<std::vector<RouteSegmentResult> > attachedRoutes;
	RouteSegmentResult(SHARED_PTR<RouteDataObject> const & object, int startPointIndex, int endPointIndex) :
		object(object), startPointIndex(startPointIndex), endPointIndex (endPointIndex), routingTime(0) {
	}
};

struct FinalRouteSegment {
	SHARED_PTR<RouteSegment> direct;
	bool reverseWaySearch;
	SHARED_PTR<RouteSegment> opposite;
	float distanceFromStart;
};

#endif /* ROUTESEGMENT_HPP_ */
