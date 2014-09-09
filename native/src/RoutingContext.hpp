/*
 * RoutingContext.hpp
 *
 *  Created on: 28/08/2014
 *      Author: javier
 */

#ifndef ROUTINGCONTEXT_HPP_
#define ROUTINGCONTEXT_HPP_

#include "Common.h"
#include <vector>

#include "PrecalculatedRouteDirection.hpp"
#include "RoutingConfiguration.hpp"
#include "RouteSegment.hpp"
#include "RouteCalculationProgress.hpp"
size_t RoutingMemorySize();

struct RoutingContext
{
public:
	RoutingContext(RoutingConfiguration& config)
		: visitedSegments(0),//// loadedTiles(0),
		  config(config), finalRouteSegment()
	{
		precalcRoute.empty = true;
	}

	// Public interface
	SHARED_PTR<RouteSegment> findRouteSegment(uint32_t x31, uint32_t y31);
	SHARED_PTR<RouteSegment> loadRouteSegment(uint32_t x31, uint32_t y31);

public:
	bool isInterrupted() const {
		return false;
	}
	// Context info
	RoutingConfiguration & config;
	int startX;
	int startY;
	int targetX;
	int targetY;
	////bool basemap;
	PrecalculatedRouteDirection precalcRoute;
	SHARED_PTR<FinalRouteSegment> finalRouteSegment;

	// Counters
	int visitedSegments;
	////int loadedTiles;
	OsmAnd::ElapsedTimer timeToLoad;
	OsmAnd::ElapsedTimer timeToCalculate;
	SHARED_PTR<RouteCalculationProgress> progress;

private:
	// Map representation for routing
	// To manage modified roads.
	std::vector<SHARED_PTR<RouteDataObject> > registered;
	// To memo map chuncks loaded
	UNORDERED(set)<int64_t> loaded;
	// To memo connections between roads.
	UNORDERED(map)<int64_t, SHARED_PTR<RouteSegment> > connections;

private:
	// Map related
	inline int64_t makeKey(int x, int y) const
	{
		return ((int64_t)x << 32) + y;
	}
	inline int64_t calcRouteId(SHARED_PTR<RouteDataObject const> const & o, int ind) {
		return ((int64_t) o->id << 10) + ind;
	}
	void loadMap(int x31, int y31);
	void add(SHARED_PTR<RouteDataObject> const & o, bbox_t const & b);
	bool acceptLine(SHARED_PTR<RouteDataObject> const & r) const
	{
		return config.router.acceptLine(r);
	}
	// Register modified route data objects with a live time equal to that of context.
	void registerRouteDataObject(SHARED_PTR<RouteDataObject> const & o);

public:
	// Counters
	size_t memorySize() const
	{
		return sizeof(RoutingContext)
				+ registered.capacity() * (sizeof(RouteDataObject) + sizeof(RouteDataObject_pointer))
				+ loaded.size() * sizeof(int64_t)
				+ connections.size() * (sizeof(std::pair<int64_t, std::shared_ptr<RouteSegment> >) + sizeof(RouteSegment))
				+ RoutingMemorySize();
	}

	inline size_t loadedMapChunks() const
	{
		return loaded.size();
	}
};

#endif /* ROUTINGCONTEXT_HPP_ */
