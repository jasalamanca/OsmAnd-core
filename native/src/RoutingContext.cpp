/*
 * RoutingContext.cpp
 *
 *  Created on: 28/08/2014
 *      Author: javier
 */

#include <iostream>
#include "RoutingContext.hpp"

#include "common2.h"
#include <boost/range/adaptor/filtered.hpp>

void RoutingQuery(bbox_t & b, RouteDataObjects_t & output);
//extern const bool TRACE_ROUTING;

SHARED_PTR<RouteSegment> RoutingContext::findRouteSegment(uint32_t x31, uint32_t y31)
{
	bbox_t b = boost::geometry::make<bbox_t>(x31-100, y31-100, x31+100, y31+100);
	RouteDataObjects_t dataObjects;
	RoutingQuery(b, dataObjects);

	auto filter = [this](RouteDataObject_pointer const & rdo)
			{
		return rdo != nullptr && acceptLine(rdo);
			};
	auto range = boost::adaptors::filter(dataObjects, filter);
	if (range.begin() == range.end())
	{
		// A second try.
		b = boost::geometry::make<bbox_t>(x31-20000, y31-20000, x31+20000, y31+20000);
		RoutingQuery(b, dataObjects);
	}
	// Candidate
	RouteDataObject_pointer road;
	size_t index = 0;
	int candidateX = -1;
	int candidateY = -1;
	double sdist = 0;
	RouteDataObjects_t::const_iterator it = dataObjects.begin();
	for (; it!= dataObjects.end(); it++) {
		RouteDataObject_pointer const & r = *it;
		if (!filter(r))
		{
//if (TRACE_ROUTING)
//{
//std::cerr << "FindRS NAT ";
//if (r == nullptr)
//	std::cerr << "NULL";
//else
//	std::cerr << r->id << " NOT accepted";
//std::cerr << std::endl;
//}
			continue;
		}
		for (size_t j = 1; j < r->pointsX.size(); ++j)
		{
			// (px, py) projection over (j-1)(j) segment
			std::pair<int, int> pr = calculateProjectionPoint31(r->pointsX[j-1], r->pointsY[j-1],
					r->pointsX[j], r->pointsY[j],
					x31, y31);
			// Both distance and squared distance (we use) are monotone and positive functions.
			double currentsDist = squareDist31TileMetric(pr.first, pr.second, x31, y31);
			if (currentsDist < sdist || road == nullptr)
			{
				// New candidate
				road = r;
				index = j;
				candidateX = pr.first;
				candidateY = pr.second;
				sdist = currentsDist;
//std::cerr << "FindRS candidate " << road->id << "[" << index << "]=(" << candidateX << ',' << candidateY << ") dist=" << sdist << std::endl;
			}
		}
	}
	if (road != nullptr)
	{
		/*** Always add a new point to road to allow 'RoutePlannerFrontEnd.searchRoute' postprocessing.
		if ((candidateX == road->pointsX[index-1]) && (candidateY == road->pointsY[index-1]))
		{
			// Projection has same coordinates. None new.
			return (SHARED_PTR<RouteSegment>) new RouteSegment(road, index-1);
		}
		if ((candidateX == road->pointsX[index]) && (candidateY == road->pointsY[index]))
		{
			// Projection has same coordinates. None new.
			return (SHARED_PTR<RouteSegment>) new RouteSegment(road, index);
		}
		***/

		SHARED_PTR<RouteDataObject> proj = SHARED_PTR<RouteDataObject>(new RouteDataObject(*road));
		proj->pointsX.insert(proj->pointsX.begin() + index, candidateX);
		proj->pointsY.insert(proj->pointsY.begin() + index, candidateY);
		if (proj->pointTypes.size() > index) {
			proj->pointTypes.insert(proj->pointTypes.begin() + index, std::vector<uint32_t>());
		}
		// re-register the best road because one more point was inserted
		registerRouteDataObject(proj);
		return SHARED_PTR<RouteSegment>(new RouteSegment(proj, index));
	}
	return nullptr;
}

SHARED_PTR<RouteSegment> RoutingContext::loadRouteSegment(uint32_t x31, uint32_t y31)
{
	UNORDERED(set)<int64_t> excludeDuplications;
	SHARED_PTR<RouteSegment> original;
	// First search on new roads. Newer is better.
	// FIXME Revision on every loadRouteSegment is crazy. We must not update map or change how we update.
	for (int i = registered.size()-1; i >= 0; --i)
	{
		SHARED_PTR<RouteDataObject> const & ro = registered[i];
		for (size_t index = 0; index < ro->pointsX.size(); ++index)
		{
			int64_t roId = calcRouteId(ro, index);
			if (excludeDuplications.count(roId) == 0) {
				excludeDuplications.insert(roId);
				if (x31 == ro->pointsX[index] && y31 == ro->pointsY[index])
				{
					SHARED_PTR<RouteSegment> s = SHARED_PTR<RouteSegment>(new RouteSegment(ro, index));
					s->next = original;
					original = 	s;
				}
			}
		}
	}

	// Second search on map info.
	int64_t key = makeKey(x31, y31);
	if (connections.count(key) == 0)
		loadMap(x31, y31);
	SHARED_PTR<RouteSegment> segment = connections[key];

	if (segment == nullptr)
		std::cerr << "loadSegment(" << x31 << ',' << y31 << ")=NULL" << std::endl;

	while (segment != nullptr)
	{
		SHARED_PTR<RouteDataObject> const & ro = segment->road;
		int sStart = segment->segmentStart;
		int64_t roId = calcRouteId(ro, sStart);
		if (excludeDuplications.count(roId) == 0) {
			excludeDuplications.insert(roId);
			// TODO Avoid this copies
			// Dued to registered. A container could be better than a linked structure.
			SHARED_PTR<RouteSegment> s = SHARED_PTR<RouteSegment>(new RouteSegment(ro, sStart));
			s->next = original;
			original = 	s;
		}
		segment = segment->next;
	}
	return original;
}

void RoutingContext::add(SHARED_PTR<RouteDataObject> const & o, bbox_t const & b)
{
	for (int i = o->pointsX.size()-1; i >= 0; --i)
	{
		uint32_t x31 = o->pointsX[i];
		uint32_t y31 = o->pointsY[i];
		if (!boost::geometry::covered_by(point_t(x31, y31), b)) continue;
		int64_t l = makeKey(x31, y31);
		SHARED_PTR<RouteSegment> segment = SHARED_PTR<RouteSegment>(new RouteSegment(o, i));
		if (connections.count(l) != 0)
			segment->next = connections[l];
		connections[l] = segment;
	}
}

void RoutingContext::loadMap(int x31, int y31)
{
	// Greatly affects execution time. Has a minimum around 14.
	static int const GRANULARITY = 14;
	x31 >>= GRANULARITY;
	y31 >>= GRANULARITY;
	int64_t key = makeKey(x31, y31);
	// Load data from map.
	if (loaded.count(key) == 0)
	{
		loaded.insert(key);

		using boost::geometry::make;
		bbox_t b = make<bbox_t>(x31 << GRANULARITY, y31 << GRANULARITY,
				(x31+1) << GRANULARITY, (y31+1) << GRANULARITY);
		RouteDataObjects_t objects;
		timeToLoad.Start();
		RoutingQuery(b, objects);
		timeToLoad.Pause();
		for (int i = objects.size()-1; i >= 0; --i)
		{
			RouteDataObject_pointer const & ro(objects[i]);
			if (ro != nullptr && acceptLine(ro))
				add(ro, b);
		}
	}
}

// Register modified route data objects with a live time equal to that of context.
void RoutingContext::registerRouteDataObject(SHARED_PTR<RouteDataObject> const & o)
{
	registered.push_back(SHARED_PTR<RouteDataObject>(o));
}
