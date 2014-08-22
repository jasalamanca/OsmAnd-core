/*
 *  RoutingIndex.hpp
 *
 *  Created on: 09/06/2014
 *      Author: javier
 */

#ifndef _ROUTING_INDEX_HPP
#define _ROUTING_INDEX_HPP

#if defined(_WIN32)
#include <io.h>
#else
#include <unistd.h>
#endif
#include <map>
#include "Common.h"
#include "common2.h"

#if defined(WIN32)
#define close _close
#endif

#include "Map.hpp"
#include "binaryRead.h"

#include <boost/geometry/algorithms/intersects.hpp>
#include <boost/geometry/algorithms/equals.hpp>////
#include <boost/geometry/algorithms/covered_by.hpp>////
#include <boost/range/algorithm/for_each.hpp>
#include <boost/range/algorithm/copy.hpp>
#include <boost/function.hpp>

#include <Logging.h>

// Full response type
//TODO typedef std::vector<RouteDataObject> RouteDataObjects_t;
typedef std::vector<RouteDataObject *> RouteDataObjects_t;

struct RouteDataObject {
	RoutingIndex* region;
	std::vector<uint32_t> types ;
	std::vector<uint32_t> pointsX ;
	std::vector<uint32_t> pointsY ;
	std::vector<uint64_t> restrictions ;
	std::vector<std::vector<uint32_t> > pointTypes;
	int64_t id;

	UNORDERED(map)<int, std::string > names;
	std::vector<std::pair<uint32_t, uint32_t> > namesIds;

	RouteDataObject() : box(point_t(INT_MAX, INT_MAX), point_t(-1, -1))
	{}

	std::string getName() {
		if(names.size() > 0) {
			return names.begin()->second;
		}
		return "";
	}

	inline int64_t getId() {
		return id;
	}

	size_t getSize() const
	{
		int s = sizeof(this);
		s += pointsX.capacity()*sizeof(uint32_t);
		s += pointsY.capacity()*sizeof(uint32_t);
		s += types.capacity()*sizeof(uint32_t);
		s += restrictions.capacity()*sizeof(uint64_t);
		std::vector<std::vector<uint32_t> >::const_iterator t = pointTypes.begin();
		for(;t!=pointTypes.end(); t++) {
			s+= (*t).capacity() * sizeof(uint32_t);
		}
		s += namesIds.capacity()*sizeof(std::pair<uint32_t, uint32_t>);
		s += names.size()*sizeof(std::pair<int, std::string>)*10;
		return s;
	}

	inline int getPointsLength() {
		return pointsX.size();
	}

	bool loop(){
		return pointsX[0] == pointsX[pointsX.size() - 1] && pointsY[0] == pointsY[pointsY.size() - 1] ;
	}

	bool roundabout();

	double directionRoute(int startPoint, bool plus) const {
		// look at comment JAVA
		return directionRoute(startPoint, plus, 5);
	}

	// Gives route direction of EAST degrees from NORTH ]-PI, PI]
	double directionRoute(size_t startPoint, bool plus, float dist) const {
		int x = pointsX[startPoint];
		int y = pointsY[startPoint];
		size_t nx = startPoint;
		int px = x;
		int py = y;
		double total = 0;
		do {
			if (plus) {
				if (++nx >= pointsX.size()) {
					break;
				}
			} else {
				if (--nx < 0) {
					break;
				}
			}
			px = pointsX[nx];
			py = pointsY[nx];
			// translate into meters
			// TODO review distance criteria
			total += abs(px - x) * 0.011 + abs(py - y) * 0.01863;
		} while (total < dist);

        if ((x == px) && (y == py))
        {
                // Calculate bearing reverse way and adjust.
                return alignAngleDifference(directionRoute(startPoint, !plus, dist) - M_PI);
        }
		return -atan2(x - px, y - py);
	}

	static double parseSpeed(std::string const & v, double def) {
		if(v == "none") {
			return 40;// RouteDataObject::NONE_MAX_SPEED;
		} else {
			int i = findFirstNumberEndIndex(v);
			if (i > 0) {
				double f = atof(v.substr(0, i).c_str());
				f /= 3.6; // km/h -> m/s
				if (v.find("mph") != std::string::npos) {
					f *= 1.6;
				}
				return f;
			}
		}
		return def;
	}

	static double parseLength(std::string const & v, double def) {
		// 14"10' not supported
		int i = findFirstNumberEndIndex(v);
		if (i > 0) {
			double f = atof(v.substr(0, i).c_str());
			if (v.find("\"") != std::string::npos  || v.find("ft") != std::string::npos) {
				// foot to meters
				f *= 0.3048;
			}
			return f;
		}
		return def;
	}

	static double parseWeightInTon(std::string const & v, double def) {
		int i = findFirstNumberEndIndex(v);
		if (i > 0) {
			double f = atof(v.substr(0, i).c_str());
			if (v.find("\"") != std::string::npos || v.find("lbs") != std::string::npos) {
				// lbs -> kg -> ton
				f = (f * 0.4535) / 1000.0;
			}
			return f;
		}
		return def;
	}

	void query(bbox_t const & b, RouteDataObjects_t & result) const
	{
		if (boost::geometry::intersects(b, Box()))
			result.push_back(const_cast<RouteDataObject *>(this));
	}

	bbox_t const & Box() const
	{
		return box;
	}
	void Box(bbox_t const & b)
	{
		box = b;
	}

private:
	bbox_t box;
};

struct RouteSubregion
{
	typedef std::vector<RouteSubregion> SubRegions_t;
	typedef boost::function<void(RouteSubregion &)> Reader_t;

	uint32_t filePointer;
	uint32_t left;
	uint32_t right;
	uint32_t top;
	uint32_t bottom;
	RoutingIndex* routingIndex;

	RouteSubregion(RoutingIndex* ind) : routingIndex(ind){
	}

	// TODO extract as algorithm and do a lazy implementation.
	// Incrementally add results
	void query(bbox_t const & b, RouteDataObjects_t & result) const
	{
		using boost::geometry::intersects;
		using boost::range::for_each;
		using boost::range::copy;

		// I can't say anything
		if (!intersects(b, box))
			return;

//std::cerr << " RSR query box? " << b << " in " << box << std::endl;
		// Before using data
		((RouteSubregion *)this)->readContent();  // TODO cast????

		// THINK: Maybe using boost::function to do only 1 call (one of them do nothing)
		for_each(subregions,
				 [&b, &result](RouteSubregion const & node){node.query(b, result);});
		for_each(dataObjects,
				[&b, &result](RouteDataObject const * obj){if (obj != NULL) obj->query(b, result);});
//// TODO add boxes to DataObject and propagate query to DataObjects.
////		copy(dataObjects, std::back_inserter(result));
	}

	// TODO Remove as soon as possible
	void querySub(bbox_t const & b, RouteDataObjects_t & result) const
	{
//std::cerr << " RSR querySub box? " << b << " in " << box << std::endl;
		using boost::range::for_each;
		using boost::range::copy;

		// I can't say anything
		if (!boost::geometry::covered_by(b, box))
			return;

		// Before using data
		((RouteSubregion *)this)->readContent();  // TODO cast????

//std::cerr << " RSR querySub OK" << std::endl;
		for_each(subregions,
				 [&b, &result](RouteSubregion const & node){node.querySub(b, result);});
		if (boost::geometry::equals(b, box))
		{
//std::cerr << " RSR querySub #RDO " << dataObjects.size() << std::endl;
			copy(dataObjects, std::back_inserter(result));
		}
	}

	// TODO Remove as soon as possible
	void queryLeafNode(bbox_t const & b, std::vector<RouteSubregion> & result) const
	{
		using boost::geometry::intersects;

		// I can't say anything
		if (!intersects(b, box))
			return;

		// Before using data
		((RouteSubregion *)this)->readContent();  // TODO cast????

		// Add this one
		if (dataObjects.size() > 0) result.push_back(*this);
	}

	void ContentReader(Reader_t r)
	{
		contentReader = r;
	}

	void DataObjects(RouteDataObjects_t && objs)
	{
		dataObjects = objs;
	}
	// Be careful
	void Box(bbox_t const & b)
	{
		box = b;
	}
	bbox_t const & Box() const
	{
		return box;
	}
private:

	// Read if not yet
	void readContent()
	{
		if (!hasContent())
			if (contentReader)
				contentReader(*this);
	}

	// Both data structures empty
	bool hasContent() const
	{
		return !subregions.empty() || !dataObjects.empty();
	}

	bbox_t box;

	// Could be an intermediate or leaf node depending on what data has.
public: 	SubRegions_t subregions; private:
	RouteDataObjects_t dataObjects;

	Reader_t contentReader;
};

struct RoutingIndex : BinaryPartIndex
{
	std::vector<tag_value> decodingRules;
	typedef std::vector<RouteSubregion> regions_t;
	regions_t subregions;
	regions_t basesubregions;
	RoutingIndex()
	: BinaryPartIndex(ROUTING_INDEX),
	  box(point_t(INT_MAX, INT_MAX), point_t(-1, -1))
	{}

	// TODO What really is basemap????
	void query(bbox_t const & b, bool base, RouteDataObjects_t & result) const
	{
//std::cerr << "RI.query " << (base?"basemap ":"map ") << "box? " << b << " in " << box << std::endl;
		using boost::geometry::intersects;
		using boost::range::for_each;
		using boost::range::copy;

		// I can't say anything
		if (!intersects(b, box))
			return;

		auto & rs = base?basesubregions:subregions;
		for_each(rs,
				 [&b, &result](RouteSubregion const & node){node.query(b, result);});
	}

	// Remove as soon as possible
	void querySub(bbox_t const & b, bool base, RouteDataObjects_t & result) const
	{
		using boost::range::for_each;

		// I can't say anything
		if (!boost::geometry::covered_by(b, box))
			return;

		auto & rs = base?basesubregions:subregions;
		for_each(rs,
				 [&b, &result](RouteSubregion const & node){node.querySub(b, result);});
	}

	// Remove as soon as possible
	void queryLeafNodes(bbox_t const & b, bool base, std::vector<RouteSubregion> & result) const
	{
////std::cerr << "RI.queryLNodes " << (base?"basemap ":"map ") << "box? " << b << " in " << box << std::endl;
		using boost::geometry::intersects;
		using boost::range::for_each;
		using boost::range::copy;

		// I can't say anything
		if (!intersects(b, box))
			return;

		auto & rs = base?basesubregions:subregions;
		for_each(rs,
				 [&b, &result](RouteSubregion const & node){node.queryLeafNode(b, result);});
	}

	void initRouteEncodingRule(uint32_t id, std::string const & tag, std::string const & val) {
		tag_value pair = tag_value(tag, val);
		while(decodingRules.size() < id + 1){
			decodingRules.push_back(pair);
		}
		decodingRules[id] = pair;
	}

	bbox_t const & Box() const
	{
		return box;
	}

	void Box(bbox_t & b)
	{
		box = b;
	}

private:
	bbox_t box;
};

#endif // _ROUTING_INDEX_HPP
