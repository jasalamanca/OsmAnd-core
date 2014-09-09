/*
 * MapIndex.hpp
 *
 *  Created on: 25/06/2014
 *      Author: javier
 */

#ifndef MAPINDEX_HPP_
#define MAPINDEX_HPP_

#include <vector>
#include "mapObjects.h"
#include "Map.hpp"
#include "BinaryIndex.hpp"
#include "SearchQuery.hpp"

#include <boost/geometry/algorithms/intersects.hpp>
#include <boost/range/algorithm/for_each.hpp>
#include <boost/range/algorithm/copy.hpp>
#include <boost/function.hpp>

#include <Logging.h>

// Full response type
typedef MapDataObject * MapDataObject_pointer;
typedef std::vector<MapDataObject_pointer> MapDataObjects_t;

struct MapTreeBounds
{
	typedef boost::function<void(MapTreeBounds &)> Reader_t;
	typedef std::vector<MapTreeBounds> Bounds_t;

	uint32_t length;
	uint32_t filePointer;
	uint32_t mapDataBlock;
	uint32_t left ;
	uint32_t right ;
	uint32_t top ;
	uint32_t bottom;
	bool ocean;

	MapTreeBounds()
	: ocean(-1),
	  box(point_t(INT_MAX, INT_MAX), point_t(-1, -1))
	{}

	inline void query(SearchQuery & q, MapDataObjects_t & result) const
	{
//OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Debug, "query");
		if (q.publisher->isCancelled())
		{
			result.clear();
			return;
		}
		// TODO Copied but not understood
		if (ocean)
			q.ocean = true;
		else
			q.mixed = true;
		//
		bbox_t b = boost::geometry::make<bbox_t>(q.left, q.top, q.right, q.bottom);
		query(b, result);
	}

	// TODO extract as algorithm and do a lazy implementation.
	// Incrementally add results
	void query(bbox_t const & b, MapDataObjects_t & result) const
	{
		using boost::geometry::intersects;
		using boost::range::for_each;
		using boost::range::copy;

		// I can't say anything
		if (!intersects(b, box))
			return;
//{
//std::ostringstream msg;
//msg << " MTB query box? " << b << " in " << box << std::endl;
//OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Debug, msg.str().c_str());
//}
		// Before using data
		((MapTreeBounds *)this)->readContent();  // TODO cast????

		// THINK: Maybe using boost::function to do only 1 call (one of them do nothing)
		for_each(bounds,
				 [&b, &result](MapTreeBounds const & node){node.query(b, result);});
		for_each(dataObjects,
				[&b, &result](MapDataObject_pointer obj)
					{
			if ((obj != nullptr) && intersects(b, obj->Box()))
				result.push_back(obj);
					});
// TODO add boxes to DataObject and propagate query to DataObjects.
////		copy(dataObjects, std::back_inserter(result));
	}

	void ContentReader(Reader_t r)
	{
		contentReader = r;
	}

	void Children(Bounds_t && nodes)
	{
		bounds = nodes;
	}
	void DataObjects(MapDataObjects_t && objs)
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
		return !bounds.empty() || !dataObjects.empty();
	}

	bbox_t box;

	// Could be an intermediate or leaf node depending on what data has.
 	Bounds_t bounds;
	MapDataObjects_t dataObjects;

	Reader_t contentReader;
};

struct MapRoot: MapTreeBounds
{
	int minZoom;
	int maxZoom;

	// TODO extract as algorithm and do a lazy implementation.
	// Incrementally add results
	void query(SearchQuery & q, MapDataObjects_t & result) const
	{
//OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Debug, "MapRoot query");
		if (minZoom <= q.zoom && maxZoom >= q.zoom)
		{
			MapTreeBounds::query(q, result);
		}
	}
};

struct MapIndex : BinaryPartIndex {

	std::vector<MapRoot> levels;

	UNORDERED(map)<int, tag_value > decodingRules;
private:
	// Mainly unused
	int nameEncodingType;
	int refEncodingType;
	int coastlineEncodingType;
	int coastlineBrokenEncodingType;
	int landEncodingType;
	int onewayAttribute;
	int onewayReverseAttribute;
	UNORDERED(set)< int > positiveLayers;
	UNORDERED(set)< int > negativeLayers;
	////

public:
	MapIndex()
	: BinaryPartIndex(MAP_INDEX),
	  nameEncodingType(-1), refEncodingType(-1),
	  coastlineEncodingType(-1), coastlineBrokenEncodingType(-1), landEncodingType(-1),
	  onewayAttribute(-1), onewayReverseAttribute(-1),
	  box(point_t(INT_MAX, INT_MAX), point_t(-1, -1))
	{}

	void finishInitializingTags() {
		int free = decodingRules.size() * 2 + 1;
		coastlineBrokenEncodingType = free++;
		initMapEncodingRule(0, coastlineBrokenEncodingType, "natural", "coastline_broken");
		if (landEncodingType == -1) {
			landEncodingType = free++;
			initMapEncodingRule(0, landEncodingType, "natural", "land");
		}
	}

	// TODO type is not used. Can we remove it on obf???
	void initMapEncodingRule(uint32_t type, uint32_t id, std::string const & tag, std::string const & val) {
		tag_value pair = tag_value(tag, val);
		decodingRules[id] = pair;

		if ("name" == tag) {
			nameEncodingType = id;
		} else if ("natural" == tag && "coastline" == val) {
			coastlineEncodingType = id;
		} else if ("natural" == tag && "land" == val) {
			landEncodingType = id;
		} else if ("oneway" == tag && "yes" == val) {
			onewayAttribute = id;
		} else if ("oneway" == tag && "-1" == val) {
			onewayReverseAttribute = id;
		} else if ("ref" == tag) {
			refEncodingType = id;
		} else if ("layer" == tag) {
			if (val != "" && val != "0") {
				if (val[0] == '-') {
					negativeLayers.insert(id);
				} else {
					positiveLayers.insert(id);
				}
			}
		}
	}

	void query(SearchQuery & q) const
	{
		using boost::geometry::intersects;
		using boost::range::for_each;

		if (q.publisher->isCancelled())
		{
			return;
		}

		// I can't say anything
		bbox_t b(point_t(q.left, q. top), point_t(q.right, q.bottom));
		if (!intersects(b, box))
			return;

//std::cerr << " MIndex query box? " << b << " in " << box << std::endl;
		MapDataObjects_t result;
		for_each(levels, [&q, &result](MapRoot const & root) { root.query(q, result); });
		q.publisher->publish(std::move(result));
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
	bbox_t box;
};

#endif /* MAPINDEX_HPP_ */
