#ifndef _OSMAND_BINARY_ROUTE_PLANNER_H
#define _OSMAND_BINARY_ROUTE_PLANNER_H

#include "Common.h"
#include "common2.h"
#include "binaryRead.h"

// Implementation
#include <algorithm>
#include <iostream>
#include "Logging.h"
#include "generalRouter.h"

typedef UNORDERED(map)<std::string, float> MAP_STR_FLOAT;
typedef UNORDERED(map)<std::string, std::string> MAP_STR_STR;

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

struct RoutingSubregionTile {
	RouteSubregion subregion;
	// make it without get/set for fast access
	int access;
	int loaded;
	int size;
	UNORDERED(map)<int64_t, SHARED_PTR<RouteSegment> > routes;

	RoutingSubregionTile(RouteSubregion const & sub)
	: subregion(sub), access(0), loaded(0) {
		size = sizeof(RoutingSubregionTile);
	}
	~RoutingSubregionTile(){
	}
	bool isLoaded() const {
		return loaded > 0;
	}

	void setLoaded(){
		loaded = abs(loaded) + 1;
	}

	void unload(){
		routes.clear();
		size = 0;
		loaded = - abs(loaded);
	}

	int getUnloadCount() {
		return abs(loaded);
	}

	size_t getSize(){
		return size + routes.size() * sizeof(std::pair<int64_t, SHARED_PTR<RouteSegment> >);
	}

	void add(SHARED_PTR<RouteDataObject> o) {
		size += o->getSize() + sizeof(RouteSegment)* o->pointsX.size();
		for (int i = o->pointsX.size()-1; i >= 0; --i) {
			uint64_t x31 = o->pointsX[i];
			uint64_t y31 = o->pointsY[i];
			uint64_t l = (((uint64_t) x31) << 31) + (uint64_t) y31;
			SHARED_PTR<RouteSegment> segment =  SHARED_PTR<RouteSegment>(new RouteSegment(o, i));
			if (routes[l] == NULL) {
				routes[l] = segment;
			} else {
				SHARED_PTR<RouteSegment> orig = routes[l];
				while (orig->next != NULL) {
					orig = orig->next;
				}
				orig->next = segment;
			}
		}
	}
};

static inline int64_t calcRouteId(SHARED_PTR<RouteDataObject const> const & o, int ind) {
	return ((int64_t) o->id << 10) + ind;
}

typedef std::pair<int, std::pair<std::string, std::string> > ROUTE_TRIPLE;
struct RoutingConfiguration {
	GeneralRouter router;

	int memoryLimitation;
	float initialDirection;

	int zoomToLoad;
	float heurCoefficient;
	int planRoadDirection;
////	std::string routerName;

	void initParams(MAP_STR_STR& attributes) {
		planRoadDirection = (int) parseFloat(attributes, "planRoadDirection", 0);
		heurCoefficient = parseFloat(attributes, "heuristicCoefficient", 1);
		// don't use file limitations?
		memoryLimitation = (int)parseFloat(attributes, "nativeMemoryLimitInMB", memoryLimitation);
		zoomToLoad = (int)parseFloat(attributes, "zoomToLoadTiles", 16);
		// routerName = parseString(attributes, "name", "default");
		// routerProfile = parseString(attributes, "baseProfile", "car");
	}

	RoutingConfiguration(float initDirection = -360, int memLimit = 64) :
			memoryLimitation(memLimit), initialDirection(initDirection) {
	}
};

bool compareRoutingSubregionTile(SHARED_PTR<RoutingSubregionTile> const & o1, SHARED_PTR<RoutingSubregionTile> const & o2);

class RouteCalculationProgress {
protected:
	int segmentNotFound ;
	float distanceFromBegin;
	int directSegmentQueueSize;
	float distanceFromEnd;
	int reverseSegmentQueueSize;

	bool cancelled;
public:
	RouteCalculationProgress() : segmentNotFound(-1), distanceFromBegin(0),
		directSegmentQueueSize(0), distanceFromEnd(0), reverseSegmentQueueSize(0), cancelled(false){
	}

	virtual ~RouteCalculationProgress(){};

	virtual bool isCancelled() const {
		return cancelled;
	}

	virtual void setSegmentNotFound(int s){
		segmentNotFound = s;
	}

	virtual void updateStatus(float distanceFromBegin,	int directSegmentQueueSize,	float distanceFromEnd,
			int reverseSegmentQueueSize) {
		this->distanceFromBegin = std::max(distanceFromBegin, this->distanceFromBegin );
		this->distanceFromEnd = std::max(distanceFromEnd,this->distanceFromEnd);
		this->directSegmentQueueSize = directSegmentQueueSize;
		this->reverseSegmentQueueSize = reverseSegmentQueueSize;
	}
};

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

	float getDeviationDistance(int x31, int y31, int ind);
	float getDeviationDistance(int x31, int y31);
	int getIndex(int x31, int y31);
	float timeEstimate(int begX, int begY, int endX, int endY);
};

void RoutingQuery(bbox_t & b, RouteDataObjects_t & output);
struct RoutingContext {
	typedef UNORDERED(map)<int64_t, SHARED_PTR<RoutingSubregionTile> > MAP_SUBREGION_TILES;
	typedef UNORDERED(map)<int64_t, std::vector<SHARED_PTR<RoutingSubregionTile> > > MAP_INDEXED_SUBREGIONS;

	int visitedSegments;
	int loadedTiles;
	OsmAnd::ElapsedTimer timeToLoad;
	OsmAnd::ElapsedTimer timeToCalculate;
////	int firstRoadDirection;
////	int64_t firstRoadId;
	RoutingConfiguration & config;
	SHARED_PTR<RouteCalculationProgress> progress;

	int gcCollectIterations;

	int startX;
	int startY;
	int targetX;
	int targetY;
	bool basemap;

	PrecalculatedRouteDirection precalcRoute;
	SHARED_PTR<FinalRouteSegment> finalRouteSegment;
	MAP_SUBREGION_TILES subregionTiles;
	MAP_INDEXED_SUBREGIONS indexedSubregions;

private:
	// To manage modified roads.
	std::vector<SHARED_PTR<RouteDataObject> > registered;
	// To memo connections between roads.
	UNORDERED(map)<int64_t, SHARED_PTR<RouteSegment> > connections;
	size_t connections_size;
	UNORDERED(set)<int64_t> loadedRDO;

	void add(SHARED_PTR<RouteDataObject> & o, int x, int y)
	{
		connections_size += o->getSize() + sizeof(RouteSegment)* o->pointsX.size();
		for (int i = o->pointsX.size()-1; i >= 0; --i)
		{
			uint32_t x31 = o->pointsX[i];
			uint32_t y31 = o->pointsY[i];
			if (x31 != x || y31 != y) continue;
			int64_t l = ((int64_t)x31 << 31) + y31;
			SHARED_PTR<RouteSegment> segment = SHARED_PTR<RouteSegment>(new RouteSegment(o, i));
			if (connections.count(l) != 0)
				segment->next = connections[l];
			connections[l] = segment;
		}
	}

	void add(SHARED_PTR<RouteDataObject> & o, bbox_t b)
	{
		connections_size += o->getSize() + sizeof(RouteSegment)* o->pointsX.size();
		for (int i = o->pointsX.size()-1; i >= 0; --i)
		{
			uint32_t x31 = o->pointsX[i];
			uint32_t y31 = o->pointsY[i];
			if (!boost::geometry::covered_by(point_t(x31, y31), b)) continue;
			int64_t l = ((int64_t)x31 << 31) + y31;
			SHARED_PTR<RouteSegment> segment = SHARED_PTR<RouteSegment>(new RouteSegment(o, i));
			if (connections.count(l) != 0)
				segment->next = connections[l];
			connections[l] = segment;
		}
	}

public:
	RoutingContext(RoutingConfiguration& config)
		: visitedSegments(0), loadedTiles(0),
////		  firstRoadDirection(0), firstRoadId(0),
		  config(config), finalRouteSegment()
		  , connections_size(0)
	{
		precalcRoute.empty = true;
	}

	bool acceptLine(SHARED_PTR<RouteDataObject> const & r) {
		return config.router.acceptLine(r);
	}

	int getSize() const {
		// multiply 2 for to maps
		int sz = subregionTiles.size() * sizeof(std::pair< int64_t, SHARED_PTR<RoutingSubregionTile> >)  * 2;
		MAP_SUBREGION_TILES::const_iterator it = subregionTiles.begin();
		for(;it != subregionTiles.end(); it++) {
			sz += it->second->getSize();
		}
		sz += connections_size;
		return sz;
	}

	void unloadUnusedTiles(int memoryLimit) {
		int sz = getSize();
		float critical = 0.9f * memoryLimit * 1024 * 1024;
		if(sz < critical) {
			return;
		}
		float occupiedBefore = sz / (1024. * 1024.);
		float desirableSize = memoryLimit * 0.7f * 1024 * 1024;
		std::vector<SHARED_PTR<RoutingSubregionTile> > list;
		MAP_SUBREGION_TILES::iterator it = subregionTiles.begin();
		int loaded = 0;
		int unloadedTiles = 0;
		for(;it != subregionTiles.end(); it++) {
			if(it->second->isLoaded()) {
				list.push_back(it->second);
				loaded++;
			}
		}
		sort(list.begin(), list.end(), compareRoutingSubregionTile);
		size_t i = 0;
		while(sz >= desirableSize && i < list.size()) {
			SHARED_PTR<RoutingSubregionTile> unload = list[i];
			i++;
			sz -= unload->getSize();
			unload->unload();
			unloadedTiles ++;
		}
		for (i = list.size()-1; i >= 0; --i) {
			list[i]->access /= 3;
		}
		OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Info, "Run GC (before %f Mb after %f Mb) unload %d of %d tiles",
				occupiedBefore, getSize() / (1024.0*1024.0),
				unloadedTiles, loaded);
	}

	void loadHeaderObjects(std::vector<SHARED_PTR<RoutingSubregionTile> > & subregions)
	{
		bool gc = false;
		for(size_t j = 0; j<subregions.size() && !gc; j++) {
			if(!subregions[j]->isLoaded()) {
				gc = true;
			}
		}
		if(gc) {
			unloadUnusedTiles(config.memoryLimitation);
		}
		for(size_t j = 0; j<subregions.size(); j++) {
			if(!subregions[j]->isLoaded()) {
				loadedTiles++;
				subregions[j]->setLoaded();
				SearchQuery q;
				std::vector<RouteDataObject*> res;
				searchRouteDataForSubRegion(&q, res, subregions[j]->subregion);
//std::cerr << "loadHO #RDO " << res.size() << std::endl;
				std::vector<RouteDataObject*>::const_iterator i = res.begin();
				for(;i!=res.end(); i++) {
					if(*i != NULL) {
						////SHARED_PTR<RouteDataObject> o(*i);
						SHARED_PTR<RouteDataObject> o(new RouteDataObject(**i));
						if(acceptLine(o)) {
							subregions[j]->add(o);
						}
					}
				}
			}
		}
	}

	void loadHeaders(uint32_t xloc, uint32_t yloc) {
//std::cerr << "loadH x = " << xloc << " y = " << yloc << std::endl;
		timeToLoad.Start();
		////int z  = config.zoomToLoad;
		////int tz = 31 - z;
		////int64_t tileId = (xloc << z) + yloc;
		int64_t tileId = 0; // All together
		//if (indexedSubregions.count(tileId) == 0) {
			SearchQuery q(xloc, xloc, yloc, yloc);// l,r,t,b
			std::vector<RouteSubregion> tempResult;
			searchRouteSubregions(&q, tempResult, basemap);
			std::vector<SHARED_PTR<RoutingSubregionTile> > collection;
			for(size_t i=0; i<tempResult.size(); i++) {
				RouteSubregion& rs = tempResult[i];
				int64_t key = ((int64_t)rs.left << 31)+ rs.filePointer;// Now unsafe
				if(subregionTiles.find(key) == subregionTiles.end()) {
					subregionTiles[key] = SHARED_PTR<RoutingSubregionTile>(new RoutingSubregionTile(rs));
					collection.push_back(subregionTiles[key]);// Only new subregions
				}
				//collection.push_back(subregionTiles[key]);
			}
			//indexedSubregions[tileId] = collection;
			indexedSubregions[tileId].insert(indexedSubregions[tileId].end(), collection.begin(), collection.end());
		//}
		loadHeaderObjects(indexedSubregions[tileId]);
		timeToLoad.Pause();
	}

	void reregisterRouteDataObject(SHARED_PTR<RouteDataObject> o, int segmentStart, uint32_t x, uint32_t y);

	void loadTileData(int x31, int y31, int zoomAround, std::vector<SHARED_PTR<RouteDataObject> > & dataObjects) {
		UNORDERED(set)<int64_t> ids;
		// First search on new road. Newer the better.
		for (int i = registered.size()-1; i >= 0; --i)
		{
			int64_t id = registered[i]->id;
			if (ids.count(id) == 0)
			{
				ids.insert(id);
				dataObjects.push_back(registered[i]);
			}
		}

		// Second search on map info.
		/***
		int t = config.zoomToLoad - zoomAround;
		int coordinatesShift = (1 << (31 - config.zoomToLoad));
		if(t <= 0) {
			t = 1;
			coordinatesShift = (1 << (31 - zoomAround));
		} else {
			t = 1 << t;
		}
		int z  = config.zoomToLoad;
		for(int i = -t; i <= t; i++) {
			for(int j = -t; j <= t; j++) {
				uint32_t xloc = (x31 + i*coordinatesShift) >> (31 - z);
				uint32_t yloc = (y31+j*coordinatesShift) >> (31 - z);
				int64_t tileId = (xloc << z) + yloc;
				loadHeaders(xloc, yloc); ***/
		loadHeaders(x31, y31);
		int64_t tileId = 0; // All together
				std::vector<SHARED_PTR<RoutingSubregionTile> >& subregions = indexedSubregions[tileId];
std::cerr << "loadTD NAT #subReg " << subregions.size() << std::endl;
				for(size_t j = 0; j<subregions.size(); j++) {
					if(subregions[j]->isLoaded()) {
						UNORDERED(map)<int64_t, SHARED_PTR<RouteSegment> >::const_iterator s = subregions[j]->routes.begin();
						while(s != subregions[j]->routes.end()) {
							SHARED_PTR<RouteSegment> seg = s->second;
							while(seg != NULL) {
								if (ids.count(seg->road->id) == 0) {
									ids.insert(seg->road->id);
									dataObjects.push_back(seg->road);
								}
								seg = seg->next;
							}
							s++;
						}
					}
				}
/****			}
		}
		****/
std::cerr << "loadTD NAT #RDO " << dataObjects.size() << std::endl;
	}

private:
	// Only to trace
	int cuenta(SHARED_PTR<RouteSegment> kk) const
	{
		int count = 0;
	while (kk != NULL)
	{
		count++;
		kk = kk->next;
	}
	return count;
	}
public:

	SHARED_PTR<RouteSegment> loadRouteSegment(uint32_t x31, uint32_t y31)
	{
//		OsmAnd::ElapsedTimer timer;
//		timer.Start();
		UNORDERED(set)<int64_t> excludeDuplications;
		SHARED_PTR<RouteSegment> original;
//std::cerr << "NAT loadRS(" << x31 << ", " << y31 << ") #registered=" << registered.size() << std::endl;
		// First search on new roads. Newer is better.
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

#define MEMO
#ifndef MEMO
		// Second search on map info.
		RouteDataObjects_t objects;
		bbox_t b(point_t(x31, y31), point_t(x31, y31));
		timeToLoad.Start();
		RoutingQuery(b, objects);
		timeToLoad.Pause();
		for (int i = objects.size()-1; i >= 0; --i)
		{
			if (objects[i] != NULL)
			{
				SHARED_PTR<RouteDataObject> ro(new RouteDataObject(*(objects[i])));////
				if (acceptLine(ro))
				{
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
			}
		}
#else
		// Second search on map info.
		//// Memo pattern
		int64_t key = ((int64_t)x31 << 31) + y31;
		if (connections.count(key) == 0)
		{
			RouteDataObjects_t objects;
			bbox_t b(point_t(x31, y31), point_t(x31, y31));
			timeToLoad.Start();
			RoutingQuery(b, objects);
			timeToLoad.Pause();
			for (int i = objects.size()-1; i >= 0; --i)
			{
				if (objects[i] != NULL)
				{
					SHARED_PTR<RouteDataObject> ro(new RouteDataObject(*(objects[i])));////
					if (acceptLine(ro))
						add(ro, x31, y31);
				}
			}
		}
		SHARED_PTR<RouteSegment> segment = connections[key];
if (segment == NULL) std::cerr << "loadSegment(" << x31 << ',' << y31 << ")=NULL" << std::endl;
		while (segment != NULL)
		{
			SHARED_PTR<RouteDataObject> const & ro = segment->road;
			int sStart = segment->segmentStart;
			int64_t roId = calcRouteId(ro, sStart);
			if (excludeDuplications.count(roId) == 0) {
				excludeDuplications.insert(roId);
				SHARED_PTR<RouteSegment> s = SHARED_PTR<RouteSegment>(new RouteSegment(ro, sStart));
				s->next = original;
				original = 	s;
			}
			segment = segment->next;
		}
#endif
////std::cerr << "loadRS original->id " << original->road->id << std::endl;
//		timer.Pause();
//		std::cerr << "NAT loadRS time=" << timer.GetElapsedMs() << std::endl;
		return original;
	}

	void loadRoad(SHARED_PTR<RouteDataObject> const & roadToLoad)
	{
		// Load data on node map.
		if (loadedRDO.count(roadToLoad->id) == 0)
		{
			loadedRDO.insert(roadToLoad->id);
		RouteDataObjects_t objects;
		bbox_t b = roadToLoad->Box();
		timeToLoad.Start();
		RoutingQuery(b, objects);
		timeToLoad.Pause();
		for (int i = objects.size()-1; i >= 0; --i)
		{
			if (objects[i] != NULL)
			{
				SHARED_PTR<RouteDataObject> ro(new RouteDataObject(*(objects[i])));////
				if (acceptLine(ro))
				{
					add(ro, b);
				}
			}
		}
		}
	}

	bool isInterrupted() const {
		return false;
	}
	float getHeuristicCoefficient() const {
		return config.heurCoefficient;
	}

	bool planRouteIn2Directions() const {
		return getPlanRoadDirection() == 0;
	}
	int getPlanRoadDirection() const {
		return config.planRoadDirection;
	}

	// Register modified route data objects with a live time equal to that of context.
	void registerRouteDataObject(SHARED_PTR<RouteDataObject> const & o)
	{
		registered.push_back(SHARED_PTR<RouteDataObject>(o));
	}
};


std::vector<RouteSegmentResult> searchRouteInternal(RoutingContext* ctx, bool leftSideNavigation);
#endif /*_OSMAND_BINARY_ROUTE_PLANNER_H*/
