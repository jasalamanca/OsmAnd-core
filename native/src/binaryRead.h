#ifndef _OSMAND_BINARY_READ_H
#define _OSMAND_BINARY_READ_H

#if defined(_WIN32)
#include <io.h>
#else
#include <unistd.h>
#endif
#include <map>
#include <vector>
#include <string>
#include "Common.h"
#include "common2.h"

#if defined(WIN32)
#define close _close
#endif

#include "mapObjects.h"
#include "renderRules.h"

////
class RoutingIndex;
class RouteSubregion;
class RouteDataObject;




static const uint MAP_VERSION = 2;


struct MapTreeBounds {
	uint32_t length;
	uint32_t filePointer;
	uint32_t mapDataBlock;
	uint32_t left ;
	uint32_t right ;
	uint32_t top ;
	uint32_t bottom;
	bool ocean;

	MapTreeBounds() {
		ocean = -1;
	}
};

struct MapRoot: MapTreeBounds {
	uint minZoom ;
	uint maxZoom ;
	std::vector<MapTreeBounds> bounds;
};

enum PART_INDEXES {
	MAP_INDEX = 1,
	POI_INDEX,
	ADDRESS_INDEX,
	TRANSPORT_INDEX,
	ROUTING_INDEX,
};

struct BinaryPartIndex {
	uint32_t length;
	int filePointer;
	PART_INDEXES type;
	std::string name;

	BinaryPartIndex(PART_INDEXES tp) : type(tp) {}
};

struct MapIndex : BinaryPartIndex {

	std::vector<MapRoot> levels;

	UNORDERED(map)<int, tag_value > decodingRules;
	int nameEncodingType;
	int refEncodingType;
	int coastlineEncodingType;
	int coastlineBrokenEncodingType;
	int landEncodingType;
	int onewayAttribute ;
	int onewayReverseAttribute ;
	UNORDERED(set)< int > positiveLayers;
	UNORDERED(set)< int > negativeLayers;

	MapIndex() : BinaryPartIndex(MAP_INDEX) {
		nameEncodingType = refEncodingType = coastlineBrokenEncodingType = coastlineEncodingType = -1;
		landEncodingType = onewayAttribute = onewayReverseAttribute = -1;
	}

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
};

struct BinaryMapFile {
	std::string inputName;
	uint32_t version;
	uint64_t dateCreated;
	std::vector<MapIndex> mapIndexes;
	std::vector<RoutingIndex*> routingIndexes;
////	std::vector<BinaryPartIndex*> indexes;
	int fd;
	int routefd;
	bool basemap;

	bool isBasemap() const {
		return basemap;
	}

	~BinaryMapFile() {
		close(fd);
		close(routefd);
	}
};

struct ResultPublisher {
	std::vector< MapDataObject*> result;

	bool publish(MapDataObject* r) {
		result.push_back(r);
		return true;
	}
	bool publish(std::vector<MapDataObject*> r) {
		result.insert(result.begin(), r.begin(), r.end());
		return true;
	}
	bool isCancelled() const {
		return false;
	}
	virtual ~ResultPublisher() {
		deleteObjects(result);
	}
};

struct SearchQuery {
	RenderingRuleSearchRequest* req;
	uint32_t left;
	uint32_t right;
	uint32_t top;
	uint32_t bottom;
	int zoom;
	ResultPublisher* publisher;

	bool ocean;
	bool mixed;

	uint numberOfVisitedObjects;
	uint numberOfAcceptedObjects;
	uint numberOfReadSubtrees;
	uint numberOfAcceptedSubtrees;

	SearchQuery(int l, int r, int t, int b, RenderingRuleSearchRequest* req, ResultPublisher* publisher) :
			req(req), left(l), right(r), top(t), bottom(b),publisher(publisher) {
		numberOfAcceptedObjects = numberOfVisitedObjects = 0;
		numberOfAcceptedSubtrees = numberOfReadSubtrees = 0;
		ocean = mixed = false;
	}
	SearchQuery(int l, int r, int t, int b) :
				left(l), right(r), top(t), bottom(b) {
	}

	SearchQuery(){

	}

	bool publish(MapDataObject* obj) {
		return publisher->publish(obj);
	}
};

// Public interface to file maps.
void searchRouteSubregions(SearchQuery const * q, std::vector<RouteSubregion>& tempResult, bool basemap);

void searchRouteDataForSubRegion(SearchQuery const * q, std::vector<RouteDataObject*>& list, RouteSubregion const * sub);

ResultPublisher* searchObjectsForRendering(SearchQuery* q, bool skipDuplicates, int renderRouteDataFile, std::string const & msgNothingFound, int& renderedState);

BinaryMapFile* initBinaryMapFile(std::string const & inputName);

bool initMapFilesFromCache(std::string const & inputName) ;

bool closeBinaryMapFile(std::string const & inputName);

#endif
