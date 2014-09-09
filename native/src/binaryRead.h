#ifndef _OSMAND_BINARY_READ_H
#define _OSMAND_BINARY_READ_H

#if defined(_WIN32)
#include <io.h>
#else
#include <unistd.h>
#endif
#include <vector>
#include <string>

#if defined(WIN32)
#define close _close
#endif

#include "renderRules.h"

////
class RoutingIndex;
class RouteSubregion;
class RouteDataObject;
////
class MapIndex;
class MapTreeBounds;
class MapRoot;
class MapDataObject;

struct ResultPublisher {
	std::vector< MapDataObject*> result;

	bool publish(MapDataObject* r) {
		result.push_back(r);
		return true;
	}
	bool publish(std::vector<MapDataObject*> const & r) {
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

void deleteObjects(std::vector <MapDataObject* > & v);

static const uint MAP_VERSION = 2;

enum PART_INDEXES {
	MAP_INDEX = 1,
	POI_INDEX,
	ADDRESS_INDEX,
	TRANSPORT_INDEX,
	ROUTING_INDEX,
};

struct BinaryPartIndex {
	uint32_t length; // TODO Used to pass dataobjects to java side. We need another form to do that.
	int filePointer; // TODO Used to pass dataobjects to java side. We need another form to do that.
////	PART_INDEXES type;
	std::string name;

	BinaryPartIndex(PART_INDEXES tp) {}////: type(tp) {}
};

////
#include "MapIndex.hpp"
////

struct BinaryMapFile {
	std::string inputName;
	uint32_t version;
	uint64_t dateCreated;
	std::vector<MapIndex> mapIndexes;
	std::vector<RoutingIndex*> routingIndexes;
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

// Public interface to file maps.
void searchRouteSubregions(SearchQuery const * q, std::vector<RouteSubregion>& tempResult, bool basemap);

ResultPublisher* searchObjectsForRendering(SearchQuery* q, bool skipDuplicates, int renderRouteDataFile, std::string const & msgNothingFound, int& renderedState);

BinaryMapFile* initBinaryMapFile(std::string const & inputName);
bool initMapFilesFromCache(std::string const & inputName) ;
bool closeBinaryMapFile(std::string const & inputName);

size_t RoutingMemorySize();

#endif
