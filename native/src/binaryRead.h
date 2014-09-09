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

////
class RoutingIndex;
class RouteSubregion;
class RouteDataObject;
////
class MapIndex;
class MapTreeBounds;
class MapRoot;
class MapDataObject;

void deleteObjects(std::vector <MapDataObject* > & v);

static const uint MAP_VERSION = 2;

#include "MapIndex.hpp"
#include "RoutingIndex.hpp"

struct BinaryMapFile {
	std::string inputName;
	uint32_t version;
	uint64_t dateCreated;
	// FIXME Introduced pointer for MapIndex because the object scope needed along lazy reading.
	// Same thing was with RoutingIndex.
	// Now we allocate dinamically both kinds of indexes to have correct scope.
	// They are needed (basically) to access to index rules when reading types. Can we change this behavior??
	std::vector<MapIndex *> mapIndexes;
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
		using boost::range::for_each;
		for_each(mapIndexes, [](MapIndex * p){ delete p; });
		for_each(routingIndexes, [](RoutingIndex * p){ delete p; });
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
