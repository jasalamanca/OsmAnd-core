#ifndef _OSMAND_BINARY_READ_H
#define _OSMAND_BINARY_READ_H

#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#if defined(_WIN32)
#include <io.h>
#else
#include <unistd.h>
#endif
#include <stdio.h>
#include <fstream>
#include <map>
#include <string>
#include <stdint.h>
#include "mapObjects.h"
#include "multipolygons.h"
#include "Common.h"
#include "common2.h"

#if defined(WIN32)
#define close _close
#endif

#include "mapObjects.h"
#include "renderRules.h"

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
struct RoutingIndex;
struct RouteSubregion {
	uint32_t length;
	uint32_t filePointer;
	uint32_t mapDataBlock;
	uint32_t left;
	uint32_t right;
	uint32_t top;
	uint32_t bottom;
	std::vector<RouteSubregion> subregions;
	RoutingIndex* routingIndex;

	RouteSubregion(RoutingIndex* ind) : length(0), filePointer(0), mapDataBlock(0), routingIndex(ind){
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

struct RoutingIndex : BinaryPartIndex {
//	UNORDERED(map)< uint32_t, tag_value > decodingRules;
	vector< tag_value > decodingRules;
	std::vector<RouteSubregion> subregions;
	std::vector<RouteSubregion> basesubregions;
	RoutingIndex() : BinaryPartIndex(ROUTING_INDEX) {
	}

	void initRouteEncodingRule(uint32_t id, std::string tag, std::string val) {
		tag_value pair = tag_value(tag, val);
		// DEFINE hash
		//encodingRules[pair] = id;
		while(decodingRules.size() < id + 1){
			decodingRules.push_back(pair);
		}
		decodingRules[id] = pair;
	}
};

struct RouteDataObject {
	RoutingIndex* region;
	std::vector<uint32_t> types ;
	std::vector<uint32_t> pointsX ;
	std::vector<uint32_t> pointsY ;
	std::vector<uint64_t> restrictions ;
	std::vector<std::vector<uint32_t> > pointTypes;
	int64_t id;

	UNORDERED(map)<int, std::string > names;
	vector<pair<uint32_t, uint32_t> > namesIds;

	string getName() {
		if(names.size() > 0) {
			return names.begin()->second;
		}
		return "";
	}

	inline int64_t getId() {
		return id;
	}

	int getSize() {
		int s = sizeof(this);
		s += pointsX.capacity()*sizeof(uint32_t);
		s += pointsY.capacity()*sizeof(uint32_t);
		s += types.capacity()*sizeof(uint32_t);
		s += restrictions.capacity()*sizeof(uint64_t);
		std::vector<std::vector<uint32_t> >::iterator t = pointTypes.begin();
		for(;t!=pointTypes.end(); t++) {
			s+= (*t).capacity() * sizeof(uint32_t);
		}
		s += namesIds.capacity()*sizeof(pair<uint32_t, uint32_t>);
		s += names.size()*sizeof(pair<int, string>)*10;
		return s;
	}

	inline int getPointsLength() {
		return pointsX.size();
	}

	bool loop(){
		return pointsX[0] == pointsX[pointsX.size() - 1] && pointsY[0] == pointsY[pointsY.size() - 1] ; 
	}
	
	bool roundabout(){
		uint sz = types.size();
		for(uint i=0; i < sz; i++) {
			tag_value r = region->decodingRules[types[i]];
			if(r.first == "roundabout" || r.second == "roundabout") {
				return true;
			} else if(r.first == "oneway" && r.second != "no" && loop()) {
				return true;
			}
		}
		return false;
	}

	double directionRoute(int startPoint, bool plus){
		// look at comment JAVA
		return directionRoute(startPoint, plus, 5);
	}

	// Gives route direction of EAST degrees from NORTH ]-PI, PI]
	double directionRoute(size_t startPoint, bool plus, float dist) {
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

	static double parseSpeed(string v, double def) {
		if(v == "none") {
			return 40;// RouteDataObject::NONE_MAX_SPEED;
		} else {
			int i = findFirstNumberEndIndex(v);
			if (i > 0) {
				double f = atof(v.substr(0, i).c_str());
				f /= 3.6; // km/h -> m/s
				if (v.find("mph") != string::npos) {
					f *= 1.6;
				}
				return f;
			}
		}
		return def;
	}
	
	static double parseLength(string v, double def) {
		// 14"10' not supported
		int i = findFirstNumberEndIndex(v);
		if (i > 0) {
			double f = atof(v.substr(0, i).c_str());
			if (v.find("\"") != string::npos  || v.find("ft") != string::npos) {
				// foot to meters
				f *= 0.3048;
			}
			return f;
		}
		return def;
	}
	
	static double parseWeightInTon(string v, double def) {
		int i = findFirstNumberEndIndex(v);
		if (i > 0) {
			double f = atof(v.substr(0, i).c_str());
			if (v.find("\"") != string::npos || v.find("lbs") != string::npos) {
				// lbs -> kg -> ton
				f = (f * 0.4535) / 1000.0;
			}
			return f;
		}
		return def;
	}
};



struct MapIndex : BinaryPartIndex {

	std::vector<MapRoot> levels;

	UNORDERED(map)<int, tag_value > decodingRules;
	// DEFINE hash
	//UNORDERED(map)<tag_value, int> encodingRules;

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

	void initMapEncodingRule(uint32_t type, uint32_t id, std::string tag, std::string val) {
		tag_value pair = tag_value(tag, val);
		// DEFINE hash
		//encodingRules[pair] = id;
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
	std::vector<BinaryPartIndex*> indexes;
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
	bool isCancelled() {
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

	coordinates cacheCoordinates;
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

ResultPublisher* searchObjectsForRendering(SearchQuery* q, bool skipDuplicates, int renderRouteDataFile, std::string msgNothingFound, int& renderedState);

BinaryMapFile* initBinaryMapFile(std::string const & inputName);

bool initMapFilesFromCache(std::string const & inputName) ;

bool closeBinaryMapFile(std::string const & inputName);

#endif
