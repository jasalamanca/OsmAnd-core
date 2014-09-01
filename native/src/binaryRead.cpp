#include "RoutingIndex.hpp"
#include "MapIndex.hpp"
#include "binaryRead.h"
#include "multipolygons.h"

#include <fcntl.h>
#include <sys/stat.h>
#include <algorithm>
#include "proto/osmand_odb.pb.h"
#include "proto/osmand_index.pb.h"
#include "proto/utils.hpp"

#include <boost/range/numeric.hpp>
#include <boost/geometry/algorithms/expand.hpp>

#include "Logging.h"

#if defined(WIN32)
#undef min
#undef max
#endif

#define DO_(EXPRESSION) if (!(EXPRESSION)) return false
using google::protobuf::io::CodedInputStream;
using google::protobuf::io::FileInputStream;
using google::protobuf::internal::WireFormatLite;

static int zoomForBaseRouteRendering  = 14;
static int detailedZoomStart = 13;
static int zoomOnlyForBasemaps  = 11;
std::map< std::string, BinaryMapFile* > openFiles;
OsmAndStoredIndex* cache = NULL;

bool readMapIndex(CodedInputStream & input, MapIndex & output,
		int fd);
bool readRoutingIndex(CodedInputStream & input, RoutingIndex & output,
		int fd);

typedef std::vector<std::string> StringTable_t;
bool readStringTable(CodedInputStream & input, StringTable_t & list)
{
	LDMessage<> inputManager(input);
	uint32_t tag;
	while ((tag = input.ReadTag()) != 0)
	{
		switch (WireFormatLite::GetTagFieldNumber(tag))
		{
		case StringTable::kSFieldNumber:
		{
			std::string s;
			WireFormatLite::ReadString(&input, &s);
			list.push_back(std::move(s));
			break;
		}
		default:
			if (!skipUnknownFields(input, tag)) {
				return false;
			}
			break;
		}
	}
	return true;
}

// Only called from renderImage on MapCreator
bool checkObjectBounds(SearchQuery* q, MapDataObject* o) {
	uint prevCross = 0;
	for (uint i = 0; i < o->points.size(); i++) {
		uint cross = 0;
		int x31 = o->points[i].first;
		int y31 = o->points[i].second;
		cross |= (x31 < q->left ? 1 : 0);
		cross |= (x31 > q->right ? 2 : 0);
		cross |= (y31 < q->top ? 4 : 0);
		cross |= (y31 > q->bottom? 8 : 0);
		if(i > 0) {
			if((prevCross & cross) == 0) {
				return true;
			}
		}
		prevCross = cross;
	}
	return false;
}

typedef UNORDERED(set)<long long> IDS_SET;

// Only called from renderImage on MapCreator
void readMapObjectsForRendering(SearchQuery * q, std::vector<MapDataObject*> & basemapResult, std::vector<MapDataObject*>& tempResult,
		std::vector<MapDataObject*>& coastLines, std::vector<MapDataObject*>& basemapCoastLines,
		int& count, bool& basemapExists, int& renderRouteDataFile, bool skipDuplicates, int& renderedState) {
	using boost::range::for_each;
	if (skipDuplicates) {
		// override it for now
		// TODO skip duplicates doesn't work correctly with basemap ?
		skipDuplicates = false;
	}
	std::map<std::string, BinaryMapFile*>::const_iterator i = openFiles.begin();
	for (; i != openFiles.end() && !q->publisher->isCancelled(); i++) {
		BinaryMapFile const * file = i->second;
		basemapExists |= file->isBasemap();
	}
	IDS_SET ids;
	i = openFiles.begin();
	for (; i != openFiles.end() && !q->publisher->isCancelled(); i++) {
		BinaryMapFile* file = i->second;
		if (q->req != NULL) {
			q->req->clearState();
		}
		q->publisher->result.clear();
		if((renderRouteDataFile == 1 || q->zoom < zoomOnlyForBasemaps) && !file->isBasemap()) {
			continue;
		} else if (!q->publisher->isCancelled()) {
			bool basemap = i->second->isBasemap();
			//readMapObjects(q, file);
			for_each(file->mapIndexes, [&q](MapIndex const & index){ index.query(*q); });
			std::vector<MapDataObject*>::const_iterator r = q->publisher->result.begin();
			tempResult.reserve((size_t) (q->publisher->result.size() + tempResult.size()));

			for (; r != q->publisher->result.end(); r++) {				
				if (skipDuplicates && (*r)->id > 0) {
					if (ids.find((*r)->id) != ids.end()) {
						continue;
					}
					ids.insert((*r)->id);
				}				
				if(basemap) {
					if(renderedState % 2 == 0 && checkObjectBounds(q, *r)) {
						renderedState |= 1;
					}
				} else {
					if(renderedState < 2 && checkObjectBounds(q, *r)) {
						renderedState |= 2;
					}
				}

				count++;
				if ((*r)->contains("natural", "coastline")) {
					if (basemap) {
						basemapCoastLines.push_back(*r);
					} else {
						coastLines.push_back(*r);
					}
				} else {
					// do not mess coastline and other types
					if (basemap) {
						basemapResult.push_back(*r);
					} else {
						tempResult.push_back(*r);
						//renderRouteDataFile = -1;
					}
				}
			}
		}
	}
}

// Only for searchNativeObjectsForRendering
void convertRouteDataObjecToMapObjects(SearchQuery* q, RouteDataObjects_t const & list,
		std::vector<MapDataObject*>& tempResult, bool skipDuplicates, IDS_SET& ids, int& renderedState) {
	RouteDataObjects_t::const_iterator rIterator = list.begin();
	tempResult.reserve((size_t) (list.size() + tempResult.size()));
	for (; rIterator != list.end(); rIterator++) {
		RouteDataObject_pointer r = (*rIterator);
		if(r == NULL) {
			continue;
		}

		if (skipDuplicates && r->id > 0) {
			if (ids.find(r->id) != ids.end()) {
				continue;
			}
			ids.insert(r->id);
		}
		MapDataObject* obj = new MapDataObject;
		bool add = true;
		std::vector<uint32_t>::const_iterator typeIt = r->types.begin();
		for (; typeIt != r->types.end(); typeIt++) {
			uint32_t k = (*typeIt);
			if (k < r->region->decodingRules.size()) {
				tag_value const & t = r->region->decodingRules[k];
				if (t.first == "highway" || t.first == "route" || t.first == "railway" || t.first == "aeroway"
						|| t.first == "aerialway") {
					obj->types.push_back(t);
				} else {
					obj->additionalTypes.push_back(t);
				}
			}
		}
		if (add) {
			for (uint32_t s = 0; s < r->pointsX.size(); s++) {
				obj->points.push_back(std::pair<int, int>(r->pointsX[s], r->pointsY[s]));
			}
			obj->id = r->id;
			UNORDERED(map)<int, std::string >::const_iterator nameIterator = r->names.begin();
			for (; nameIterator != r->names.end(); nameIterator++) {
				obj->objectNames[r->region->decodingRules[nameIterator->first].first] = nameIterator->second;
			}
			obj->area = false;
			if(renderedState < 2 && checkObjectBounds(q, obj)) {
				renderedState |= 2;
			}
			tempResult.push_back(obj);
		} else {
			delete obj;
		}
		////delete r;
	}
}

/***
void readRouteMapObjects(SearchQuery* q, BinaryMapFile* file, std::vector<RouteSubregion>& found,
		RoutingIndex* routeIndex, std::vector<MapDataObject*>& tempResult, bool skipDuplicates,
		IDS_SET& ids, int& renderedState) {
	std::sort(found.begin(), found.end(), sortRouteRegions);
//	lseek(file->fd, 0, SEEK_SET);
//	FileInputStream input(file->fd);
//	input.SetCloseOnDelete(false);
//	CodedInputStream cis(&input);
//	cis.SetTotalBytesLimit(INT_MAX, INT_MAX >> 1);
	for (std::vector<RouteSubregion>::const_iterator sub = found.begin(); sub != found.end(); sub++) {
		std::vector<RouteDataObject*> list;
//		cis.Seek(sub->filePointer + sub->mapDataBlock);
//		uint32_t length;
//		cis.ReadVarint32(&length);
//		uint32_t old = cis.PushLimit(length);
//		readRouteTreeData(&cis, &(*sub), list, routeIndex);
//		cis.PopLimit(old);
		//routeIndex->query(sub->Box(), true, list);
		routeIndex->query(sub->Box(), false, list);
		convertRouteDataObjecToMapObjects(q, list, tempResult, skipDuplicates, ids, renderedState);
	}
}
***/

// Only for searchNativeObjectsForRendering
void readRouteDataAsMapObjects(SearchQuery* q, BinaryMapFile* file, std::vector<MapDataObject*>& tempResult,
		bool skipDuplicates, IDS_SET& ids, int& renderedState) {
	std::vector<RoutingIndex*>::iterator routeIndex = file->routingIndexes.begin();
	for (; routeIndex != file->routingIndexes.end(); routeIndex++) {
		if (q->publisher->isCancelled()) {
			break;
		}
		/****
		std::vector<RouteSubregion> & subs = (*routeIndex)->subregions;
		if (q->zoom <= zoomForBaseRouteRendering) {
			subs = (*routeIndex)->basesubregions;
		}
		std::vector<RouteSubregion> found;
		lseek(file->fd, 0, SEEK_SET);
		FileInputStream input(file->fd);
		input.SetCloseOnDelete(false);
		CodedInputStream cis(&input);
		cis.SetTotalBytesLimit(INT_MAX, INT_MAX >> 1);
		searchRouteRegion(&cis, q, *routeIndex, subs, found);
////		checkAndInitRouteRegionRules(file->fd, (*routeIndex));
		***/
		//readRouteMapObjects(q, file, found, (*routeIndex), tempResult, skipDuplicates, ids, renderedState);
		bbox_t qbox(point_t(q->left, q->top), point_t(q->right, q->bottom));
		RouteDataObjects_t temp;
		(*routeIndex)->query(qbox, (q->zoom <= zoomForBaseRouteRendering), temp);
		convertRouteDataObjecToMapObjects(q, temp, tempResult, skipDuplicates, ids, renderedState);
	}
}

// Only called from renderImage on MapCreator
ResultPublisher* searchObjectsForRendering(SearchQuery * q, bool skipDuplicates, int renderRouteDataFile,
		std::string const & msgNothingFound, int& renderedState) {
	int count = 0;
	std::vector<MapDataObject*> basemapResult;
	std::vector<MapDataObject*> tempResult;
	std::vector<MapDataObject*> coastLines;
	std::vector<MapDataObject*> basemapCoastLines;

	bool basemapExists = false;
	readMapObjectsForRendering(q, basemapResult, tempResult, coastLines, basemapCoastLines, count,
			basemapExists, renderRouteDataFile, skipDuplicates, renderedState);

	bool objectsFromMapSectionRead = tempResult.size() > 0;
	bool objectsFromRoutingSectionRead = false;
	if (renderRouteDataFile >= 0 && q->zoom >= zoomOnlyForBasemaps) {
		IDS_SET ids;
		std::map<std::string, BinaryMapFile*>::const_iterator i = openFiles.begin();
		for (; i != openFiles.end() && !q->publisher->isCancelled(); i++) {
			BinaryMapFile* file = i->second;
			// false positive case when we have 2 sep maps Country-roads & Country
			if(file->mapIndexes.size() == 0 || renderRouteDataFile == 1) {
				if (q->req != NULL) {
					q->req->clearState();
				}
				q->publisher->result.clear();
				uint sz = tempResult.size();
				readRouteDataAsMapObjects(q, file, tempResult, skipDuplicates, ids, renderedState);
				objectsFromRoutingSectionRead = tempResult.size() != sz;
			}
		}
		OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Info, "Route objects %d", tempResult.size());
	}

	// sort results/ analyze coastlines and publish back to publisher
	if (q->publisher->isCancelled()) {
		deleteObjects(coastLines);
		deleteObjects(tempResult);
		deleteObjects(basemapCoastLines);
		deleteObjects(basemapResult);
	} else {
		bool ocean = q->ocean;
		bool land = q->mixed;
		bool addBasemapCoastlines = true;
		bool emptyData = q->zoom > zoomOnlyForBasemaps && tempResult.empty() && coastLines.empty();
		// determine if there are enough objects like land/lake..
		bool basemapMissing = q->zoom <= zoomOnlyForBasemaps && basemapCoastLines.empty() && !basemapExists;
		bool detailedLandData = q->zoom >= 14 && tempResult.size() > 0 && objectsFromMapSectionRead;
		if (!coastLines.empty()) {
			bool coastlinesWereAdded = processCoastlines(coastLines, q->left, q->right, q->bottom, q->top, q->zoom,
					basemapCoastLines.empty(), true, tempResult);
			addBasemapCoastlines = (!coastlinesWereAdded && !detailedLandData) || q->zoom <= zoomOnlyForBasemaps;
		} else {
			addBasemapCoastlines = !detailedLandData;
		}
		bool fillCompleteArea = false;
		if (addBasemapCoastlines) {
			bool coastlinesWereAdded = processCoastlines(basemapCoastLines, q->left, q->right, q->bottom, q->top, q->zoom,
					true, true, tempResult);
			fillCompleteArea = !coastlinesWereAdded;
		}
		// processCoastlines always create new objects
		deleteObjects(basemapCoastLines);
		deleteObjects(coastLines);
		OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Info,"Land %d ocean %d fillCompleteArea %d", 
							land, ocean, fillCompleteArea);
		if (fillCompleteArea) {
			MapDataObject* o = new MapDataObject();
			o->points.push_back(int_pair(q->left, q->top));
			o->points.push_back(int_pair(q->right, q->top));
			o->points.push_back(int_pair(q->right, q->bottom));
			o->points.push_back(int_pair(q->left, q->bottom));
			o->points.push_back(int_pair(q->left, q->top));
			if (ocean && !land) {
				o->types.push_back(tag_value("natural", "coastline"));
			} else {
				o->types.push_back(tag_value("natural", "land"));
			}
			o->area = true;
			o->additionalTypes.push_back(tag_value("layer", "-5"));
			tempResult.push_back(o);
		}
		if (emptyData || basemapMissing) {
			// message
			// avoid overflow int errors
			MapDataObject* o = new MapDataObject();
			o->points.push_back(int_pair(q->left + (q->right - q->left) / 2, q->top + (q->bottom - q->top) / 2));
			o->types.push_back(tag_value("natural", "coastline"));
			o->objectNames["name"] = msgNothingFound;
			tempResult.push_back(o);
		}
		if (q->zoom <= zoomOnlyForBasemaps || emptyData || (objectsFromRoutingSectionRead && q->zoom < detailedZoomStart)) {
			tempResult.insert(tempResult.end(), basemapResult.begin(), basemapResult.end());
		} else {
			deleteObjects(basemapResult);
		}
		q->publisher->result.clear();
		q->publisher->publish(tempResult);
		OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Info,
				"Search : tree - read( %d), accept( %d), objs - visit( %d), accept(%d), in result(%d) ",
				q->numberOfReadSubtrees, q->numberOfAcceptedSubtrees, q->numberOfVisitedObjects, q->numberOfAcceptedObjects,
				q->publisher->result.size());
	}
	return q->publisher;
}

/****
void searchRouteRegion(CodedInputStream * input, SearchQuery const * q, RoutingIndex * ind,
		std::vector<RouteSubregion> & subregions, std::vector<RouteSubregion>& toLoad) {
	for (std::vector<RouteSubregion>::iterator subreg = subregions.begin();
						subreg != subregions.end(); subreg++) {
		if (subreg->right >= q->left && q->right >= subreg->left &&
				subreg->bottom >= q->top && q->bottom >= subreg->top) {
			if(subreg->subregions.empty() && subreg->mapDataBlock == 0){
				input->Seek(subreg->filePointer);
				uint32_t old = input->PushLimit(subreg->length);
				readRouteTree(input, &(*subreg), NULL, ind, -1, false);///
				input->PopLimit(old);
			}
			searchRouteRegion(input, q, ind, subreg->subregions, toLoad);
			if(subreg->mapDataBlock != 0) {
				toLoad.push_back(*subreg);
			}
		}
	}
}
****/

////////////////////////////////
///// End MapIndex


bool closeBinaryMapFile(std::string const & inputName) {
	std::map<std::string, BinaryMapFile*>::iterator iterator;
	if ((iterator = openFiles.find(inputName)) != openFiles.end()) {
		delete iterator->second;
		openFiles.erase(iterator);
		return true;
	}
	return false;
}

bool initMapFilesFromCache(std::string const & inputName) {
	GOOGLE_PROTOBUF_VERIFY_VERSION;
#if defined(_WIN32)
	int fileDescriptor = open(inputName.c_str(), O_RDONLY | O_BINARY);
#else
	int fileDescriptor = open(inputName.c_str(), O_RDONLY);
#endif
	if (fileDescriptor < 0) {
		OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Error, "Cache file could not be open to read : %s", inputName.c_str());
		return false;
	}
	FileInputStream input(fileDescriptor);
	CodedInputStream cis(&input);
	cis.SetTotalBytesLimit(INT_MAX, INT_MAX >> 1);
	OsmAndStoredIndex* c = new OsmAndStoredIndex();
	if(c->MergeFromCodedStream(&cis)){
		OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Info, "Native Cache file initialized %s", inputName.c_str());
		cache = c;
		for (int i = 0; i < cache->fileindex_size(); i++) {
			FileIndex fi = cache->fileindex(i);
		}
		return true;
	}
	return false;
}

bool hasEnding (std::string const &fullString, std::string const &ending)
{
    if (fullString.length() >= ending.length()) {
        return (0 == fullString.compare (fullString.length() - ending.length(), ending.length(), ending));
    } else {
        return false;
    }
}

bool initMapStructure(CodedInputStream & input, BinaryMapFile & file)
{
	uint32_t tag;
	uint32_t versionConfirm = -2;
	while ((tag = input.ReadTag()) != 0)
	{
		switch (WireFormatLite::GetTagFieldNumber(tag))
		{
		// required uint32_t version = 1;
		case OsmAndStructure::kVersionFieldNumber:
			readUInt32(input, file.version);
			break;
		case OsmAndStructure::kDateCreatedFieldNumber:
			readUInt64(input, file.dateCreated);
			break;
		case OsmAndStructure::kMapIndexFieldNumber:
		{
			MapIndex mapIndex;
			readMapIndex(input, mapIndex, file.fd);//, false);
			file.mapIndexes.push_back(std::move(mapIndex));
			file.basemap = file.basemap || mapIndex.name.find("basemap") != std::string::npos;
			break;
		}
		case OsmAndStructure::kRoutingIndexFieldNumber:
		{
			RoutingIndex* routingIndex = new RoutingIndex;
			readRoutingIndex(input, *routingIndex, file.fd);
			file.routingIndexes.push_back(routingIndex);
			break;
		}
		case OsmAndStructure::kVersionConfirmFieldNumber:
			readUInt32(input, versionConfirm);
			break;
		default:
			if (!skipUnknownFields(input, tag)) {
				return false;
			}
			break;
		}
	}  // end of while
	if (file.version != versionConfirm) {
		OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Error, "Corrupted file. It should be ended as it starts with version %d %d", file.version, versionConfirm);
		return false;
	}
	if (file.version != MAP_VERSION) {
		OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Error, "Version of the file is not supported.");
		return false;
	}
	return true;
}

BinaryMapFile* initBinaryMapFile(std::string const & inputName) {
	GOOGLE_PROTOBUF_VERIFY_VERSION;
	closeBinaryMapFile(inputName);

#if defined(_WIN32)
	int fileDescriptor = open(inputName.c_str(), O_RDONLY | O_BINARY);
	int routeDescriptor = open(inputName.c_str(), O_RDONLY | O_BINARY);
#else
	int fileDescriptor = open(inputName.c_str(), O_RDONLY);
	int routeDescriptor = open(inputName.c_str(), O_RDONLY);
#endif
	if (fileDescriptor < 0 || routeDescriptor < 0 || routeDescriptor == fileDescriptor) {
		OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Error, "File could not be open to read from C : %s", inputName.c_str());
		return NULL;
	}
	BinaryMapFile* mapFile = new BinaryMapFile();
	mapFile->fd = fileDescriptor;

	mapFile->routefd = routeDescriptor;
	FileIndex* fo = NULL;
	if (cache != NULL) {
		struct stat stat;
		fstat(fileDescriptor, &stat);
		for (int i = 0; i < cache->fileindex_size(); i++) {
			FileIndex const & fi = cache->fileindex(i);
			if (hasEnding(inputName, fi.filename()) && fi.size() == stat.st_size) {
				fo = cache->mutable_fileindex(i);
				break;
			}
		}
	}
	if (fo != NULL)
	{  // Previously cached
		mapFile->version = fo->version();
		mapFile->dateCreated = fo->datemodified();
		for (int i = 0; i < fo->mapindex_size(); i++) {
			MapIndex mi;
			MapPart const & mp = fo->mapindex(i);
			////mi.filePointer = mp.offset();
			////mi.length = mp.size();
			mi.name = mp.name();
			for (int j = 0; j < mp.levels_size(); j++) {
				MapLevel const & ml = mp.levels(j);
				MapRoot mr;
				mr.bottom = ml.bottom();
				mr.left = ml.left();
				mr.right = ml.right();
				mr.top = ml.top();
				mr.maxZoom = ml.maxzoom();
				mr.minZoom = ml.minzoom();
				mr.filePointer = ml.offset();
				mr.length = ml.size();
				mi.levels.push_back(mr);
			}
			mapFile->basemap = mapFile->basemap || mi.name.find("basemap") != std::string::npos;
			mapFile->mapIndexes.push_back(mi);
		}

		for (int i = 0; i < fo->routingindex_size(); i++) {
			RoutingIndex *mi = new RoutingIndex();
			RoutingPart const & mp = fo->routingindex(i);
			////mi->filePointer = mp.offset();
			////mi->length = mp.size();
			mi->name = mp.name();
			for (int j = 0; j < mp.subregions_size(); j++) {
				RoutingSubregion const & ml = mp.subregions(j);
				RouteSubregion mr(mi);
				mr.bottom = ml.bottom();
				mr.left = ml.left();
				mr.right = ml.right();
				mr.top = ml.top();
				////mr.mapDataBlock = ml.shiftodata();
				mr.filePointer = ml.offset();
				////mr.length = ml.size();
				if (ml.basemap()) {
					mi->basesubregions.push_back(mr);
				} else {
					mi->subregions.push_back(mr);
				}
			}
			mapFile->routingIndexes.push_back(mi);
		}
		OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Debug, "Native file initialized from cache %s", inputName.c_str());
	} else {
		FileInputStream input(fileDescriptor);
		input.SetCloseOnDelete(false);
		CodedInputStream cis(&input);
		cis.SetTotalBytesLimit(INT_MAX, INT_MAX >> 1);

		if (!initMapStructure(cis, *mapFile)) {
			OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Error, "File not initialised : %s", inputName.c_str());
			delete mapFile;
			return NULL;
		}
	}
	mapFile->inputName = inputName;
	openFiles.insert(std::pair<std::string, BinaryMapFile*>(inputName, mapFile));
	return mapFile;
}

///////////////
//// Global access
void MapQuery(SearchQuery & q/*, MapDataObjects_t & output*/)
{
	using boost::range::for_each;
	for_each(openFiles, [&q/*, &output*/](std::pair<std::string, BinaryMapFile *> const & fp)
			{
		BinaryMapFile const * file = fp.second;
		for_each(file->mapIndexes, [&q/*, &output*/](MapIndex const & index){index.query(q);});
			});
}

void RoutingQuery(bbox_t & b, RouteDataObjects_t & output)
{
	// FIXME To avoid typical errors between subRegion read coordinates and what would really be.
	// Expand 30 unit around real box.
	// Previous code loaded tile by tile. That reduced the impact of that bug but the bug remains.
	// Correct solution implies to calculate boxes bottom-up and update, extending incorrect ones,
	// which conflicts with lazy reading of map files.
	b = boost::geometry::make<bbox_t>(b.min_corner().x()-30, b.min_corner().y()-30,
			b.max_corner().x()+30, b.max_corner().y()+30);

	using boost::range::for_each;
	for_each(openFiles, [&b, &output](std::pair<std::string, BinaryMapFile *> const & fp)
			{
		BinaryMapFile const * file = fp.second;
		// NO basemap
		for_each(file->routingIndexes, [&b, &output](RoutingIndex const * index){index->query(b, false, output);});
			});
//std::cerr << "RoutingQuery #RDO " << output.size() << std::endl;
}

void RoutingQuery(SearchQuery & q, RouteDataObjects_t & output)
{
	bbox_t b(point_t(q.left, q.top), point_t(q.right, q.bottom));
	RoutingQuery(b, output);
}
