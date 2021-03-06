/*
 * binaryRoutingIndexRead.cpp
 *
 *  Created on: 25/06/2014
 *      Author: javier
 */

#include "RoutingIndex.hpp"
#include "binaryRead.h"

#include "proto/osmand_odb.pb.h"
#include "proto/utils.hpp"

////
// EXTERNAL
extern std::map< std::string, BinaryMapFile* > openFiles;
typedef std::vector<std::string> StringTable_t;
bool readStringTable(CodedInputStream & input, StringTable_t & list);

//////////////////////////
//// RoutingIndex

const static int RESTRICTION_SHIFT = 3;
const static int RESTRICTION_MASK = 7;
static const int ROUTE_SHIFT_COORDINATES = 4;

using google::protobuf::io::CodedInputStream;
using google::protobuf::io::FileInputStream;
using google::protobuf::internal::WireFormatLite;

void searchRouteDataForSubRegion(SearchQuery const * q, RouteDataObjects_t & list,
		RouteSubregion const & sub)
{
	std::map<std::string, BinaryMapFile*>::const_iterator i = openFiles.begin();
	RoutingIndex const * rs = sub.routingIndex;
	for (; i != openFiles.end() && !q->publisher->isCancelled(); i++) {
		BinaryMapFile* file = i->second;
		for (std::vector<RoutingIndex*>::iterator routingIndex = file->routingIndexes.begin();
				routingIndex != file->routingIndexes.end(); routingIndex++) {
			if (q->publisher->isCancelled()) {
				break;
			}
			if (rs != NULL && (rs->name != (*routingIndex)->name)){
				continue;
			}
			////searchRouteSubRegion(file->routefd, list, (*routingIndex), sub);
			// TODO I don't know if base map is requested.
			//routingIndex->query(sub->Box(), true, list);
			(*routingIndex)->querySub(sub.Box(), false, list);
//OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Error, "dataObject #list %d", list.size());
			return;
		}
	}
}

bool readRouteEncodingRule(CodedInputStream & input, RoutingIndex & index, uint32_t id)
{
	LDMessage<> inputManager(input);
	int tag;
	std::string tagS;
	std::string value;
	while ((tag = input.ReadTag()) != 0)
	{
		switch (WireFormatLite::GetTagFieldNumber(tag))
		{
		case OsmAndRoutingIndex_RouteEncodingRule::kValueFieldNumber:
			WireFormatLite::ReadString(&input, &value);
			break;
		case OsmAndRoutingIndex_RouteEncodingRule::kTagFieldNumber:
			WireFormatLite::ReadString(&input, &tagS);
			break;
		case OsmAndRoutingIndex_RouteEncodingRule::kIdFieldNumber:
			readUInt32(input, id);
			break;
		default:
			if (!skipUnknownFields(input, tag)) {
				return false;
			}
			break;
		}
	}  // end of while
	// Add rule to index
	index.initRouteEncodingRule(id, tagS, value);
	return true;
}

////
void updatePointTypes(std::vector<std::vector<uint32_t> > & pointTypes, std::vector<size_t> const & skipped)
{
	for (size_t i = 0; i < skipped.size(); ++i)
	{
//		OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Error, "Mirando skipped[%d]=%d de %d", i, skipped[i], pointTypes.size());
		size_t index = skipped[i];

		if (index >= pointTypes.size()) return; // No more points have type info.

		std::vector<uint32_t> & accumulate = pointTypes[index-1];
		std::vector<uint32_t> & drop = pointTypes[index];
		// Accumulate types of both points.
		accumulate.insert(accumulate.end(), drop.begin(), drop.end());
		// Delete drop info (index+1 position)
		pointTypes.erase(pointTypes.begin()+index);
	}
}

template <typename SEQ>
bool readRouteTypes(CodedInputStream & input, SEQ & output)
{
	LDMessage<> inputManager(input);
	uint32_t t;
	while (input.BytesUntilLimit() > 0) {
		readUInt32(input, t);
		output.push_back(t);
	}
	return true;
}

bool readRoutePoints(CodedInputStream & input, RouteDataObject * output,
		RouteSubregion const & context, std::vector<size_t> & skipped)
{
	LDMessage<> inputManager(input);
	int px = context.Box().min_corner().x() >> ROUTE_SHIFT_COORDINATES;
	int py = context.Box().min_corner().y() >> ROUTE_SHIFT_COORDINATES;
	bbox_t box = output->Box();
	output->pointsX.reserve(5);
	output->pointsY.reserve(5);
	while (input.BytesUntilLimit() > 0)
	{
		int deltaX, deltaY;
		readSint32(input, deltaX);
		readSint32(input, deltaY);
		if (deltaX == 0 && deltaY == 0 && !output->pointsX.empty())
		{
			skipped.push_back(output->pointsX.size());
			continue;
		}

		uint32_t x = deltaX + px;
		uint32_t y = deltaY + py;
		output->pointsX.push_back(x << ROUTE_SHIFT_COORDINATES);
		output->pointsY.push_back(y << ROUTE_SHIFT_COORDINATES);
		boost::geometry::expand(box, point_t(x << ROUTE_SHIFT_COORDINATES, y << ROUTE_SHIFT_COORDINATES));
		px = x;
		py = y;
	}
	output->Box(box);
	return true;
}

bool readRouteNames(CodedInputStream & input, RouteDataObject * output)
{
	LDMessage<> inputManager(input);
	output->namesIds.reserve(3);
	uint32_t s;
	uint32_t t;
	while (input.BytesUntilLimit() > 0)
	{
		readUInt32(input, s);
		readUInt32(input, t);
		output->namesIds.push_back(std::make_pair(s, t));
	}
	return true;
}

bool readRoutePTypes(CodedInputStream & input, RouteDataObject * output)
{
	LDMessage<> inputManager(input);
	while (input.BytesUntilLimit() > 0)
	{
		uint32_t pointInd;
		readUInt32(input, pointInd);
		if (output->pointTypes.size() <= pointInd) {
			output->pointTypes.resize(pointInd + 1, std::vector<uint32_t>());
		}
		readRouteTypes(input, output->pointTypes[pointInd]);
	}
	return true;
}

bool readRouteDataObject(CodedInputStream & input, RouteDataObject * output,
		RouteSubregion const & context)
{
//std::cerr << "BEGIN readRDO pos " << input.TotalBytesRead() << " RDO dir " << output << std::endl;
	LDMessage<> inputManager(input);
	std::vector<size_t> skipped;
	int tag;
	while ((tag = input.ReadTag()) != 0)
	{
		switch (WireFormatLite::GetTagFieldNumber(tag))
		{
		case RouteData::kTypesFieldNumber:
			readRouteTypes(input, output->types);
			break;
		case RouteData::kRouteIdFieldNumber:
			readInt64(input, output->id);
			break;
		case RouteData::kPointsFieldNumber:
			readRoutePoints(input, output, context, skipped);
			break;
		case RouteData::kStringNamesFieldNumber:
			readRouteNames(input, output);
			break;
		case RouteData::kPointTypesFieldNumber:
			readRoutePTypes(input, output);
			break;
		default:
			if (!skipUnknownFields(input, tag)) {
				return false;
			}
			break;
		}
	} // end while
	updatePointTypes(output->pointTypes, skipped);
//std::cerr << "readRDO types #" << output->types.size() << std::endl;
//std::cerr << "readRDO id " << output->id << std::endl;
//std::cerr << "readRDO points #" << output->pointsX.size() << std::endl;
//std::cerr << "readRDO namesIds #" << output->namesIds.size() << std::endl;
//if (output->pointTypes.size() > 0)
//	std::cerr << "readRDO Ptypes #" << output->pointTypes.size() << " when read #points " << output->pointsX.size() << std::endl;
	return true;
}

typedef UNORDERED(map)<int64_t, std::vector<uint64_t> > Restrictions_t;
// Reads the restriction for a polyline.
bool readRestrictions(CodedInputStream & input, Restrictions_t & output)
{
	LDMessage<> inputManager(input);
	uint64_t from = 0;
	uint64_t to = 0;
	uint64_t type = 0;
	int tm;
	int ts;
	while ((ts = input.ReadTag()) != 0)
	{
		switch (WireFormatLite::GetTagFieldNumber(ts))
		{
		case RestrictionData::kFromFieldNumber:
			readInt32(input, tm);
			from = tm;
			break;
		case RestrictionData::kToFieldNumber:
			readInt32(input, tm);
			to = tm;
			break;
		case RestrictionData::kTypeFieldNumber:
			readInt32(input, tm);
			type = tm;
			break;
		default:
			if (!skipUnknownFields(input, ts)) {
				return false;
			}
			break;
		}
	}  // end of while
//OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Error, "restriction[%d] += (%d, %d) from,to,type", from, to, type);
	output[from].push_back((to << RESTRICTION_SHIFT) + type);
	return true;
}

#define NODE_CAPACITY 40

typedef std::vector<int64_t> IdTable_t;
// Reads the table of ids.
bool readIdTable(CodedInputStream & input, IdTable_t & output)
{
	LDMessage<> inputManager(input);
	output.reserve(NODE_CAPACITY);
	int64_t routeId = 0;
	int tag;
	while ((tag = input.ReadTag()) != 0)
	{
		switch (WireFormatLite::GetTagFieldNumber(tag))
		{
		// Delta encoded ids
		case IdTable::kRouteIdFieldNumber:
			int64_t val;
			readSint64(input, val);
			routeId += val;
////OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Error, "idTables[%d] = %d", output.size(), routeId);
			output.push_back(routeId);
			break;
		default:
			if (!skipUnknownFields(input, tag)) {
				return false;
			}
			break;
		}
	} // end of while
	return true;
}

bool readRouteTreeBase(CodedInputStream & input, RouteSubregion & output,
		RouteSubregion const * parentTree,	RoutingIndex * ind, int fd);

// Reads children
bool readRouteTreeNodes(CodedInputStream & input, RouteSubregion & output, RoutingIndex * ind, int fd)
{
//OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Error, "readRouteTreeNodes from %d", input.TotalBytesRead());
	LDMessage<OSMAND_FIXED32> inputManager(input);
	output.subregions.reserve(NODE_CAPACITY);
	uint32_t tag;
	while ((tag = input.ReadTag()) != 0)
	{
		switch (WireFormatLite::GetTagFieldNumber(tag))
		{
		case OsmAndRoutingIndex_RouteDataBox::kBoxesFieldNumber:
				{
				RouteSubregion subregion(ind);
				readRouteTreeBase(input, subregion, &output, ind, fd);
				output.subregions.push_back(std::move(subregion));
				}
			break;
		default:
			if (!skipUnknownFields(input, tag)) {
				return false;
			}
			break;
		}
	}  // End of while
	return true;
}

// Reads DataObjects for this leaf node.
bool readRouteTreeData(CodedInputStream & input, RouteSubregion & output, RoutingIndex* routingIndex)
{
//OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Error, "readRouteTreeData from %d", input.TotalBytesRead());
	LDMessage<> inputManager(input);
	RouteDataObjects_t dataObjects;
	dataObjects.reserve(NODE_CAPACITY);
	IdTable_t idTables;
	Restrictions_t restrictions;
	std::vector<std::string> stringTable;
	int tag;
	while ((tag = input.ReadTag()) != 0)
	{
		switch (WireFormatLite::GetTagFieldNumber(tag))
		{
		case OsmAndRoutingIndex_RouteDataBlock::kDataObjectsFieldNumber:
		{
			RouteDataObject_pointer obj = RouteDataObject_pointer(new RouteDataObject);
			// map reading routines will do a safe use of base pointer.
			readRouteDataObject(input, obj.get(), output);
			if (dataObjects.size() <= obj->id ) {
				dataObjects.resize(obj->id + 1, NULL);//normally dataobject come ordered resize???
//OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Error, "id %d cap %d", obj->id, dataObjects.capacity());
			}
			obj->region = routingIndex;
			dataObjects[obj->id] = obj;
			}
			break;
		case OsmAndRoutingIndex_RouteDataBlock::kStringTableFieldNumber:
			readStringTable(input, stringTable);
			break;
		case OsmAndRoutingIndex_RouteDataBlock::kRestrictionsFieldNumber:
			readRestrictions(input, restrictions);
			break;
		case OsmAndRoutingIndex_RouteDataBlock::kIdTableFieldNumber:
			readIdTable(input, idTables);
			break;
		default:
			if (!skipUnknownFields(input, tag)) {
				return false;
			}
			break;
		}
	}  // end of while
	// Moving restrictions to DataObjects
	Restrictions_t::const_iterator itRestrictions = restrictions.begin();
	for (; itRestrictions != restrictions.end(); itRestrictions++) {
		RouteDataObject_pointer & fromr = dataObjects[itRestrictions->first]; // avoid using []
		if (fromr != NULL) {
			fromr->restrictions = itRestrictions->second;
			for (size_t i = 0; i < fromr->restrictions.size(); i++) {
				uint32_t to = fromr->restrictions[i] >> RESTRICTION_SHIFT;
				uint64_t valto = (idTables[to] << RESTRICTION_SHIFT) | ((long) fromr->restrictions[i] & RESTRICTION_MASK);
				fromr->restrictions[i] = valto;
			}
		}
	}
	RouteDataObjects_t::const_iterator dobj = dataObjects.begin();
	for (; dobj != dataObjects.end(); dobj++) {
		if (*dobj != NULL) {
			if ((*dobj)->id < idTables.size()) {
				(*dobj)->id = idTables[(*dobj)->id];
			}
			std::vector<std::pair<uint32_t, uint32_t> >::const_iterator itnames = (*dobj)->namesIds.begin();
			for(; itnames != (*dobj)->namesIds.end(); itnames++) {
				if((*itnames).second >= stringTable.size()) {
					OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Error, "ERROR VALUE string table %d", (*itnames).second );
				} else {
					(*dobj)->names[(int) (*itnames).first] = stringTable[(*itnames).second];
				}
			}
		}
	}

	output.DataObjects(std::move(dataObjects));

	return true;
}

// Reads the minimal data for a RTree node and prepare to read the rest.
bool readRouteTreeBase(CodedInputStream & input, RouteSubregion & output,
		RouteSubregion const * parentTree,	RoutingIndex * ind, int fd)
{
	uint32_t lPos = input.TotalBytesRead();
	LDMessage<OSMAND_FIXED32> inputManager(input);
	uint32_t mPos = input.TotalBytesRead();
	uint32_t objectsOffset = 0; // Will be an intermediate node but if has ShiftToData
//OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Error, "New RouteSubregion filepos %d", lPos);

	// Start reading message fields
	uint32_t tag;
	int i = 0;
	while ((tag = input.ReadTag()) != 0)
	{
		switch (WireFormatLite::GetTagFieldNumber(tag))
		{
		// Delta encoded box coordinates
		case OsmAndRoutingIndex_RouteDataBox::kLeftFieldNumber:
			readSint32(input, i);
			output.left = i + (parentTree != NULL ? parentTree->left : 0);
			break;
		case OsmAndRoutingIndex_RouteDataBox::kRightFieldNumber:
			readSint32(input, i);
			output.right = i + (parentTree != NULL ? parentTree->right : 0);
			break;
		case OsmAndRoutingIndex_RouteDataBox::kTopFieldNumber:
			readSint32(input, i);
			output.top = i + (parentTree != NULL ? parentTree->top : 0);
			break;
		case OsmAndRoutingIndex_RouteDataBox::kBottomFieldNumber:
			readSint32(input, i);
			output.bottom = i + (parentTree != NULL ? parentTree->bottom : 0);
			break;

		// Is a leaf.
		case OsmAndRoutingIndex_RouteDataBox::kShiftToDataFieldNumber:
			readInt(input, objectsOffset);
//OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Error, "Leaf node. Data offset %d", objectsOffset);
			break;

		default:
			if (!skipUnknownFields(input, tag)) {
				return false;
			}
			break;
		}
	}  // End of while

	output.Box(bbox_t(point_t(output.left, output.top), point_t(output.right, output.bottom)));
//std::cerr << "Box read " << output.Box() << std::endl;
	if (objectsOffset == 0)
	{  // An intermediate node.
		output.ContentReader([lPos, ind, fd](RouteSubregion & output)
				{
			int filed = dup(fd); // Duplicated fd to permit parallel access.
			lseek(filed, 0, SEEK_SET); // need to start from begining
			FileInputStream is(filed);
			is.SetCloseOnDelete(true);
			CodedInputStream input(&is);
			input.SetTotalBytesLimit(INT_MAX, INT_MAX >> 1);
			input.Seek(lPos); // Needed to correctelly set TotalBytesRead
			readRouteTreeNodes(input, output, ind, fd);
				});
	}
	else
	{  // A leaf node.
		uint32_t pos = mPos+objectsOffset;
		output.ContentReader([pos, ind, fd](RouteSubregion & output)
				{
			int filed = dup(fd); // Duplicated fd to permit parallel access.
			lseek(filed, 0, SEEK_SET); // need to start from begining
			FileInputStream is(filed);
			is.SetCloseOnDelete(true);
			CodedInputStream input(&is);
			input.SetTotalBytesLimit(INT_MAX, INT_MAX >> 1);
			input.Seek(pos); // Needed to correctelly set TotalBytesRead
			readRouteTreeData(input, output, ind);
				});
	}
	return true;
}

bool readRoutingIndex(CodedInputStream & input, RoutingIndex & output, int fd)
{
//OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Error, "new RoutingIndex pos %d", input.TotalBytesRead());
    // TODO Remove filePointer and length ASAP
	LDMessage<OSMAND_FIXED32> inputManager(input);
	output.subregions.reserve(2);
	output.basesubregions.reserve(2);
    output.filePointer = input.TotalBytesRead();
	output.length = input.BytesUntilLimit();
	uint32_t defaultId = 1;
	uint32_t tag;
	while ((tag = input.ReadTag()) != 0)
	{
		switch (WireFormatLite::GetTagFieldNumber(tag))
		{
		case OsmAndRoutingIndex::kNameFieldNumber:
			WireFormatLite::ReadString(&input, &output.name);
			break;
		case OsmAndRoutingIndex::kRulesFieldNumber:
			readRouteEncodingRule(input, output, defaultId++);
			break ;
		case OsmAndRoutingIndex::kRootBoxesFieldNumber:
		case OsmAndRoutingIndex::kBasemapBoxesFieldNumber:
		{
			bool basemap = WireFormatLite::GetTagFieldNumber(tag) == OsmAndRoutingIndex::kBasemapBoxesFieldNumber;
			RouteSubregion subregion(&output);
			readRouteTreeBase(input, subregion, NULL, &output, fd);
			if(basemap) {
//std::cerr << "RSR is basemap" << std::endl;
				output.basesubregions.push_back(std::move(subregion));
			} else {
				output.subregions.push_back(std::move(subregion));
			}
			break;
		}
		case OsmAndRoutingIndex::kBlocksFieldNumber:
			// Finish reading
			input.Skip(input.BytesUntilLimit());
			break;
		default:
			if (!skipUnknownFields(input, tag)) {
				return false;
			}
			break;
		}
	}  // end of while

	// Calculate bounding box.
	bbox_t box(output.Box());
	// My accumulate
	auto op = [&box](RouteSubregion const & r){boost::geometry::expand(box, r.Box());};
	boost::range::for_each(output.basesubregions, op);
	boost::range::for_each(output.subregions, op);
	output.Box(box);
//std::cout << "Box " << output.Box() << std::endl;
//OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Error, "readRI #decR %d, #subR %d, #basesubR %d",
//		output.decodingRules.size(), output.subregions.size(), output.basesubregions.size());
	return true;
}

//// Fin RoutingIndex
////////////////////////

////////////////
// AUX for code dependencies
////////////////////
bool RouteDataObject::roundabout()
{
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
