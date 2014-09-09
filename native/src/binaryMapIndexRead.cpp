/*
 * binaryMapIndexRead.cpp
 *
 *  Created on: 30/06/2014
 *      Author: javier
 */

#include "MapIndex.hpp"

#include "proto/osmand_odb.pb.h"
#include "proto/utils.hpp"

////
// EXTERNAL
typedef std::vector<std::string> StringTable_t;
bool readStringTable(CodedInputStream & input, StringTable_t & list);


///////////////////////////////
//// MapIndex

/****
// Only called from renderImage on MapCreator
MapDataObject* readMapDataObject(CodedInputStream* input, MapTreeBounds const * tree, SearchQuery* req,
			MapIndex const * root) {
	uint32_t tag = WireFormatLite::GetTagFieldNumber(input->ReadTag());
	bool area = MapData::kAreaCoordinatesFieldNumber == tag;
	if(!area && MapData::kCoordinatesFieldNumber != tag) {
		return NULL;
	}

	MapDataObject* dataObject = new MapDataObject();
	dataObject->area = area;
	coordinates & coor = dataObject->points;
	uint32_t size;
	input->ReadVarint32(&size);
	int old = input->PushLimit(size);
	int px = tree->left & MASK_TO_READ;
	int py = tree->top & MASK_TO_READ;
	bool contains = false;
	int minX = INT_MAXIMUM;
	int maxX = 0;
	int minY = INT_MAXIMUM;
	int maxY = 0;
	req->numberOfVisitedObjects++;
	int x;
	int y;
	while (input->BytesUntilLimit() > 0) {
		if (!WireFormatLite::ReadPrimitive<int32_t, WireFormatLite::TYPE_SINT32>(input, &x)) {
			return NULL;
		}
		if (!WireFormatLite::ReadPrimitive<int32_t, WireFormatLite::TYPE_SINT32>(input, &y)) {
			return NULL;
		}
		x = (x << SHIFT_COORDINATES) + px;
		y = (y << SHIFT_COORDINATES) + py;
		coor.push_back(std::pair<int, int>(x, y));
		px = x;
		py = y;
		if (!contains && req->left <= x && req->right >= x && req->top <= y && req->bottom >= y) {
			contains = true;
		}
		if (!contains) {
			minX = std::min(minX, x);
			maxX = std::max(maxX, x);
			minY = std::min(minY, y);
			maxY = std::max(maxY, y);
		}
	}
	if (!contains) {
		if (maxX >= req->left && minX <= req->right && minY <= req->bottom && maxY >= req->top) {
			contains = true;
		}
	}
	input->PopLimit(old);
	if (!contains) {
		return NULL;
	}

	// READ types
	std::vector< coordinates > & innercoordinates = dataObject->polygonInnerCoordinates;
	std::vector< tag_value > & additionalTypes = dataObject->additionalTypes;
	std::vector< tag_value > & types = dataObject->types;
	UNORDERED(map)< std::string, unsigned int> & stringIds = dataObject->stringIds;
	int64_t id = 0;
	bool loop = true;
	while (loop) {
		uint32_t t = input->ReadTag();
		switch (WireFormatLite::GetTagFieldNumber(t)) {
		case 0:
			loop = false;
			break;
		case MapData::kPolygonInnerCoordinatesFieldNumber: {
			coordinates polygon;

			px = tree->left & MASK_TO_READ;
			py = tree->top & MASK_TO_READ;
			input->ReadVarint32(&size);
			old = input->PushLimit(size);
			while (input->BytesUntilLimit() > 0) {
				WireFormatLite::ReadPrimitive<int32_t, WireFormatLite::TYPE_SINT32>(input, &x);
				WireFormatLite::ReadPrimitive<int32_t, WireFormatLite::TYPE_SINT32>(input, &y);
				x = (x << SHIFT_COORDINATES) + px;
				y = (y << SHIFT_COORDINATES) + py;
				polygon.push_back(std::pair<int, int>(x, y));
				px = x;
				py = y;
			}
			input->PopLimit(old);
			innercoordinates.push_back(polygon);
			break;
		}
		case MapData::kAdditionalTypesFieldNumber: {
			input->ReadVarint32(&size);
			old = input->PushLimit(size);
			while (input->BytesUntilLimit() > 0) {
				WireFormatLite::ReadPrimitive<int32_t, WireFormatLite::TYPE_INT32>(input, &x);
				UNORDERED(map)<int, tag_value>::const_iterator dR = root->decodingRules.find(x);
				if (dR != root->decodingRules.end()) {
					tag_value const & t = dR->second;
					additionalTypes.push_back(t);
				}
			}
			input->PopLimit(old);
			break;
		}
		case MapData::kTypesFieldNumber: {
			input->ReadVarint32(&size);
			old = input->PushLimit(size);
			while (input->BytesUntilLimit() > 0) {
				WireFormatLite::ReadPrimitive<int32_t, WireFormatLite::TYPE_INT32>(input, &x);
				UNORDERED(map)<int, tag_value>::const_iterator dR = root->decodingRules.find(x);
				if (dR != root->decodingRules.end()) {
					tag_value const & t = dR->second;
					types.push_back(t);
				}
			}
			input->PopLimit(old);
			bool acceptTps = acceptTypes(req, types, root);
			if (!acceptTps) {
				return NULL;
			}
			break;
		}
		case MapData::kIdFieldNumber:
			WireFormatLite::ReadPrimitive<int64_t, WireFormatLite::TYPE_SINT64>(input, &id);
			dataObject->id = id;
			break;
		case MapData::kStringNamesFieldNumber:
			input->ReadVarint32(&size);
			old = input->PushLimit(size);
			while (input->BytesUntilLimit() > 0) {
				WireFormatLite::ReadPrimitive<int32_t, WireFormatLite::TYPE_INT32>(input, &x);
				WireFormatLite::ReadPrimitive<int32_t, WireFormatLite::TYPE_INT32>(input, &y);
				UNORDERED(map)<int, tag_value>::const_iterator dR = root->decodingRules.find(x);
				if (dR != root->decodingRules.end()) {
					tag_value const & t = dR->second;
					stringIds[t.first] = y;
				}
			}
			input->PopLimit(old);
			break;
		default: {
			if (WireFormatLite::GetTagWireType(t) == WireFormatLite::WIRETYPE_END_GROUP) {
				return NULL;
			}
			if (!skipUnknownFields(input, t)) {
				return NULL;
			}
			break;
		}
		}
	}

	req->numberOfAcceptedObjects++;

	return dataObject;
}
***/

/****
bool readMapTreeBounds(CodedInputStream* input, MapTreeBounds* tree, MapRoot const * root) {
	int tag;
	int32_t si;
	while ((tag = input->ReadTag()) != 0) {
		switch (WireFormatLite::GetTagFieldNumber(tag)) {
		case OsmAndMapIndex_MapDataBox::kLeftFieldNumber: {
			DO_((WireFormatLite::ReadPrimitive<int32_t, WireFormatLite::TYPE_SINT32>(input, &si)));
			tree->left = si + root->left;
			break;
		}
		case OsmAndMapIndex_MapDataBox::kRightFieldNumber: {
			DO_((WireFormatLite::ReadPrimitive<int32_t, WireFormatLite::TYPE_SINT32>(input, &si)));
			tree->right = si + root->right;
			break;
		}
		case OsmAndMapIndex_MapDataBox::kTopFieldNumber: {
			DO_((WireFormatLite::ReadPrimitive<int32_t, WireFormatLite::TYPE_SINT32>(input, &si)));
			tree->top = si + root->top;
			break;
		}
		case OsmAndMapIndex_MapDataBox::kBottomFieldNumber: {
			DO_((WireFormatLite::ReadPrimitive<int32_t, WireFormatLite::TYPE_SINT32>(input, &si)));
			tree->bottom = si + root->bottom;
			break;
		}
		case OsmAndMapIndex_MapDataBox::kOceanFieldNumber: {
			DO_((WireFormatLite::ReadPrimitive<bool, WireFormatLite::TYPE_BOOL>(input, &tree->ocean)));
			break;
		}
		default: {
			if (WireFormatLite::GetTagWireType(tag) == WireFormatLite::WIRETYPE_END_GROUP) {
				return true;
			}
			if (!skipUnknownFields(input, tag)) {
				return false;
			}
			break;
		}
		}
	}
	return true;
}
***/

/****
 * HERE WAS PUBLISHING MAPDATA TO REQUEST
bool readMapDataBlocks(CodedInputStream* input, SearchQuery * req, MapTreeBounds const * tree, MapIndex const * root) {
	uint64_t baseId = 0;
	int tag;
	std::vector< MapDataObject* > results;
	while ((tag = input->ReadTag()) != 0) {
		if (req->publisher->isCancelled()) {
			return false;
		}
		switch (WireFormatLite::GetTagFieldNumber(tag)) {
		case MapDataBlock::kBaseIdFieldNumber : {
			WireFormatLite::ReadPrimitive<uint64_t, WireFormatLite::TYPE_UINT64>(input, &baseId);
			break;
		}
		case MapDataBlock::kStringTableFieldNumber: {
////			uint32_t length;
////			DO_((WireFormatLite::ReadPrimitive<uint32_t, WireFormatLite::TYPE_UINT32>(input, &length)));
////			int oldLimit = input->PushLimit(length);
			if(results.size() > 0) {
				std::vector<std::string> stringTable;
				readStringTable(*input, stringTable);
				for (std::vector<MapDataObject*>::const_iterator obj = results.begin(); obj != results.end(); obj++) {
					if ((*obj)->stringIds.size() > 0) {
						UNORDERED(map)<std::string, unsigned int >::const_iterator  val=(*obj)->stringIds.begin();
						while(val != (*obj)->stringIds.end()){
							(*obj)->objectNames[val->first]=stringTable[val->second];
							val++;
						}
					}
				}
			}
		else
		{
			skipUnknownFields(input, tag);
		}
////			input->Skip(input->BytesUntilLimit());
////			input->PopLimit(oldLimit);
			break;
		}
		case MapDataBlock::kDataObjectsFieldNumber: {
			uint32_t length;
			DO_((WireFormatLite::ReadPrimitive<uint32_t, WireFormatLite::TYPE_UINT32>(input, &length)));
			int oldLimit = input->PushLimit(length);
			MapDataObject* mapObject = readMapDataObject(input, tree, req, root);
			if (mapObject != NULL) {
				mapObject->id += baseId;
				req->publish(mapObject);
				results.push_back(mapObject);
			}
			input->Skip(input->BytesUntilLimit());
			input->PopLimit(oldLimit);
			break;
		}
		default: {
			if (WireFormatLite::GetTagWireType(tag) == WireFormatLite::WIRETYPE_END_GROUP) {
				return true;
			}
			if (!skipUnknownFields(input, tag)) {
				return false;
			}
			break;
		}
		}
	}
	return true;
}
***/

typedef UNORDERED(map)<std::string, unsigned int> stringIds_t;
bool readStrIds(CodedInputStream & input, stringIds_t & output,
		MapIndex const & root)
{
	LDMessage<> inputManager(input);
	int i, j;
	while (input.BytesUntilLimit() > 0)
	{
		readInt32(input, i);
		readInt32(input, j);
		// TODO: What about doing that in future when needed??????
		UNORDERED(map)<int, tag_value>::const_iterator dR = root.decodingRules.find(i);
		if (dR != root.decodingRules.end()) {
			output[dR->second.first] = j;
		}
	}
	return true;
}

typedef std::vector<tag_value> Types_t;
bool readTypes(CodedInputStream & input, Types_t & output,
		MapIndex const & root)
{
	LDMessage<> inputManager(input);
	int type;
	while (input.BytesUntilLimit() > 0)
	{
		readInt32(input, type);
		// TODO: What about doing that in future when needed??????
		UNORDERED(map)<int, tag_value>::const_iterator dR = root.decodingRules.find(type);
		if (dR != root.decodingRules.end()) {
			output.push_back(dR->second);
		}
	}
	return true;
}

static const int SHIFT_COORDINATES = 5;
static const int MASK_TO_READ = ~((1 << SHIFT_COORDINATES) - 1);
bool readCoordinates(CodedInputStream & input, coordinates & output,
		MapTreeBounds const & tree)
{
	LDMessage<> inputManager(input);
	// Delta encoded coordinates
	int px = tree.Box().min_corner().x() & MASK_TO_READ;
	int py = tree.Box().min_corner().y() & MASK_TO_READ;
	int x;
	int y;
	while (input.BytesUntilLimit() > 0)
	{
		readSint32(input, x);
		readSint32(input, y);
		x = (x << SHIFT_COORDINATES) + px;
		y = (y << SHIFT_COORDINATES) + py;
		output.push_back(std::make_pair(x, y));
		px = x;
		py = y;
	}

	return true;
}

#define NODE_CAPACITY 40

MapDataObject* readMapDataObject(CodedInputStream & input,
		MapTreeBounds const & tree, MapIndex const & index)
{
	LDMessage<> inputManager(input);

	// Must start with line or ring/area description
	uint32_t tag = WireFormatLite::GetTagFieldNumber(input.ReadTag());
	bool area = MapData::kAreaCoordinatesFieldNumber == tag;
	if(!area && MapData::kCoordinatesFieldNumber != tag)
	{
		input.Skip(input.BytesUntilLimit());
		return NULL;
	}

	MapDataObject* dataObject = new MapDataObject();
	dataObject->area = area;
	readCoordinates(input, dataObject->points, tree);

	while ((tag = input.ReadTag()) != 0)
	{
		switch (WireFormatLite::GetTagFieldNumber(tag))
		{
		case MapData::kPolygonInnerCoordinatesFieldNumber:
		{
			coordinates polygon;
			readCoordinates(input, polygon, tree);
			dataObject->polygonInnerCoordinates.push_back(std::move(polygon));
			break;
		}
		case MapData::kAdditionalTypesFieldNumber:
			readTypes(input, dataObject->additionalTypes, index);
			break;
		case MapData::kTypesFieldNumber:
			readTypes(input, dataObject->types, index);
// Check on search
//			if (!acceptTypes(req, dataObject->types, index))
//			{
//				input.Skip(input.BytesUntilLimit());
//				return NULL;
//			}
			break;
		case MapData::kIdFieldNumber:
		{
			int64_t id;
			readSint64(input, id);
			dataObject->id = id;
		}
			break;
		case MapData::kStringNamesFieldNumber:
			readStrIds(input, dataObject->stringIds, index);
			break;
		default:
			if (!skipUnknownFields(input, tag))
			{
				input.Skip(input.BytesUntilLimit());
				return NULL;
			}
			break;
		}
	} // end of while

	return dataObject;
}

bool readMapDataBlocks(CodedInputStream & input, MapTreeBounds & output,
		MapIndex const & index)
{
//OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Error, "readMapData from %d", input.TotalBytesRead());
	LDMessage<> inputManager(input);
	uint64_t baseId = 0;
	MapDataObjects_t dataObjects;
	dataObjects.reserve(NODE_CAPACITY);
	StringTable_t stringTable;
	int tag;
	while ((tag = input.ReadTag()) != 0)
	{
		switch (WireFormatLite::GetTagFieldNumber(tag))
		{
		case MapDataBlock::kBaseIdFieldNumber:
			readUInt64(input, baseId);
			break;
		case MapDataBlock::kStringTableFieldNumber:
		{
			if(dataObjects.size() > 0) {
				readStringTable(input, stringTable);
				for (MapDataObjects_t::const_iterator obj = dataObjects.begin(); obj != dataObjects.end(); obj++) {
					UNORDERED(map)<std::string, unsigned int >::const_iterator  val=(*obj)->stringIds.begin();
					while(val != (*obj)->stringIds.end()){
						(*obj)->objectNames[val->first]=stringTable[val->second];
						val++;
					}
				}
			}
			else
				skipUnknownFields(input, tag);
			break;
		}
		case MapDataBlock::kDataObjectsFieldNumber:
		{
			MapDataObject* mapObject = readMapDataObject(input, output, index);
			if (mapObject != NULL) {
				mapObject->id += baseId;
				dataObjects.push_back(mapObject);
			}
			break;
		}
		default:
			if (!skipUnknownFields(input, tag)) {
				return false;
			}
			break;
		}
	} // End of while

	output.DataObjects(std::move(dataObjects));
	return true;
}

bool readMapTreeBoundsBase(CodedInputStream & input, MapTreeBounds & output,
		MapTreeBounds const & root, MapIndex const & index, int fd);
bool readMapTreeBoundsNodes(CodedInputStream & input, MapTreeBounds & output,
		MapIndex const & index, int fd)
{
//OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Error, "readRouteTreeNodes from %d", input.TotalBytesRead());
	LDMessage<OSMAND_FIXED32> inputManager(input);
	MapTreeBounds::Bounds_t nodes;
	nodes.reserve(NODE_CAPACITY);
	int tag;
	while ((tag = input.ReadTag()) != 0)
	{
		switch (WireFormatLite::GetTagFieldNumber(tag))
		{
		case OsmAndMapIndex_MapDataBox::kBoxesFieldNumber:
		{
			MapTreeBounds node;
			if (output.ocean) {
				node.ocean = output.ocean;
			}
			readMapTreeBoundsBase(input, node, output, index, fd);
			nodes.push_back(std::move(node));
			break;
		}
		default:
			if (!skipUnknownFields(input, tag)) {
				return false;
			}
			break;
		}
	}  // End of while

	output.Children(std::move(nodes));
	return true;
}

bool readMapTreeBoundsBase(CodedInputStream & input, MapTreeBounds & output,
		MapTreeBounds const & root, MapIndex const & index, int fd)
{
//OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Error, "new MapBB pos %d", input.TotalBytesRead());
	uint32_t lPos = input.TotalBytesRead();
	LDMessage<OSMAND_FIXED32> inputManager(input);
	uint32_t mPos = input.TotalBytesRead();
	uint32_t objectsOffset = 0; // Will be an intermediate node but if has ShiftToMaplPos);

	int tag;
	int32_t si;
	while ((tag = input.ReadTag()) != 0)
	{
		switch (WireFormatLite::GetTagFieldNumber(tag))
		{
		// Delta encoded box coordinates
		case OsmAndMapIndex_MapDataBox::kLeftFieldNumber:
			readSint32(input, si);
			output.left = si + root.left;
			break;
		case OsmAndMapIndex_MapDataBox::kRightFieldNumber:
			readSint32(input, si);
			output.right = si + root.right;
			break;
		case OsmAndMapIndex_MapDataBox::kTopFieldNumber:
			readSint32(input, si);
			output.top = si + root.top;
			break;
		case OsmAndMapIndex_MapDataBox::kBottomFieldNumber:
			readSint32(input, si);
			output.bottom = si + root.bottom;
			break;
		case OsmAndMapIndex_MapDataBox::kOceanFieldNumber:
			readBool(input, output.ocean);
			break;

		// Is a leaf
		case OsmAndMapIndex_MapDataBox::kShiftToMapDataFieldNumber:
			readInt(input, objectsOffset);
//OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Error, "Leaf node. Data offset %d", objectsOffset);
			break;
		default:
			if (!skipUnknownFields(input, tag)) {
				return false;
			}
			break;
		}
	}  // end of while

	output.Box(bbox_t(point_t(output.left, output.top), point_t(output.right, output.bottom)));
//{
//std::ostringstream msg;
//msg << "MapTreeBounds Box read " << output.Box() << std::endl;
//OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Error, msg.str().c_str());
//}
	if (objectsOffset == 0)
	{  // An intermediate node.
		output.ContentReader([lPos, &index, fd](MapTreeBounds & output)
				{
			int filed = dup(fd); // Duplicated fd to permit parallel access.
			lseek(filed, 0, SEEK_SET); // need to start from begining
			FileInputStream is(filed);
			is.SetCloseOnDelete(true);
			CodedInputStream input(&is);
			input.SetTotalBytesLimit(INT_MAX, INT_MAX >> 1);
			input.Seek(lPos); // Needed to correctelly set TotalBytesRead
			readMapTreeBoundsNodes(input, output, index, fd);
				});
	}
	else
	{  // A leaf node.
		uint32_t pos = mPos+objectsOffset;
		output.ContentReader([pos, &index, fd](MapTreeBounds & output)
				{
			int filed = dup(fd); // Duplicated fd to permit parallel access.
			lseek(filed, 0, SEEK_SET); // need to start from begining
			FileInputStream is(filed);
			is.SetCloseOnDelete(true);
			CodedInputStream input(&is);
			input.SetTotalBytesLimit(INT_MAX, INT_MAX >> 1);
			input.Seek(pos); // Needed to correctelly set TotalBytesRead
			readMapDataBlocks(input, output, index);
				});
	}

	return true;
}

/***
bool readMapLevel(CodedInputStream* input, MapRoot* root, bool initSubtrees) {
	int tag;
	int si;
	while ((tag = input->ReadTag()) != 0) {
		switch (WireFormatLite::GetTagFieldNumber(tag)) {
		case OsmAndMapIndex_MapRootLevel::kMaxZoomFieldNumber: {
			int z;
			DO_((WireFormatLite::ReadPrimitive<int32_t, WireFormatLite::TYPE_INT32>(input, &z)));
			root->maxZoom = z;
			break;
		}
		case OsmAndMapIndex_MapRootLevel::kMinZoomFieldNumber: {
			int z;
			DO_((WireFormatLite::ReadPrimitive<int32_t, WireFormatLite::TYPE_INT32>(input, &z)));
			root->minZoom = z;
			break;
		}
		case OsmAndMapIndex_MapRootLevel::kBottomFieldNumber: {
			DO_((WireFormatLite::ReadPrimitive<int32_t, WireFormatLite::TYPE_INT32>(input, &si)));
			root->bottom = si;
			break;
		}
		case OsmAndMapIndex_MapRootLevel::kTopFieldNumber: {
			DO_((WireFormatLite::ReadPrimitive<int32_t, WireFormatLite::TYPE_INT32>(input, &si)));
			root->top = si;
			break;
		}
		case OsmAndMapIndex_MapRootLevel::kLeftFieldNumber: {
			DO_((WireFormatLite::ReadPrimitive<int32_t, WireFormatLite::TYPE_INT32>(input, &si)));
			root->left = si;
			break;
		}
		case OsmAndMapIndex_MapRootLevel::kRightFieldNumber: {
			DO_((WireFormatLite::ReadPrimitive<int32_t, WireFormatLite::TYPE_INT32>(input, &si)));
			root->right = si;
			break;
		}
		case OsmAndMapIndex_MapRootLevel::kBoxesFieldNumber: {
			if (!initSubtrees) {
				input->Skip(input->BytesUntilLimit());
				break;
			}
			MapTreeBounds bounds;
			readInt(input, &bounds.length);
			bounds.filePointer = input->TotalBytesRead();
			int oldLimit = input->PushLimit(bounds.length);
			readMapTreeBounds(input, &bounds, root);
			root->bounds.push_back(bounds);
			input->PopLimit(oldLimit);
			break;
		}

		case OsmAndMapIndex_MapRootLevel::kBlocksFieldNumber: {
			input->Skip(input->BytesUntilLimit());
			break;
		}

		default: {
			if (WireFormatLite::GetTagWireType(tag) == WireFormatLite::WIRETYPE_END_GROUP) {
				return true;
			}
			if (!skipUnknownFields(input, tag)) {
				return false;
			}
			break;
		}
		}
	}
	return true;
}
***/

//bool readMapLevelNodes(CodedInputStream & input, MapRoot & output,
bool readMapLevelNodes(CodedInputStream & input, MapTreeBounds & output,
		MapIndex const & index, int fd)
{
	LDMessage<OSMAND_FIXED32> inputMnager(input);
	MapRoot::Bounds_t nodes;
	int tag;
	while ((tag = input.ReadTag()) != 0)
	{
		switch (WireFormatLite::GetTagFieldNumber(tag))
		{
		case OsmAndMapIndex_MapRootLevel::kBoxesFieldNumber:
		{
			MapTreeBounds bounds;
			readMapTreeBoundsBase(input, bounds, output, index, fd);
			nodes.push_back(std::move(bounds));
		}
			break;
		case OsmAndMapIndex_MapRootLevel::kBlocksFieldNumber:
			// Fast end
			input.Skip(input.BytesUntilLimit());
			break;
		default:
			if (!skipUnknownFields(input, tag)) {
				return false;
			}
			break;
		}
	}  // End of while

	output.Children(std::move(nodes));
	return true;
}

bool readMapLevelBase(CodedInputStream & input, MapRoot & output,
		MapIndex const & index, int fd)
{
//OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Error, "new MapLevel pos %d", input.TotalBytesRead());
	uint32_t pos = input.TotalBytesRead();
	LDMessage<OSMAND_FIXED32> inputManager(input);
	int tag;
	int si;
	while ((tag = input.ReadTag()) != 0)
	{
		switch (WireFormatLite::GetTagFieldNumber(tag))
		{
		case OsmAndMapIndex_MapRootLevel::kMaxZoomFieldNumber:
			readInt32(input, output.maxZoom);
			break;
		case OsmAndMapIndex_MapRootLevel::kMinZoomFieldNumber:
			readInt32(input, output.minZoom);
			break;
		case OsmAndMapIndex_MapRootLevel::kBottomFieldNumber:
			readInt32(input, si);
			output.bottom = si;
			break;
		case OsmAndMapIndex_MapRootLevel::kTopFieldNumber:
			readInt32(input, si);
			output.top = si;
			break;
		case OsmAndMapIndex_MapRootLevel::kLeftFieldNumber:
			readInt32(input, si);
			output.left = si;
			break;
		case OsmAndMapIndex_MapRootLevel::kRightFieldNumber:
			readInt32(input, si);
			output.right = si;
			break;
		case OsmAndMapIndex_MapRootLevel::kBoxesFieldNumber:
		case OsmAndMapIndex_MapRootLevel::kBlocksFieldNumber:
			// Fast end
			input.Skip(input.BytesUntilLimit());
			break;
		default:
			if (!skipUnknownFields(input, tag)) {
				return false;
			}
			break;
		}
	} // End of while

	output.Box(bbox_t(point_t(output.left, output.top), point_t(output.right, output.bottom)));
//{
//std::ostringstream msg;
//msg << "MapRoot Box read " << output.Box() << std::endl;
//OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Error, msg.str().c_str());
//}
	// Lazy read of nodes
	output.ContentReader([pos, &index, fd](MapTreeBounds & output)
			{
		int filed = dup(fd);
		lseek(filed, 0, SEEK_SET);  // We start from the begining
		FileInputStream is(filed);
		is.SetCloseOnDelete(true);
		CodedInputStream input(&is);
		input.SetTotalBytesLimit(INT_MAX, INT_MAX >> 1);
		input.Seek(pos); // Needed to correctelly set TotalBytesRead
		readMapLevelNodes(input, output, index, fd);
			});

	return true;
}

bool readMapEncodingRule(CodedInputStream & input, MapIndex & output, uint32_t id)
{
	LDMessage<> inputManager(input);
	int tag;
	std::string tagS;
	std::string value;
	uint32_t type = 0;
	while ((tag = input.ReadTag()) != 0)
	{
		switch (WireFormatLite::GetTagFieldNumber(tag))
		{
		case OsmAndMapIndex_MapEncodingRule::kValueFieldNumber:
			WireFormatLite::ReadString(&input, &value);
			break;
		case OsmAndMapIndex_MapEncodingRule::kTagFieldNumber:
			WireFormatLite::ReadString(&input, &tagS);
			break;
		case OsmAndMapIndex_MapEncodingRule::kTypeFieldNumber:
			readUInt32(input, type);
			break;
		case OsmAndMapIndex_MapEncodingRule::kIdFieldNumber:
			readUInt32(input, id);
			break;
		default:
			if (!skipUnknownFields(input, tag)) {
				return false;
			}
			break;
		}
	}  // End of while

	// Special case for check to not replace primary with primary_link
	output.initMapEncodingRule(type, id, tagS, value);
	return true;
}

bool readMapIndex(CodedInputStream & input, MapIndex & output,
		int fd)
{
//OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Error, "new MapIndex pos %d", input.TotalBytesRead());
	LDMessage<OSMAND_FIXED32> inputManager(input);
	uint32_t tag;
	uint32_t defaultId = 1;
	while ((tag = input.ReadTag()) != 0)
	{
		switch (WireFormatLite::GetTagFieldNumber(tag))
		{
		case OsmAndMapIndex::kNameFieldNumber:
			WireFormatLite::ReadString(&input, &output.name);
			break;
		case OsmAndMapIndex::kRulesFieldNumber:
			readMapEncodingRule(input, output, defaultId++);
			break;
		case OsmAndMapIndex::kLevelsFieldNumber:
		{
			MapRoot mapLevel;
			readMapLevelBase(input, mapLevel, output, fd);
			output.levels.push_back(std::move(mapLevel));
			break;
		}
		default:
			if (!skipUnknownFields(input, tag)) {
				return false;
			}
			break;
		}
	}  // end of while

	output.finishInitializingTags();
	// Calculate bounding box.
	bbox_t box(output.Box());
	// My accumulate
	using boost::range::for_each;
	using boost::geometry::expand;
	for_each(output.levels, [&box](MapRoot const & r){expand(box, r.Box());});
	output.Box(box);
//{
//std::ostringstream msg;
//msg << "Box " << output.Box() << std::endl;
//OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Error, msg.str().c_str());
//}
//OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Error, "readMI #decR %d, #levels %d",
//			output.decodingRules.size(), output.levels.size());
	return true;
}

// Only called from renderImage on MapCreator
bool acceptTypes(SearchQuery* req, std::vector<tag_value> const & types, MapIndex const * root) {
	RenderingRuleSearchRequest* r = req->req;
	for (std::vector<tag_value>::const_iterator type = types.begin(); type != types.end(); type++) {
		for (int i = 1; i <= 3; i++) {
			r->setIntFilter(r->props()->R_MINZOOM, req->zoom);
			r->setStringFilter(r->props()->R_TAG, type->first);
			r->setStringFilter(r->props()->R_VALUE, type->second);
			if (r->search(i, false)) {
				return true;
			}
		}
		r->setStringFilter(r->props()->R_TAG, type->first);
		r->setStringFilter(r->props()->R_VALUE, type->second);
		r->setStringFilter(r->props()->R_NAME_TAG, "");
		if (r->search(RenderingRulesStorage::TEXT_RULES, false)) {
			return true;
		}
	}

	return false;
}

/***
// Only called from renderImage on MapCreator
bool searchMapTreeBounds(CodedInputStream* input, MapTreeBounds* current, MapTreeBounds const * parent,
		SearchQuery * req, std::vector<MapTreeBounds>* foundSubtrees) {
	int init = 0;
	int tag;
	int si;
	req->numberOfReadSubtrees++;
	while ((tag = input->ReadTag()) != 0) {
		if (req->publisher->isCancelled()) {
			return false;
		}
		if (init == 0xf) {
			init = 0;
			// coordinates are init
			if (current->right < req->left || current->left > req->right ||
				current->top > req->bottom || current->bottom < req->top) {
				return false;
			} else {
				req->numberOfAcceptedSubtrees++;
			}
		}
		switch (WireFormatLite::GetTagFieldNumber(tag)) {
		// required uint32_t version = 1;
		case OsmAndMapIndex_MapDataBox::kLeftFieldNumber: {
			DO_((WireFormatLite::ReadPrimitive<int32_t, WireFormatLite::TYPE_SINT32>(input, &si)));
			current->left = si + parent->left;
			init |= 1;
			break;
		}
		case OsmAndMapIndex_MapDataBox::kRightFieldNumber: {
			DO_((WireFormatLite::ReadPrimitive<int32_t, WireFormatLite::TYPE_SINT32>(input, &si)));
			current->right = si + parent->right;
			init |= 2;
			break;
		}
		case OsmAndMapIndex_MapDataBox::kTopFieldNumber: {
			DO_((WireFormatLite::ReadPrimitive<int32_t, WireFormatLite::TYPE_SINT32>(input, &si)));
			current->top = si + parent->top;
			init |= 4;
			break;
		}
		case OsmAndMapIndex_MapDataBox::kBottomFieldNumber : {
			DO_((WireFormatLite::ReadPrimitive<int32_t, WireFormatLite::TYPE_SINT32>(input, &si)));
			current->bottom = si +  parent->bottom;
			init |= 8;
			break;
		}
		case OsmAndMapIndex_MapDataBox::kShiftToMapDataFieldNumber : {
			readInt(input, &current->mapDataBlock);
			current->mapDataBlock += current->filePointer;
			foundSubtrees->push_back(*current);
			break;
		}
		case OsmAndMapIndex_MapDataBox::kOceanFieldNumber : {
			DO_((WireFormatLite::ReadPrimitive<bool, WireFormatLite::TYPE_BOOL>(input, &current->ocean)));
			if(current->ocean){
				req->ocean = true;
			} else {
				req->mixed = true;
			}
			break;
		}
		case OsmAndMapIndex_MapDataBox::kBoxesFieldNumber: {
			MapTreeBounds* child = new MapTreeBounds();
			readInt(input, &child->length);
			child->filePointer = input->TotalBytesRead();
			int oldLimit = input->PushLimit(child->length);
			if (current->ocean) {
				child->ocean = current->ocean;
			}
			searchMapTreeBounds(input, child, current, req, foundSubtrees);
			input->PopLimit(oldLimit);
			input->Seek(child->filePointer + child->length);
			delete child;
			break;
		}
		default: {
			if (WireFormatLite::GetTagWireType(tag) == WireFormatLite::WIRETYPE_END_GROUP) {
				return true;
			}
			if (!skipUnknownFields(input, tag)) {
				return false;
			}
			break;
		}
		}
	}
	return true;
}
***/


////bool sortTreeBounds (const MapTreeBounds& i,const MapTreeBounds& j) { return (i.mapDataBlock<j.mapDataBlock); }

/****
// Only called from renderImage on MapCreator
void searchMapData(CodedInputStream* input, MapRoot* root, MapIndex const * ind, SearchQuery * req) {
	// search
	for (std::vector<MapTreeBounds>::iterator i = root->bounds.begin();
			i != root->bounds.end(); i++) {
		if (req->publisher->isCancelled()) {
			return;
		}
		if (i->right < req->left || i->left > req->right ||
				i->top > req->bottom || i->bottom < req->top) {
			continue;
		}
		std::vector<MapTreeBounds> foundSubtrees;
		input->Seek(i->filePointer);
		int oldLimit = input->PushLimit(i->length);
		searchMapTreeBounds(input, &(*i), root, req, &foundSubtrees);
		input->PopLimit(oldLimit);

		sort(foundSubtrees.begin(), foundSubtrees.end(), sortTreeBounds);
		uint32_t length;
		for (std::vector<MapTreeBounds>::const_iterator tree = foundSubtrees.begin();
					tree != foundSubtrees.end(); tree++) {
			if (req->publisher->isCancelled()) {
				return;
			}
			input->Seek(tree->mapDataBlock);
			WireFormatLite::ReadPrimitive<uint32_t, WireFormatLite::TYPE_UINT32>(input, &length);
			int oldLimit = input->PushLimit(length);
			readMapDataBlocks(input, req, &(*tree), ind);
			input->PopLimit(oldLimit);
		}
	}
}
***/

/****
// Needed to read rules when files where initialized fron cache file.
void checkAndInitRouteRegionRules(int fileInd, RoutingIndex* routingIndex){
	// init decoding rules
	if (routingIndex->decodingRules.size() == 0) {
		lseek(fileInd, 0, SEEK_SET);
		FileInputStream input(fileInd);
		input.SetCloseOnDelete(false);
		CodedInputStream cis(&input);
		cis.SetTotalBytesLimit(INT_MAXIMUM, INT_MAXIMUM >> 1);

		cis.Seek(routingIndex->filePointer);
		uint32_t old = cis.PushLimit(routingIndex->length);
		readRoutingIndex(&cis, routingIndex, true);
		cis.PopLimit(old);
	}
}
****/

/***
// Only called from renderImage on MapCreator
void readMapObjects(SearchQuery * q, BinaryMapFile* file) {
	for (std::vector<MapIndex>::iterator mapIndex = file->mapIndexes.begin(); mapIndex != file->mapIndexes.end();
			mapIndex++) {
		for (std::vector<MapRoot>::iterator mapLevel = mapIndex->levels.begin(); mapLevel != mapIndex->levels.end();
				mapLevel++) {
			if (q->publisher->isCancelled()) {
				break;
			}

			if (mapLevel->minZoom <= q->zoom && mapLevel->maxZoom >= q->zoom) {
				if (mapLevel->right >= q->left && q->right >= mapLevel->left &&
						mapLevel->bottom >= q->top && q->bottom >= mapLevel->top) {
					// OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Info, "Search map %s", mapIndex->name.c_str());
					// lazy initializing rules
					if (mapIndex->decodingRules.size() == 0) {
						lseek(file->fd, 0, SEEK_SET);
						FileInputStream input(file->fd);
						input.SetCloseOnDelete(false);
						CodedInputStream cis(&input);
						cis.SetTotalBytesLimit(INT_MAXIMUM, INT_MAXIMUM >> 1);
						cis.Seek(mapIndex->filePointer);
						int oldLimit = cis.PushLimit(mapIndex->length);
						readMapIndex(&cis, &(*mapIndex), true);
						cis.PopLimit(oldLimit);
					}
					// lazy initializing subtrees
					if (mapLevel->bounds.size() == 0) {
						lseek(file->fd, 0, SEEK_SET);
						FileInputStream input(file->fd);
						input.SetCloseOnDelete(false);
						CodedInputStream cis(&input);
						cis.SetTotalBytesLimit(INT_MAXIMUM, INT_MAXIMUM >> 1);
						cis.Seek(mapLevel->filePointer);
						int oldLimit = cis.PushLimit(mapLevel->length);
						readMapLevel(&cis, &(*mapLevel), true);
						cis.PopLimit(oldLimit);
					}
					lseek(file->fd, 0, SEEK_SET);
					FileInputStream input(file->fd);
					input.SetCloseOnDelete(false);
					CodedInputStream cis(&input);
					cis.SetTotalBytesLimit(INT_MAX, INT_MAX >> 1);
					searchMapData(&cis, &(*mapLevel), &(*mapIndex), q);
				}
			}
		}
	}
}
***/
