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
	output.reserve(4);
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

bool readMainCoordinates(CodedInputStream & input, MapDataObject & output,
		MapTreeBounds const & tree)
{
	if (!readCoordinates(input, output.points, tree)) return false;
	bbox_t box = output.Box();
	boost::for_each(output.points,
			[&box](std::pair<int, int> const & p){ boost::geometry::expand(box, point_t(p.first, p.second)); });
	output.Box(box);
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
	readMainCoordinates(input, *dataObject, tree);

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

bool readMapLevelNodes(CodedInputStream & input, MapTreeBounds & output,
		MapIndex const & index, int fd)
{
	LDMessage<OSMAND_FIXED32> inputMnager(input);
	MapRoot::Bounds_t nodes(NODE_CAPACITY);
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
	output.levels.reserve(4);
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
