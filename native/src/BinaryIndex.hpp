/*
 * BinaryIndex.hpp
 *
 *  Created on: 09/09/2014
 *      Author: javier
 */

#ifndef BINARYINDEX_HPP_
#define BINARYINDEX_HPP_

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

#endif /* BINARYINDEX_HPP_ */
