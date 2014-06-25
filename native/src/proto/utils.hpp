/*
 * utils.hpp
 *
 *  Created on: 18/06/2014
 *      Author: javier
 */

#ifndef UTILS_HPP_
#define UTILS_HPP_

#include <google/protobuf/wire_format_lite_inl.h>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>

using google::protobuf::io::CodedInputStream;
using google::protobuf::io::FileInputStream;
using google::protobuf::internal::WireFormatLite;

#define INT_MAXIMUM 0x7fffffff

// QUITAR
inline bool readInt(CodedInputStream* input, uint32_t* sz ){
	uint8_t buf[4];
	if (!input->ReadRaw(buf, 4)) {
		return false;
	}
	*sz = ((buf[0] << 24) + (buf[1] << 16) + (buf[2] << 8) + buf[3]);
	return true;
}

inline bool readInt(CodedInputStream & input, uint32_t & sz)
{
	uint8_t buf[4];
	if (!input.ReadRaw(buf, 4)) {
		return false;
	}
	sz = ((buf[0] << 24) + (buf[1] << 16) + (buf[2] << 8) + buf[3]);
	return true;
}

inline bool skipFixed32(CodedInputStream* input) {
	uint32_t sz;
	if (!readInt(input, &sz)) {
		return false;
	}
	return input->Skip(sz);
}

inline bool skipFixed32(CodedInputStream & input)
{
	uint32_t sz;
	if (!readInt(input, sz)) {
		return false;
	}
	return input.Skip(sz);
}

inline bool skipUnknownFields(CodedInputStream* input, int tag) {
	if (WireFormatLite::GetTagWireType(tag) == WireFormatLite::WIRETYPE_FIXED32_LENGTH_DELIMITED) {
		if (!skipFixed32(input)) {
			return false;
		}
	} else if (!WireFormatLite::SkipField(input, tag)) {
		return false;
	}
	return true;
}

inline bool skipUnknownFields(CodedInputStream & input, uint32_t const tag) {
	if (WireFormatLite::GetTagWireType(tag) == WireFormatLite::WIRETYPE_FIXED32_LENGTH_DELIMITED) {
		if (!skipFixed32(input)) {
			return false;
		}
	} else if (!WireFormatLite::SkipField(&input, tag)) {
		return false;
	}
	return true;
}


///////
//// EXTRACT
///////
inline bool readSint32(CodedInputStream & input, int32_t & output)
{
	return WireFormatLite::ReadPrimitive<int32_t, WireFormatLite::TYPE_SINT32>(&input, &output);
}

inline bool readSint64(CodedInputStream & input, int64_t & output)
{
	return WireFormatLite::ReadPrimitive<int64_t, WireFormatLite::TYPE_SINT64>(&input, &output);
}

inline bool readInt32(CodedInputStream & input, int32_t & output)
{
	return WireFormatLite::ReadPrimitive<int32_t, WireFormatLite::TYPE_INT32>(&input, &output);
}

inline bool readInt64(CodedInputStream & input, int64_t & output)
{
	return WireFormatLite::ReadPrimitive<int64_t, WireFormatLite::TYPE_INT64>(&input, &output);
}

inline bool readUInt32(CodedInputStream & input, uint32_t & output)
{
	return WireFormatLite::ReadPrimitive<uint32_t, WireFormatLite::TYPE_UINT32>(&input, &output);
}

inline bool readUInt64(CodedInputStream & input, uint64_t & output)
{
	return WireFormatLite::ReadPrimitive<uint64_t, WireFormatLite::TYPE_UINT64>(&input, &output);
}

enum LengthCodification
{
	PROTOBUF_DEFAULT,
	OSMAND_FIXED32
};
// Control limits for length delimited messages.
template <enum LengthCodification lc = PROTOBUF_DEFAULT>
class LDMessage
{
public:
	// Message starts here.
	LDMessage(CodedInputStream & in)
	: input(in)
	{
		old = takeLength();
	}
	// Message starts at pos.
	LDMessage(CodedInputStream & in, uint32_t pos)
	: input(in)
	{
		input.Seek(pos);
		old = takeLength();
	}
	// Message processed.
	~LDMessage()
	{
		input.PopLimit(old);
	}

private:
	uint32_t takeLength()
	{
		uint32_t length = 0;
		input.ReadVarint32(&length);
		return input.PushLimit(length);
	}

	CodedInputStream & input; ////
	uint32_t old;
};

template <>
inline uint32_t LDMessage<OSMAND_FIXED32>::takeLength()
{
	uint32_t length = 0;
	readInt(input, length);
	return input.PushLimit(length);
}

#endif /* UTILS_HPP_ */
