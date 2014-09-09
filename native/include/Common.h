#ifndef _OSMAND_COMMON_H_
#define _OSMAND_COMMON_H_

// Unordered containers
#include <unordered_map>
#include <unordered_set>
#define UNORDERED_NAMESPACE std
#define UNORDERED_map unordered_map
#define UNORDERED_set unordered_set
#define UNORDERED(cls) UNORDERED_NAMESPACE::UNORDERED_##cls
 
// Smart pointers
#include <memory>
#define SHARED_PTR std::shared_ptr

//namespace OsmAnd
//{
//    typedef UNORDERED(map)<std::string, float> StringToFloatMap;
//    typedef UNORDERED(map)<std::string, std::string> StringToStringMap;
//}

typedef std::pair<std::string, std::string> tag_value;

#endif // _OSMAND_COMMON_H_
