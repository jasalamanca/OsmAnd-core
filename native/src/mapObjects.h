#ifndef _OSMAND_MAP_OBJECTS_H
#define _OSMAND_MAP_OBJECTS_H

#include <vector>
#include <string>

#include "Common.h"

typedef std::pair<int, int> int_pair;
typedef std::vector< std::pair<int, int> > coordinates;


class MapDataObject
{
public:
	std::vector<tag_value>  types;
	std::vector<tag_value>  additionalTypes;
	coordinates points;
	std::vector < coordinates > polygonInnerCoordinates;

	// TODO Maybe auxiliar. Remove it.
	UNORDERED(map)< std::string, unsigned int> stringIds;

	UNORDERED(map)< std::string, std::string > objectNames;
	bool area;
	long long id;

	// Only called from renderImage on MapCreator
	bool cycle() const {
		return points[0] == points[points.size() -1];
	}
	bool containsAdditional(std::string const & key, std::string const & val) const {
		std::vector<tag_value>::const_iterator it = additionalTypes.begin();
		while (it != additionalTypes.end()) {
			if (it->first == key && it->second == val) {
				return true;
			}
			it++;
		}
		return false;
	}

	bool contains(std::string const & key, std::string const & val) const {
		std::vector<tag_value>::const_iterator it = types.begin();
		while (it != types.end()) {
			if (it->first == key) {
				return it->second == val;
			}
			it++;
		}
		return false;
	}

	// Only called from renderImage on MapCreator
	int getSimpleLayer() const {
		std::vector<tag_value>::const_iterator it = additionalTypes.begin();
		bool tunnel = false;
		bool bridge = false;
		while (it != additionalTypes.end()) {
			if (it->first == "layer") {
				if(it->second.length() > 0) {
					if(it->second[0] == '-'){
						return -1;
					} else if (it->second[0] == '0'){
						return 0;
					} else {
						return 1;
					}
				}
			} else if (it->first == "tunnel") {
				tunnel = "yes" == it->second;
			} else if (it->first == "bridge") {
				bridge = "yes" == it->second;
			}
			it++;
		}
		if (tunnel) {
			return -1;
		} else if (bridge) {
			return 1;
		}
		return 0;
	}
};



void deleteObjects(std::vector <MapDataObject* > & v);


#endif /*_OSMAND_MAP_OBJECTS_H*/
