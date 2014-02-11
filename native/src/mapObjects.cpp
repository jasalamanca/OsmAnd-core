#include "mapObjects.h"

void deleteObjects(std::vector <MapDataObject* > & v)
{
	for(size_t i = 0; i< v.size(); i++)
	{
		delete v[i];
	}
	v.clear();
}
