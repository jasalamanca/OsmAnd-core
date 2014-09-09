#include "mapObjects.h"

#include "Logging.h"

void deleteObjects(std::vector <MapDataObject* > & v)
{
OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Warning, "deleteObjects");
	for(size_t i = 0; i< v.size(); i++)
	{
		delete v[i];
	}
	v.clear();
}
