#ifndef _OSMAND_RENDERING_H
#define _OSMAND_RENDERING_H

#include "common2.h"
#include "renderRules.h"
#include "mapObjects.h"
#include <vector>
#include <SkCanvas.h>


void doRendering(std::vector <MapDataObject* > const & mapDataObjects, SkCanvas & canvas,
		RenderingRuleSearchRequest* req,	RenderingContext & rc);

#endif
