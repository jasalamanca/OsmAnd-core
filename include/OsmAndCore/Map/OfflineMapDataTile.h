/**
* @file
*
* @section LICENSE
*
* OsmAnd - Android navigation software based on OSM maps.
* Copyright (C) 2010-2013  OsmAnd Authors listed in AUTHORS file
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.

* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef __OFFLINE_MAP_DATA_TILE_H_
#define __OFFLINE_MAP_DATA_TILE_H_

#include <cstdint>
#include <memory>

#include <QList>

#include <OsmAndCore.h>
#include <OsmAndCore/CommonTypes.h>
#include <OsmAndCore/Map/MapTypes.h>

namespace OsmAnd {

    class OfflineMapDataProvider;
    class OfflineMapDataProvider_P;
    namespace Model {
        class MapObject;
    }

    class OfflineMapDataTile_P;
    class OSMAND_CORE_API OfflineMapDataTile
    {
    private:
        const std::unique_ptr<OfflineMapDataTile_P> _d;
    protected:
        OfflineMapDataTile(const MapFoundationType tileFoundation, const QList< std::shared_ptr<const Model::MapObject> >& mapObjects);
    public:
        virtual ~OfflineMapDataTile();

        const MapFoundationType tileFoundation;
        const QList< std::shared_ptr<const Model::MapObject> > mapObjects;

    friend class OsmAnd::OfflineMapDataProvider;
    friend class OsmAnd::OfflineMapDataProvider_P;
    };

} // namespace OsmAnd

#endif // __OFFLINE_MAP_DATA_TILE_H_
