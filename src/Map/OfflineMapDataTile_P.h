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

#ifndef __OFFLINE_MAP_DATA_TILE_P_H_
#define __OFFLINE_MAP_DATA_TILE_P_H_

#include <cstdint>
#include <memory>

#include <OsmAndCore.h>
#include <CommonTypes.h>
#include <OfflineMapDataProvider_P.h>

namespace OsmAnd {

    class OfflineMapDataTile;
    class OfflineMapDataTile_P
    {
    private:
    protected:
        OfflineMapDataTile_P(OfflineMapDataTile* owner);

        OfflineMapDataTile* const owner;

        std::weak_ptr<OfflineMapDataProvider_P::Link> _link;
        std::weak_ptr<OfflineMapDataProvider_P::TileEntry> _refEntry;

        void cleanup();
    public:
        virtual ~OfflineMapDataTile_P();

    friend class OsmAnd::OfflineMapDataTile;
    friend class OsmAnd::OfflineMapDataProvider_P;
    };

} // namespace OsmAnd

#endif // __OFFLINE_MAP_DATA_TILE_P_H_
