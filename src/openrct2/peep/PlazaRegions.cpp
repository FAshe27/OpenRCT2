/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "PlazaRegions.h"

#include "../Diagnostic.h"
#include "../GameState.h"
#include "../world/Footpath.h"
#include "../world/Map.h"
#include "../world/tile_element/BannerElement.h"
#include "../world/tile_element/EntranceElement.h"
#include "../world/tile_element/PathElement.h"
#include "../world/tile_element/TileElement.h"

#include <algorithm>

namespace OpenRCT2::Plaza
{
    namespace
    {
        bool _dirty = true; // cache starts dirty (design section 2.3)
        bool _diagnosticsLogged = false;
        TileCoordsXY _cacheMapSize{};
        std::vector<int32_t> _regionByTile; // PlazaGrid component ids (0 = none), row-major
        std::vector<Region> _regions;       // index = id - 1

        // Mirrors of the file-static helpers in GuestPathfinding.cpp
        // (GetBannerOnPath / BannerClearPathEdges / PathGetPermittedEdges).
        // They are duplicated here because the originals are translation-
        // unit static; the logic is small and stable, but any change to
        // banner edge-clearing in GuestPathfinding.cpp must be mirrored
        // here or door extraction will disagree with stock stepping.
        const TileElement* GetBannerOnPath(const TileElement* pathElement)
        {
            if (pathElement->isLastForTile())
                return nullptr;

            const TileElement* bannerElement = pathElement + 1;
            do
            {
                if (bannerElement->getType() == TileElementType::path)
                    return nullptr;
                if (bannerElement->getType() == TileElementType::banner)
                    return bannerElement;
                if (bannerElement->isLastForTile())
                    return nullptr;
            } while (bannerElement++ != nullptr);

            return nullptr;
        }

        int32_t PermittedEdgesOf(const PathElement* pathElement)
        {
            int32_t edges = pathElement->getEdgesAndCorners() & 0x0F;
            const TileElement* bannerElement = GetBannerOnPath(reinterpret_cast<const TileElement*>(pathElement));
            if (bannerElement != nullptr)
            {
                do
                {
                    edges &= bannerElement->asBanner()->getAllowedEdges();
                } while ((bannerElement = GetBannerOnPath(bannerElement)) != nullptr);
            }
            return edges;
        }

        // The flat (non-sloped), non-queue, non-ghost path element heights
        // present on one tile, folded into the PlazaGrid encoding.
        int32_t FlatPathZAt(const TileCoordsXY& tile)
        {
            TileElement* tileElement = MapGetFirstElementAt(tile);
            if (tileElement == nullptr)
                return PlazaGrid::kNoFlatPath;

            int32_t foundZ = PlazaGrid::kNoFlatPath;
            do
            {
                if (tileElement->isGhost())
                    continue;
                if (tileElement->getType() != TileElementType::path)
                    continue;
                const auto* pathElement = tileElement->asPath();
                if (pathElement->isQueue() || pathElement->isSloped())
                    continue;
                if (foundZ == PlazaGrid::kNoFlatPath)
                {
                    foundZ = tileElement->baseHeight;
                }
                else if (foundZ != tileElement->baseHeight)
                {
                    return PlazaGrid::kAmbiguousFlatPath;
                }
            } while (!(tileElement++)->isLastForTile());

            return foundZ;
        }

        const PathElement* PathElementAtZ(const TileCoordsXY& tile, int32_t z, bool flatNonQueueOnly)
        {
            TileElement* tileElement = MapGetFirstElementAt(tile);
            if (tileElement == nullptr)
                return nullptr;
            do
            {
                if (tileElement->isGhost())
                    continue;
                if (tileElement->getType() != TileElementType::path)
                    continue;
                if (tileElement->baseHeight != z)
                    continue;
                const auto* pathElement = tileElement->asPath();
                if (flatNonQueueOnly && (pathElement->isQueue() || pathElement->isSloped()))
                    continue;
                return pathElement;
            } while (!(tileElement++)->isLastForTile());
            return nullptr;
        }

        bool HasParkEntranceElementAt(const TileCoordsXY& tile, int32_t z)
        {
            TileElement* tileElement = MapGetFirstElementAt(tile);
            if (tileElement == nullptr)
                return false;
            do
            {
                if (tileElement->isGhost())
                    continue;
                if (tileElement->getType() != TileElementType::entrance)
                    continue;
                if (tileElement->baseHeight != z)
                    continue;
                if (tileElement->asEntrance()->getEntranceType() == EntranceType::parkEntrance)
                    return true;
            } while (!(tileElement++)->isLastForTile());
            return false;
        }

        void Rebuild()
        {
            _regions.clear();
            _regionByTile.clear();
            _cacheMapSize = getGameState().mapSize;
            const int32_t width = _cacheMapSize.x;
            const int32_t height = _cacheMapSize.y;
            if (width <= 0 || height <= 0)
            {
                _dirty = false;
                return;
            }

            const size_t total = static_cast<size_t>(width) * static_cast<size_t>(height);

            // Pass 1: flat path heights, then exact 2x2 eligibility, then
            // connected components (all PlazaGrid, all deterministic).
            std::vector<int32_t> flatPathZ(total, PlazaGrid::kNoFlatPath);
            for (int32_t y = 0; y < height; y++)
            {
                for (int32_t x = 0; x < width; x++)
                {
                    flatPathZ[static_cast<size_t>(PlazaGrid::GridIndex(width, { x, y }))] = FlatPathZAt({ x, y });
                }
            }

            const std::vector<char> eligible = PlazaGrid::ComputeEligible(flatPathZ, width, height);
            const PlazaGrid::Components components = PlazaGrid::LabelComponents(eligible, width, height);
            _regionByTile = components.id;

            // Pass 2: per-region data. A region's base z is the flat-path z
            // shared by its tiles (same-z by construction of the flood
            // fill over equal-z eligibility... note eligibility only links
            // equal-z 2x2 blocks, and the flood fill follows eligible
            // tiles, so all tiles of one component share one z).
            _regions.resize(static_cast<size_t>(components.count));
            for (size_t i = 0; i < _regions.size(); i++)
            {
                _regions[i].id = static_cast<uint16_t>(i + 1);
                _regions[i].baseZ = 0;
            }
            for (int32_t y = 0; y < height; y++)
            {
                for (int32_t x = 0; x < width; x++)
                {
                    const size_t idx = static_cast<size_t>(PlazaGrid::GridIndex(width, { x, y }));
                    const int32_t id = components.id[idx];
                    if (id == 0)
                        continue;
                    Region& region = _regions[static_cast<size_t>(id - 1)];
                    if (region.tiles.empty())
                        region.baseZ = flatPathZ[idx];
                    region.tiles.push_back({ x, y });
                }
            }

            // Pass 3: doors. Permitted edges come from the live tile data
            // (banner-cleared); landing legality mirrors the search's own
            // test: a path element on the landing tile that is valid for a
            // step from the region's height in the crossing direction
            // (FootpathIsZAndDirectionValid), or a park entrance element.
            std::vector<int32_t> permittedEdges(total, 0);
            std::vector<char> landingOk(total * 4, 0);
            for (int32_t y = 0; y < height; y++)
            {
                for (int32_t x = 0; x < width; x++)
                {
                    const TileCoordsXY tile{ x, y };
                    const size_t idx = static_cast<size_t>(PlazaGrid::GridIndex(width, { x, y }));
                    const int32_t id = components.id[idx];
                    if (id == 0)
                        continue;
                    const int32_t regionZ = _regions[static_cast<size_t>(id - 1)].baseZ;
                    const PathElement* insideElement = PathElementAtZ(tile, regionZ, true);
                    if (insideElement != nullptr)
                        permittedEdges[idx] = PermittedEdgesOf(insideElement);

                    for (int32_t dir = 0; dir < 4; dir++)
                    {
                        const PlazaGrid::Tile landingTile = PlazaGrid::TileInDirection({ x, y }, dir);
                        if (!PlazaGrid::InBounds(width, height, landingTile))
                            continue;
                        const TileCoordsXY landing{ landingTile.x, landingTile.y };
                        if (HasParkEntranceElementAt(landing, regionZ))
                        {
                            landingOk[idx * 4 + static_cast<size_t>(dir)] = 1;
                            continue;
                        }
                        // Any path element on the landing tile (thin,
                        // sloped or queue) that the stock validity test
                        // accepts for a step in this direction.
                        TileElement* tileElement = MapGetFirstElementAt(landing);
                        if (tileElement == nullptr)
                            continue;
                        do
                        {
                            if (tileElement->isGhost())
                                continue;
                            if (tileElement->getType() != TileElementType::path)
                                continue;
                            if (FootpathIsZAndDirectionValid(*tileElement->asPath(), regionZ, static_cast<Direction>(dir)))
                            {
                                // Also require the reverse connection, the
                                // same condition the search relies on: the
                                // landing's edges must lead back inside.
                                if (tileElement->asPath()->getEdges() & (1 << PlazaGrid::ReverseDirection(dir)))
                                {
                                    landingOk[idx * 4 + static_cast<size_t>(dir)] = 1;
                                    break;
                                }
                            }
                        } while (!(tileElement++)->isLastForTile());
                    }
                }
            }

            const std::vector<PlazaGrid::Door> doors = PlazaGrid::ExtractDoors(
                components.id, width, height, permittedEdges, landingOk);
            for (const auto& door : doors)
            {
                const int32_t id = components.id[static_cast<size_t>(PlazaGrid::GridIndex(width, door.inside))];
                if (id != 0)
                    _regions[static_cast<size_t>(id - 1)].doors.push_back(door);
            }

            _dirty = false;

            // M1 diagnostics: one console line per rebuild plus one per
            // region, behind the diagnostics flag so a rebuild triggered
            // mid-game does not spam; LogPlazaDiagnostics() emits the same
            // lines on demand.
            if (!_diagnosticsLogged)
            {
                _diagnosticsLogged = true;
                LogPlazaDiagnostics();
            }
        }
    } // namespace

    void InvalidatePlazaRegions()
    {
        _dirty = true;
    }

    bool PlazaRegionsDirty()
    {
        return _dirty;
    }

    bool EnsurePlazaRegions()
    {
        if (_dirty)
            Rebuild();
        return !_regionByTile.empty() || _regions.empty();
    }

    TileCoordsXY PlazaCacheMapSize()
    {
        return _cacheMapSize;
    }

    uint16_t GetRegionIdAt(const TileCoordsXYZ& loc)
    {
        if (!EnsurePlazaRegions())
            return 0;
        if (loc.x < 0 || loc.y < 0 || loc.x >= _cacheMapSize.x || loc.y >= _cacheMapSize.y)
            return 0;
        const int32_t id = _regionByTile[static_cast<size_t>(PlazaGrid::GridIndex(_cacheMapSize.x, { loc.x, loc.y }))];
        if (id == 0)
            return 0;
        const Region& region = _regions[static_cast<size_t>(id - 1)];
        if (region.baseZ != loc.z)
            return 0;
        return static_cast<uint16_t>(id);
    }

    const Region* FindRegion(uint16_t regionId)
    {
        if (regionId == 0 || !EnsurePlazaRegions())
            return nullptr;
        if (regionId > _regions.size())
            return nullptr;
        return &_regions[regionId - 1];
    }

    const std::vector<Region>& AllRegions()
    {
        EnsurePlazaRegions();
        return _regions;
    }

    bool RevalidateRegionTile(const TileCoordsXYZ& loc, uint16_t regionId)
    {
        const PathElement* element = PathElementAtZ({ loc.x, loc.y }, loc.z, true);
        if (element == nullptr)
        {
            InvalidatePlazaRegions();
            return false;
        }
        if (GetRegionIdAt(loc) != regionId)
        {
            InvalidatePlazaRegions();
            return false;
        }
        return true;
    }

    bool ComputeSteerDestination(
        const Region& region, const CoordsXY& fromWorld, const TileCoordsXYZ& start, const TileCoordsXYZ& target,
        bool targetIsDoorLanding, CoordsXY& outDestination, TileCoordsXYZ& outTargetTile)
    {
        if (_dirty || _regionByTile.empty())
            return false;
        const int32_t width = _cacheMapSize.x;
        const int32_t height = _cacheMapSize.y;
        const int32_t regionId = region.id;

        // Resolve the in-region endpoint: the target itself, or the inside
        // tile of the door whose landing is the target.
        PlazaGrid::Tile goalTile{ target.x, target.y };
        const PlazaGrid::Door* door = nullptr;
        if (targetIsDoorLanding)
        {
            for (const auto& candidate : region.doors)
            {
                if (candidate.landing.x == target.x && candidate.landing.y == target.y)
                {
                    door = &candidate;
                    goalTile = candidate.inside;
                    break;
                }
            }
            if (door == nullptr)
                return false;
        }

        const PlazaGrid::Tile startTile{ start.x, start.y };
        const std::vector<PlazaGrid::Tile> bfsPath = PlazaGrid::BfsPath(
            _regionByTile, width, height, regionId, startTile, goalTile);
        if (bfsPath.empty())
            return false;

        const PlazaGrid::WorldPoint anchor{ fromWorld.x, fromWorld.y };

        // Already standing on the endpoint tile: aim straight at the final
        // target (goal tile centre, or the landing tile centre for a door).
        if (bfsPath.size() == 1)
        {
            const PlazaGrid::WorldPoint dest = PlazaGrid::TileCentre({ target.x, target.y });
            const PlazaGrid::Tile extra = targetIsDoorLanding ? PlazaGrid::Tile{ target.x, target.y }
                                                              : PlazaGrid::Tile{ -1, -1 };
            if (!PlazaGrid::StaircaseStaysIn(
                    _regionByTile, width, height, regionId, anchor, dest, extra, PlazaGrid::kWalkTolerance))
                return false;
            outDestination = { dest.x, dest.y };
            outTargetTile = target;
            return true;
        }

        const std::vector<PlazaGrid::Tile> pulled = PlazaGrid::StringPull(
            _regionByTile, width, height, regionId, bfsPath, anchor);
        if (pulled.empty())
            return false;

        const PlazaGrid::Tile waypoint = pulled.front();
        const bool completesInRegion = (waypoint == goalTile);

        if (completesInRegion && targetIsDoorLanding)
        {
            // Final hop: try to aim all the way at the landing tile centre.
            // The staircase from the current anchor to the landing centre
            // must stay on region tiles plus the landing tile itself.
            const PlazaGrid::WorldPoint landingCentre = PlazaGrid::TileCentre({ target.x, target.y });
            if (PlazaGrid::StaircaseStaysIn(
                    _regionByTile, width, height, regionId, anchor, landingCentre, { target.x, target.y },
                    PlazaGrid::kWalkTolerance))
            {
                outDestination = { landingCentre.x, landingCentre.y };
                outTargetTile = target;
                return true;
            }
            // Otherwise aim at the door's inside tile; on arrival the next
            // steering call runs the size-1 case above and crosses.
        }

        const PlazaGrid::WorldPoint dest = PlazaGrid::TileCentre(waypoint);
        outDestination = { dest.x, dest.y };
        outTargetTile = { waypoint.x, waypoint.y, region.baseZ };
        return true;
    }

    bool SteerSegmentStaysInRegion(
        const Region& region, const CoordsXY& fromWorld, const CoordsXY& toWorld, const PlazaGrid::Tile& extraAllowed)
    {
        if (_dirty || _regionByTile.empty())
            return false;
        return PlazaGrid::StaircaseStaysIn(
            _regionByTile, _cacheMapSize.x, _cacheMapSize.y, region.id, { fromWorld.x, fromWorld.y }, { toWorld.x, toWorld.y },
            extraAllowed, PlazaGrid::kWalkTolerance);
    }

    void LogPlazaDiagnostics()
    {
        EnsurePlazaRegions();
        size_t totalTiles = 0;
        size_t totalDoors = 0;
        for (const auto& region : _regions)
        {
            totalTiles += region.tiles.size();
            totalDoors += region.doors.size();
        }
        LOG_INFO(
            "PlazaRegions: %zu region(s), %zu plaza tile(s), %zu door(s) on a %d x %d map", _regions.size(), totalTiles,
            totalDoors, _cacheMapSize.x, _cacheMapSize.y);
        for (const auto& region : _regions)
        {
            LOG_INFO(
                "PlazaRegions: region %u z=%d tiles=%zu doors=%zu steerable=%s", region.id, region.baseZ, region.tiles.size(),
                region.doors.size(), region.steerable() ? "yes" : "no");
        }
    }
} // namespace OpenRCT2::Plaza
