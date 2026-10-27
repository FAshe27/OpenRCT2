/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#pragma once

#include "../world/Location.hpp"
#include "PlazaGrid.h"

#include <cstdint>
#include <vector>

// V2 plaza walking (personal fork): the runtime-only plaza region cache.
//
// A plaza region is a connected component of flat, non-queue, non-ghost
// path tiles that each belong to at least one 2x2 block of such tiles at
// the same height (see PlazaGrid.h and the v2 design, section 2). The cache
// is a pure derived view of tile data: it is never saved, serialised or
// networked, it holds no peep state, and rebuilding it is deterministic
// (region ids are assigned in scan order). It is rebuilt lazily on first
// use after InvalidatePlazaRegions() has been called (footpath / entrance
// game actions and park load paths call it; steering also self-heals by
// revalidating tiles at steer time and invalidating on mismatch).

namespace OpenRCT2::Plaza
{
    // Steering is disabled for regions larger than this (design section
    // 2.2): BFS / string-pulling cost stays bounded. Oversized regions
    // still get ids and appear in diagnostics.
    inline constexpr int32_t kPlazaMaxSteerTiles = 2048;

    // At most this many doors per region are scored with the stock
    // heuristic search when choosing an exit (design section 1.3).
    inline constexpr size_t kPlazaMaxScoredDoors = 12;

    struct Region
    {
        uint16_t id{};   // 1-based; 0 means "no region" everywhere in this API
        int32_t baseZ{}; // path-element baseHeight units, shared by all tiles
        std::vector<TileCoordsXY> tiles;
        std::vector<PlazaGrid::Door> doors;

        bool steerable() const
        {
            return static_cast<int32_t>(tiles.size()) <= kPlazaMaxSteerTiles;
        }
    };

    // Marks the cache dirty. Cheap; safe to call from any game action.
    void InvalidatePlazaRegions();

    // Builds the cache if dirty. Returns false when there is no usable map
    // (zero size), in which case all lookups report "no region".
    bool EnsurePlazaRegions();

    bool PlazaRegionsDirty();

    // Map size the cache was built for (0 when not built).
    TileCoordsXY PlazaCacheMapSize();

    // Region id at a tile location (z in path-element baseHeight units),
    // or 0 when the tile is not in a region at that height. Builds the
    // cache on first use.
    uint16_t GetRegionIdAt(const TileCoordsXYZ& loc);

    // Region by id, or nullptr. The pointer is invalidated by a rebuild.
    const Region* FindRegion(uint16_t regionId);

    // All regions (for diagnostics). Empty until built.
    const std::vector<Region>& AllRegions();

    // Self-healing revalidation used at steer time: true iff the tile at
    // loc currently holds a flat, non-queue, non-ghost path element at
    // loc.z AND the cache still assigns it regionId. On mismatch the cache
    // is invalidated (the caller should decline to stock behaviour).
    bool RevalidateRegionTile(const TileCoordsXYZ& loc, uint16_t regionId);

    // Computes the next steering destination inside a region.
    //
    // fromWorld: the peep's current world position (the staircase anchor).
    // start:     the peep's current region tile.
    // target:    the tile to head for. When targetIsDoorLanding is false it
    //            must be a tile of the region itself (goal-in-region). When
    //            true it must be the landing tile of one of the region's
    //            doors; the in-region path then ends at that door's inside
    //            tile and the final hop aims at the landing tile centre.
    //
    // On success returns true and sets outDestination (world coords of the
    // farthest string-pulled waypoint centre, or the landing/goal centre
    // when the pulled path completes) and outTargetTile (the tile that
    // destination belongs to). Every staircase the walker will take toward
    // outDestination stays on region tiles (plus the landing tile for the
    // final hop) - this is proven by simulating the walker, see
    // PlazaGrid::StaircaseStaysIn. Returns false when no in-region path
    // exists; the caller must fall back to stock behaviour.
    bool ComputeSteerDestination(
        const Region& region, const CoordsXY& fromWorld, const TileCoordsXYZ& start, const TileCoordsXYZ& target,
        bool targetIsDoorLanding, CoordsXY& outDestination, TileCoordsXYZ& outTargetTile);

    // M1 diagnostics: logs a one-line summary per region (LOG_INFO).
    void LogPlazaDiagnostics();
} // namespace OpenRCT2::Plaza
