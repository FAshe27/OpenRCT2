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
    // laneRight is the guest's own lane preference in PeepMoveOneTile's
    // convention (positive = keep-right side of travel), used by the region
    // lateral spread: inside the region the guest follows a parallel line
    // offset from the shared route by laneRight scaled with the local
    // cross-section width of the region (see PlazaGrid::LateralLineAt). The
    // offset fades to zero where the region narrows, and every offset
    // destination is containment-validated before it is returned.
    bool ComputeSteerDestination(
        const Region& region, const CoordsXY& fromWorld, const TileCoordsXYZ& start, const TileCoordsXYZ& target,
        bool targetIsDoorLanding, int32_t laneRight, CoordsXY& outDestination, TileCoordsXYZ& outTargetTile);

    // True when every staircase the dominant-axis walker would take from
    // fromWorld to toWorld stays on tiles of the region (plus extraAllowed,
    // for a steering destination on a door-landing tile). Exposed for the
    // following-v2 passing offset in GuestPathfinding.cpp, which shifts a
    // steering destination sideways around a slower guest and must prove the
    // shifted segment is still a legal in-region walk. Returns false when
    // the region cache is not built.
    bool SteerSegmentStaysInRegion(
        const Region& region, const CoordsXY& fromWorld, const CoordsXY& toWorld, const PlazaGrid::Tile& extraAllowed);

    // M1 diagnostics: logs a one-line summary per region (LOG_INFO).
    void LogPlazaDiagnostics();

    // ---- Handoff-fix steering watch (personal fork; DIAGNOSIS items 1-2) ----
    //
    // A runtime-only side table beside the region cache above. It holds
    // no peep state of its own and is never saved, serialised or
    // networked: it only remembers, per peep id, what the steering system
    // itself just did (which door landing it delivered the guest to, and
    // how much net progress recent steering decisions bought). Like the
    // cache it is cleared wholesale by InvalidatePlazaRegions()
    // (footpath / entrance actions and park load), and every entry is
    // additionally bounded by the tick windows below, so a stale entry -
    // e.g. one left behind by a despawned guest whose id is later
    // reused - can at worst decline steering for its new owner for a
    // bounded while. All inputs are synced game state (ticks, tiles,
    // peep ids): the table is fully deterministic and draws no
    // ScenarioRand.

    // Re-capture guard: after steering has delivered a goal-directed
    // guest out of a region (NotePlazaSteerExit) and the guest has been
    // observed back inside the SAME region with the SAME goal, steering
    // declines while both bounds hold, so stock pathfinding keeps
    // uncontested control of the onward route from the landing.
    inline constexpr uint32_t kPlazaSteerGuardWindowTicks = 1200;
    inline constexpr int32_t kPlazaSteerGuardRadiusTiles = 4;

    // Progress watchdog: a goal-directed guest whose steering decisions
    // (NotePlazaSteerDecision calls) reach kPlazaWatchMinDecisions
    // within one window with at most kPlazaWatchMaxNetTiles of net
    // displacement from the window anchor and no improvement in goal
    // distance (beyond kPlazaWatchGoalSlackTiles) is looping: the caller
    // force-declines steering, resets the guest's stock plan, and the
    // table suppresses steering for kPlazaWatchCooldownTicks while stock
    // re-plans.
    inline constexpr uint32_t kPlazaWatchWindowTicks = 8192;
    inline constexpr uint32_t kPlazaWatchMinDecisions = 8;
    inline constexpr int32_t kPlazaWatchMaxNetTiles = 2;
    inline constexpr int32_t kPlazaWatchGoalSlackTiles = 1;
    inline constexpr uint32_t kPlazaWatchCooldownTicks = 2048;

    // Records that steering has just issued the final-hop destination
    // onto a door landing for a goal-directed guest: (regionId, goal) is
    // the pairing the re-capture guard protects once the guest has been
    // observed outside the region. Intermediate waypoints toward the
    // door must NOT be recorded - only the delivery itself.
    void NotePlazaSteerExit(uint32_t peepId, uint16_t regionId, const TileCoordsXYZ& goal, const TileCoordsXY& landing);

    // Feeds the guard's delivery observation: called on every steering
    // consultation for a guest, with the region the guest currently
    // stands in (0 when not on a region tile). Marks a pending exit as
    // observed once the guest is seen outside the exited region.
    void ObservePlazaSteerGuest(uint32_t peepId, uint16_t currentRegionId, uint32_t tick);

    // True when the re-capture guard declines steering for this
    // guest/region/goal pairing. A lapse (window or radius exceeded, or
    // the goal changed) clears the exit mark as a side effect.
    bool PlazaSteerGuardDeclines(
        uint32_t peepId, uint16_t regionId, const TileCoordsXYZ& current, const TileCoordsXYZ& goal, uint32_t tick);

    // True while the watchdog's post-fire cooldown suppresses steering
    // for this guest.
    bool PlazaSteerCoolingDown(uint32_t peepId, uint32_t tick);

    // Accounts one steering decision for a goal-directed guest standing
    // on a region tile. Returns true exactly when the watchdog fires:
    // the caller must decline steering and give the guest a fresh stock
    // plan (Peep::resetPathfindGoal); the cooldown above then keeps
    // steering off the guest while stock re-plans.
    bool NotePlazaSteerDecision(
        uint32_t peepId, uint16_t regionId, const TileCoordsXYZ& current, const TileCoordsXYZ& goal, uint32_t tick);
} // namespace OpenRCT2::Plaza
