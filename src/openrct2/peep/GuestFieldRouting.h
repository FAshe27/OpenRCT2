/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#pragma once

#include <cstdint>

using Direction = uint8_t;
struct TileCoordsXYZ;

namespace OpenRCT2
{
    struct Peep;
} // namespace OpenRCT2

namespace OpenRCT2::PathFinding
{
    /**
     * Distance-field routing (personal fork).
     *
     * For a fixed guest destination (a park entrance, a ride entrance or
     * queue tail, a shop, a peep spawn) one reverse breadth-first search over
     * the movement graph paints every reachable path node with its exact
     * distance to the destination. The field is shared by every guest
     * heading to that destination, is runtime-only (nothing is persisted),
     * and is rebuilt lazily after path-network edits, at most a small
     * number of builds per tick. Guests only ever read one number - their
     * current node's distance - so the guest-side cost is O(1) per
     * pathfinding consultation and does not scale with the guest count.
     *
     * The field stores distances only, never directions: a guest may step
     * to any connected neighbour whose distance is strictly lower than
     * their current node's, choosing among those by a deterministic
     * per-guest preference. Strict decrease guarantees arrival and makes
     * loops impossible; unreachability is known before the guest sets out.
     */

    // Marks every cached field stale. Called from the game actions that can
    // change the path network; the rebuild itself is lazy and budgeted.
    void InvalidateGuestFields();

    // Drops every cached field outright. Called on park load.
    void ClearGuestFields();

    // Attempts one field-routed step for a goal-directed guest standing on
    // loc and heading for goal. permittedEdges is the current node's
    // banner-cleared edge mask, as computed by ChooseDirection. On success
    // returns true and stores the step direction in outDirection; on any
    // doubt (no field yet, field over the cap, guest not on a routed node,
    // already on the destination's doorstep) returns false and the caller
    // runs the stock heuristic search unchanged.
    bool GuestFieldTryStep(
        Peep& peep, const TileCoordsXYZ& loc, const TileCoordsXYZ& goal, uint8_t permittedEdges, Direction& outDirection);

    // Wide path lanes (personal fork): the field distance from node to goal, when goal's field is available and covers
    // node. Read-only, like GuestFieldTryStep: no peep state is written and no ScenarioRand is drawn. May build the
    // field, within the usual per-tick build budget.
    bool GuestFieldDistance(const TileCoordsXYZ& goal, const TileCoordsXYZ& node, uint32_t& outDistance);

} // namespace OpenRCT2::PathFinding
