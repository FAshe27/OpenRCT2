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

    // Lane changes (personal fork): the movement-layer companion to the
    // field. GuestFieldTryStep can only ever rank strictly-downhill
    // steps, so on a straight multi-wide path a goal-directed guest's
    // column is conserved from entry - a lateral step is exactly
    // equal-distance and never survives the downhill filter. Called as
    // the first line of PeepMoveOneTile with the direction a chooser
    // picked, this may substitute a sideways step for a straight one
    // when the tile ahead is blocked by a slower or stopped guest, or
    // clearly more crowded than the tile beside, subject to a chain of
    // gates (walking guests going straight only, a hash-gated cooldown
    // window, the movement graph's own step test, and for field-routed
    // guests an equal-distance-only rule that preserves the field's
    // strict-decrease arrival proof). Returns the direction unchanged
    // unless every gate passes. Draws no ScenarioRand and keeps no
    // state: every input is a deterministic function of game state.
    Direction GuestSpreadTryLaneChange(Direction direction, Peep& peep);

} // namespace OpenRCT2::PathFinding
