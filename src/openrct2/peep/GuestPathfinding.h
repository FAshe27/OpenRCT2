/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#pragma once

#include "../Identifiers.h"

using Direction = uint8_t;
struct TileCoordsXYZ;

namespace OpenRCT2
{
    struct Guest;
    struct Peep;
} // namespace OpenRCT2

namespace OpenRCT2::PathFinding
{
    Direction ChooseDirection(
        const TileCoordsXYZ& loc, const TileCoordsXYZ& goal, Peep& peep, bool ignoreForeignQueues, RideId queueRideIndex);

    // The signed within-tile lane (-8..+8) that the lane spread in
    // PeepMoveOneTile applies for a guest with this peepId travelling in
    // this direction: positive leans towards direction 1 on X-travel and
    // towards direction 2 on Y-travel. Exposed so other per-guest
    // preferences - the corridor step scoring in the distance-field
    // routing - can agree with the lane the guest will actually walk,
    // instead of deriving a second, divergent value.
    int32_t GuestSpreadSignedLane(uint32_t peepId, Direction direction);

    int32_t CalculateNextDestination(Guest& peep);

    int32_t GuestPathFindParkEntranceEntering(Peep& peep, uint8_t edges);

    int32_t GuestPathFindPeepSpawn(Peep& peep, uint8_t edges);

    int32_t GuestPathFindParkEntranceLeaving(Peep& peep, uint8_t edges);

} // namespace OpenRCT2::PathFinding
