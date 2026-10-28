/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#pragma once

#include "Ride.h"

namespace OpenRCT2::RideStaffing
{
    // Operator staffing for rides and stalls (the Worker staff type).
    //
    // Requirements are a pure function of the ride type descriptor, derived at
    // runtime and never stored: roller coasters need 4 operators, every other
    // ride (flat rides, transport, water, go karts, mazes, ...) needs 2, stalls
    // need 1, and facilities (toilets, first aid, cash machines, info kiosks)
    // need none. Multi-station coasters are a flat 4 per ride in v1.
    //
    // Only operators standing at their post (PeepState::operating) count.
    // All staffing consequences are inert until the feature engages: see IsEngaged().

    struct Staffing
    {
        int32_t required = 0;
        int32_t atPost = 0;
        int32_t assigned = 0;

        // Fraction of the requirement met by operators at post, in (0, 1].
        // Defined as 1 when nothing is required.
        constexpr float fraction() const
        {
            if (required <= 0)
                return 1.0f;
            return static_cast<float>(atPost) / static_cast<float>(required);
        }

        constexpr bool fullyStaffed() const
        {
            return atPost >= required;
        }
    };

    int32_t GetRequiredOperators(const Ride& ride);
    Staffing GetStaffing(const Ride& ride);
    int32_t CountOperatorsAtPost(const Ride& ride);
    int32_t CountAssignedOperators(const Ride& ride);

    // The legacy-load guard: staffing consequences only engage once the player has
    // hired their first Worker (StaffHireNewAction sets ParkFlag::workersEngaged
    // permanently), or while any Worker exists in the park (covers workers placed
    // by the scenario editor or cheats). A park loaded with zero workers hired
    // therefore behaves exactly like stock until the player opts in.
    bool IsEngaged();

    // True when the ride's trains may not depart / guests may not board because
    // staffing is engaged, the ride requires operators, and none are at post.
    // Applies to Testing as well as Open (testing needs crew on site too);
    // closed and broken-down rides are handled by stock logic. For Testing
    // only this zero-operator floor applies - capacity scaling and dwell
    // stretch remain open-only.
    bool IsDispatchBlocked(const Ride& ride);

    // The opening gate's top-up switch (staff list, Workers tab). Enabled
    // unless the player turned it off for this park; the flag is stored
    // inverted so pre-toggle saves read as enabled.
    bool IsAutoAssignOnOpenEnabled();

    // Opening gate: start every assigned worker's trip to this ride now, and
    // - only while IsAutoAssignOnOpenEnabled() - top the crew up to the
    // requirement from the unassigned pool (nearest first, ties to the lowest
    // entity id, picked-up workers skipped; workers assigned to other rides
    // are never stolen). Summoning assigned workers happens under both
    // toggle states.
    void SummonWorkers(Ride& ride);

    // True when the stall is shut because staffing is engaged and no operator is
    // at post. Facilities never require operators and are never shut by this.
    bool IsStallClosed(const Ride& ride);

    // True when staffing is engaged, the ride requires operators, and none
    // are at post. Guests use this at the ride-choice point: such a ride
    // draws no new guests, in any state (an open ride whose crew fell to
    // zero mid-operation included). Partially crewed rides return false.
    bool HasZeroOperatorsAtPost(const Ride& ride);

    // Capacity scaling for tracked (non-flat) rides: at most this many trains may
    // be out on the circuit at once. Returns the ride's full train count when
    // staffing does not constrain it.
    int32_t GetCirculatingTrainCap(const Ride& ride);

    // Dispatch dwell stretch for understaffed rides: the base waiting time scaled
    // by required / atPost (rounded up), so a half-crewed ride dispatches half as
    // often. Returns the base value unchanged when staffing does not constrain it.
    int32_t ScaleDwellTicks(const Ride& ride, int32_t baseTicks);

    // Assigns every unassigned worker to the worst-staffed post, worst first
    // (closed stalls before rides, then lowest staffing fraction, then lowest
    // ride id; the nearest unassigned worker wins each post). One-shot: workers
    // are never pulled off an existing assignment. Returns how many were assigned.
    int32_t AutoAssignAll();
} // namespace OpenRCT2::RideStaffing
