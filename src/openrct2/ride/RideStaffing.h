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

namespace OpenRCT2
{
    struct Staff;
} // namespace OpenRCT2

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

    // The staff-list auto-assign toggle (Workers tab), under its redefined
    // contract: its ONLY effect is at hire time (AssignNewWorker below). It
    // never assigns an existing worker, never tops a crew up, and does not
    // gate dispatch - assigned workers are sent to their posts by the staff
    // think regardless of it. Enabled unless the player turned it off for
    // this park; the flag is stored inverted (ParkFlag::
    // workersAutoAssignDisabled - a historical name, kept because the flag
    // is serialised) so pre-toggle saves read as enabled.
    bool IsAutoAssignNewWorkersEnabled();

    // Hire-time self-assignment, called by StaffHireNewAction for a fresh
    // Worker while IsAutoAssignNewWorkersEnabled(): the worker takes the
    // single worst-staffed post that still has a shortfall, in
    // AutoAssignAll's selection order (shut stalls first, then lowest
    // staffing fraction, then lowest ride id), over the hire-eligible set:
    // rides that are open or testing, plus closed rides with a pending
    // open request. A plain closed ride is NOT eligible - it would only
    // soak the hire until somebody opens it. One hire is at most one
    // assignment; with no eligible shortfall the worker simply spawns
    // unassigned. Returns whether an assignment was made. Deterministic;
    // draws no ScenarioRand.
    bool AssignNewWorker(Staff& worker);

    // Opening gate: start every assigned worker's trip to this ride now.
    // Never assigns anyone, in either toggle state - hire-time is the only
    // system assignment, everything after that is manual.
    void SummonWorkers(Ride& ride);

    // True when this ride's operators are drawn standing at their post.
    // Only coaster crews are visible in the station; every other class
    // works out of sight (stall operators behind the counter, flat-ride
    // and tracked non-coaster crews likewise). Facilities require no
    // operators and never have anyone at post. This function is the one
    // place the class test lives - the paint pass asks only this.
    bool AreOperatorsVisibleAtPost(const Ride& ride);

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
    // Retained for StaffAutoAssignAction, which no UI invokes any more; the
    // hire path shares the selection order through AssignNewWorker.
    int32_t AutoAssignAll();
} // namespace OpenRCT2::RideStaffing
