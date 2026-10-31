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

    // Opening gate: start every assigned worker's trip to this ride now.
    // Summoning never assigns anyone - assignments are manual only, made
    // through the staff-window dropdown or the ride-window button.
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

    // Break rooms (Workers): a break room is a ride of the cloned type
    // RIDE_TYPE_BREAK_ROOM. This classification is the one place break-room
    // logic keys off: the cloned objects bind to the type, and the type
    // carries no guest interaction (see rtd/shops/BreakRoom.h), so guests
    // can never target a room - only workers travel to one, through the
    // worker machinery in Staff.cpp.
    bool IsBreakRoom(const Ride& ride);

    // How many workers may rest in one room at once (D3, as amended).
    constexpr int32_t kBreakRoomCapacity = 20;

    // The break room whose facility tile contains loc, or nullptr when the
    // tile holds no break room. A resting worker's room is always derived
    // this way from the tile under him, never stored on the worker.
    Ride* GetBreakRoomAtTile(const CoordsXY& loc);

    // Workers currently resting inside this room (state onBreak, derived
    // room == this room). Counted on demand by scanning the staff list, so
    // the count can never drift from reality across pickups, firings and
    // demolitions - there is no stored counter to keep in sync.
    int32_t CountWorkersOnBreak(const Ride& room);

    // Where an idle worker's break trip should take him: his home room
    // (Staff::breakRoom) when it is still a break room, open, and below
    // capacity; otherwise the nearest open break room with capacity,
    // measured from the worker. nullptr when no room can take him.
    Ride* FindBreakRoomTripTarget(Staff& worker);

    // Restock: stall stock and the runs that refill it. A stall's stock
    // (Ride::shopStock) decrements per completed sale while staffing is
    // engaged; at the trigger level the stall's second at-post worker -
    // the runner - walks to fetch more, and the stall refills when he
    // returns to post. The run machinery itself lives in Staff.cpp, like
    // the break trips; the rules that other systems consult live here.

    // True when the stall is staffed and open but has nothing to sell:
    // staffing is engaged and the stock counter has reached zero. Guests
    // refuse a sold-out stall at the same two points as a shut one
    // (Guest::shouldGoOnRide and PeepInteractWithShop).
    bool IsStallSoldOut(const Ride& ride);

    // One completed stall sale, reported by Guest::updateBuying beside
    // the items-sold increments so the two stay in lockstep: the stock
    // counter decrements, floored at zero. An unengaged park never
    // decrements - with no workers there is no runner, and a stock
    // counter that drained with nobody able to refill it would deadlock
    // every stall sold out; not draining is part of behaving exactly
    // like stock until the player opts in.
    void RecordStallSale(Ride& ride);

    // How much stock a stall may fall to before it sends its runner.
    constexpr int32_t kRestockTriggerLevel = 25;

    // After a restock run is forgotten (source unreachable or demolished,
    // assignment pulled, worker picked up), this long must pass before
    // the worker may be dispatched again - the break-room grace cadence,
    // reused so a stall with a broken source retries politely instead of
    // re-dispatching every think.
    constexpr uint32_t kRestockRetryTicks = 1000;

    // True while any worker assigned to this ride is out on a restock run
    // for it (any phase). Counted on demand by scanning the staff list,
    // like CountWorkersOnBreak, so the answer can never drift from
    // reality across pickups, firings and demolitions.
    bool IsRestockInFlight(const Ride& ride);

    // Stockrooms (Restock): a stockroom is a ride of the cloned type
    // RIDE_TYPE_STOCKROOM - an unstaffed supply building whose only
    // visitors are workers on restock runs (see rtd/shops/Stockroom.h).
    // This classification is the one place stockroom logic keys off,
    // exactly like break rooms above.
    bool IsStockroom(const Ride& ride);

    // The stockroom whose facility tile contains loc, or nullptr when the
    // tile holds no stockroom. A loading runner's room is always derived
    // this way from the tile under him, never stored on the worker.
    Ride* GetStockroomAtTile(const CoordsXY& loc);

    // Where a restock run should fetch from: the nearest open stockroom,
    // measured from the worker (who at dispatch stands at his stall, so
    // this is the stall's nearest too). nullptr when the park has no
    // open stockroom - and then no run is dispatched at all: restock
    // is reliant on stockrooms (Franklin's ruling, 2026-10-30), so the
    // stall keeps its stock level until one is built and opens. No
    // capacity limit: a stockroom takes every runner its stalls send.
    Ride* FindStockroomTripTarget(Staff& worker);

} // namespace OpenRCT2::RideStaffing
