/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "RideStaffing.h"

#include "../GameState.h"
#include "../entity/EntityList.h"
#include "../entity/Staff.h"
#include "../world/Map.h"
#include "../world/ParkData.h"
#include "RideData.h"
#include "RideManager.hpp"
#include "Vehicle.h"

#include <algorithm>
#include <vector>

namespace OpenRCT2::RideStaffing
{
    int32_t GetRequiredOperators(const Ride& ride)
    {
        const auto& rtd = ride.getRideTypeDescriptor();
        switch (rtd.Classification)
        {
            case RideClassification::kioskOrFacility:
                // Toilets, first aid, cash machines and information kiosks run unattended.
                return 0;
            case RideClassification::shopOrStall:
                return 1;
            case RideClassification::ride:
                break;
        }

        // Tracked rides and flat rides alike: coasters are crew-heavy, the rest are not.
        // Multi-station coasters are a flat 4 per ride in v1.
        if (rtd.Category == RideCategory::rollerCoaster)
            return 4;
        return 2;
    }

    bool AreOperatorsVisibleAtPost(const Ride& ride)
    {
        const auto& rtd = ride.getRideTypeDescriptor();
        return rtd.Classification == RideClassification::ride && rtd.Category == RideCategory::rollerCoaster;
    }

    int32_t CountOperatorsAtPost(const Ride& ride)
    {
        int32_t count = 0;
        for (auto* staff : EntityList<Staff>())
        {
            if (staff->isWorker() && staff->currentRide == ride.id && staff->state == PeepState::operating)
                count++;
        }
        return count;
    }

    int32_t CountAssignedOperators(const Ride& ride)
    {
        int32_t count = 0;
        for (auto* staff : EntityList<Staff>())
        {
            if (staff->isWorker() && staff->currentRide == ride.id)
                count++;
        }
        return count;
    }

    Staffing GetStaffing(const Ride& ride)
    {
        Staffing result;
        result.required = GetRequiredOperators(ride);
        if (result.required <= 0)
            return result;
        result.atPost = CountOperatorsAtPost(ride);
        result.assigned = CountAssignedOperators(ride);
        return result;
    }

    bool IsEngaged()
    {
        const auto& park = getGameState().park;
        if (park.flags.has(ParkFlag::workersEngaged))
            return true;

        // Workers can also appear without a hire action (scenario editor, cheats).
        for (auto* staff : EntityList<Staff>())
        {
            if (staff->isWorker())
                return true;
        }
        return false;
    }

    bool IsDispatchBlocked(const Ride& ride)
    {
        // Open and Testing alike: testing needs crew on site too (this
        // supersedes v1's testing exemption). Only the zero-operator floor
        // applies to Testing; capacity scaling and dwell stretch below keep
        // their own testing early-outs.
        if (ride.status != RideStatus::open && ride.status != RideStatus::testing)
            return false;
        const auto staffing = GetStaffing(ride);
        if (staffing.required <= 0 || staffing.atPost > 0)
            return false;
        return IsEngaged();
    }

    bool IsStallClosed(const Ride& ride)
    {
        const auto& rtd = ride.getRideTypeDescriptor();
        if (rtd.Classification != RideClassification::shopOrStall)
            return false;
        if (GetRequiredOperators(ride) <= 0)
            return false;
        if (CountOperatorsAtPost(ride) > 0)
            return false;
        return IsEngaged();
    }

    bool HasZeroOperatorsAtPost(const Ride& ride)
    {
        if (GetRequiredOperators(ride) <= 0)
            return false;
        if (CountOperatorsAtPost(ride) > 0)
            return false;
        return IsEngaged();
    }

    int32_t GetCirculatingTrainCap(const Ride& ride)
    {
        const int32_t numTrains = ride.numTrains;
        if (numTrains <= 1)
            return numTrains;

        const auto& rtd = ride.getRideTypeDescriptor();
        if (rtd.Classification != RideClassification::ride || rtd.flags.has(RtdFlag::isFlatRide))
            return numTrains;
        if (ride.status == RideStatus::testing)
            return numTrains;

        const auto staffing = GetStaffing(ride);
        if (staffing.required <= 0 || staffing.fullyStaffed())
            return numTrains;
        if (!IsEngaged())
            return numTrains;
        if (staffing.atPost <= 0)
            return 0;

        // Floor of the staffed share of the fleet, but never below one train out.
        return std::max(1, numTrains * staffing.atPost / staffing.required);
    }

    int32_t ScaleDwellTicks(const Ride& ride, int32_t baseTicks)
    {
        if (ride.status == RideStatus::testing)
            return baseTicks;

        const auto staffing = GetStaffing(ride);
        if (staffing.required <= 0 || staffing.fullyStaffed() || staffing.atPost <= 0)
            return baseTicks;
        if (!IsEngaged())
            return baseTicks;

        // Dwell stretches by the inverse of the staffing fraction, capped by the
        // 7-bit station depart counter.
        const int32_t scaled = (baseTicks * staffing.required + staffing.atPost - 1) / staffing.atPost;
        return std::min(scaled, 127);
    }

    void SummonWorkers(Ride& ride)
    {
        const int32_t required = GetRequiredOperators(ride);
        if (required <= 0)
            return;

        // Assigned crew start their trip now: summoning an assigned worker
        // is their assignment taking effect, not an assignment. Nobody is
        // assigned here - assignments are manual only, and this loop only
        // walks workers who already hold one. (Crew already travelling or
        // at post are left be.) An assigned worker resting in a break room
        // comes out first: the summon is an exit trigger, and the trip to
        // post starts from the room's frontage tile like any other.
        for (auto* staff : EntityList<Staff>())
        {
            if (!staff->isWorker() || staff->currentRide != ride.id)
                continue;

            if (staff->state == PeepState::patrolling || staff->state == PeepState::falling)
            {
                staff->workerBeginHeadingToPost();
            }
            else if (staff->state == PeepState::onBreak)
            {
                staff->workerLeaveBreakRoom();
                staff->workerBeginHeadingToPost();
            }
        }
    }

    bool IsBreakRoom(const Ride& ride)
    {
        return ride.type == RIDE_TYPE_BREAK_ROOM;
    }

    Ride* GetBreakRoomAtTile(const CoordsXY& loc)
    {
        auto* tileElement = MapGetFirstElementAt(loc);
        if (tileElement == nullptr)
            return nullptr;

        do
        {
            if (tileElement->getType() == TileElementType::track)
            {
                auto* ride = GetRide(tileElement->asTrack()->getRideIndex());
                if (ride != nullptr && IsBreakRoom(*ride))
                    return ride;
            }
        } while (!(tileElement++)->isLastForTile());

        return nullptr;
    }

    int32_t CountWorkersOnBreak(const Ride& room)
    {
        int32_t count = 0;
        for (auto* staff : EntityList<Staff>())
        {
            if (staff->isWorker() && staff->state == PeepState::onBreak
                && GetBreakRoomAtTile(CoordsXY{ staff->x, staff->y }) == &room)
            {
                count++;
            }
        }
        return count;
    }

    Ride* FindBreakRoomTripTarget(Staff& worker)
    {
        const auto hasSpace = [](const Ride& room) { return CountWorkersOnBreak(room) < kBreakRoomCapacity; };

        if (auto* home = GetRide(worker.breakRoom);
            home != nullptr && IsBreakRoom(*home) && home->status != RideStatus::closed && hasSpace(*home))
        {
            return home;
        }

        Ride* best = nullptr;
        int64_t bestDistance = 0;
        for (auto& ride : RideManager(getGameState()))
        {
            if (!IsBreakRoom(ride) || ride.status == RideStatus::closed || !hasSpace(ride))
                continue;

            const auto stationStart = ride.getStation().getStart();
            const int64_t dx = stationStart.x - worker.x;
            const int64_t dy = stationStart.y - worker.y;
            const int64_t distance = dx * dx + dy * dy;
            if (best == nullptr || distance < bestDistance)
            {
                best = &ride;
                bestDistance = distance;
            }
        }
        return best;
    }

    bool IsStallSoldOut(const Ride& ride)
    {
        const auto& rtd = ride.getRideTypeDescriptor();
        if (rtd.Classification != RideClassification::shopOrStall)
            return false;
        if (GetRequiredOperators(ride) <= 0)
            return false;
        if (ride.shopStock > 0)
            return false;
        return IsEngaged();
    }

    void RecordStallSale(Ride& ride)
    {
        if (ride.getRideTypeDescriptor().Classification != RideClassification::shopOrStall)
            return;
        if (ride.shopStock == 0)
            return;
        if (!IsEngaged())
            return;
        ride.shopStock--;
    }

    bool IsRestockInFlight(const Ride& ride)
    {
        for (auto* staff : EntityList<Staff>())
        {
            if (staff->isWorker() && staff->currentRide == ride.id && staff->restockPhase != 0)
                return true;
        }
        return false;
    }

    bool IsStockroom(const Ride& ride)
    {
        return ride.type == RIDE_TYPE_STOCKROOM;
    }

    Ride* GetStockroomAtTile(const CoordsXY& loc)
    {
        auto* tileElement = MapGetFirstElementAt(loc);
        if (tileElement == nullptr)
            return nullptr;

        do
        {
            if (tileElement->getType() == TileElementType::track)
            {
                auto* ride = GetRide(tileElement->asTrack()->getRideIndex());
                if (ride != nullptr && IsStockroom(*ride))
                    return ride;
            }
        } while (!(tileElement++)->isLastForTile());

        return nullptr;
    }

    Ride* FindStockroomTripTarget(Staff& worker)
    {
        Ride* best = nullptr;
        int64_t bestDistance = 0;
        for (auto& ride : RideManager(getGameState()))
        {
            if (!IsStockroom(ride) || ride.status == RideStatus::closed)
                continue;

            const auto stationStart = ride.getStation().getStart();
            const int64_t dx = stationStart.x - worker.x;
            const int64_t dy = stationStart.y - worker.y;
            const int64_t distance = dx * dx + dy * dy;
            if (best == nullptr || distance < bestDistance)
            {
                best = &ride;
                bestDistance = distance;
            }
        }
        return best;
    }
} // namespace OpenRCT2::RideStaffing
