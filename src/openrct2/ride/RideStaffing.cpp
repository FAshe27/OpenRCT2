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

    bool IsAutoAssignOnOpenEnabled()
    {
        // Stored inverted (bit set = disabled) so parks saved before the
        // toggle existed read as enabled.
        return !getGameState().park.flags.has(ParkFlag::workersAutoAssignDisabled);
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

    namespace
    {
        CoordsXY GetPostLocation(const Ride& ride)
        {
            const auto& rtd = ride.getRideTypeDescriptor();
            if (rtd.Classification == RideClassification::shopOrStall)
            {
                return TileCoordsXY(ride.getStation().start).toCoordsXY();
            }

            const auto stations = ride.getStations();
            for (size_t i = 0; i < stations.size(); i++)
            {
                const auto& station = ride.getStation(StationIndex::FromUnderlying(static_cast<uint8_t>(i)));
                if (!station.exit.isNull())
                    return TileCoordsXY(station.exit).toCoordsXY();
                if (!station.entrance.isNull())
                    return TileCoordsXY(station.entrance).toCoordsXY();
            }
            return ride.overallView;
        }

        int64_t DistanceSquared(const CoordsXY& a, const CoordsXY& b)
        {
            const int64_t dx = static_cast<int64_t>(a.x) - b.x;
            const int64_t dy = static_cast<int64_t>(a.y) - b.y;
            return dx * dx + dy * dy;
        }
    } // namespace

    int32_t AutoAssignAll()
    {
        struct Target
        {
            RideId rideId;
            bool isStall;
            float fraction;
            CoordsXY postLocation;
        };

        int32_t assignedCount = 0;
        while (true)
        {
            // Recompute targets each round: assignments change the fractions.
            std::vector<Target> targets;
            for (auto& ride : RideManager(getGameState()))
            {
                if (ride.status == RideStatus::closed)
                    continue;
                const auto staffing = GetStaffing(ride);
                if (staffing.required <= 0 || staffing.assigned >= staffing.required)
                    continue;
                targets.push_back(
                    { ride.id, ride.getRideTypeDescriptor().Classification == RideClassification::shopOrStall,
                      staffing.fraction(), GetPostLocation(ride) });
            }
            if (targets.empty())
                break;

            // Worst first: shut stalls before rides, then lowest staffing fraction,
            // then lowest ride id (stable, deterministic).
            std::sort(targets.begin(), targets.end(), [](const Target& a, const Target& b) {
                if (a.isStall != b.isStall)
                    return a.isStall > b.isStall;
                if (a.fraction != b.fraction)
                    return a.fraction < b.fraction;
                return a.rideId.ToUnderlying() < b.rideId.ToUnderlying();
            });
            const auto& target = targets.front();

            // The nearest unassigned worker wins the post; ties go to the lowest entity id.
            Staff* bestWorker = nullptr;
            int64_t bestDistance = 0;
            for (auto* staff : EntityList<Staff>())
            {
                if (!staff->isWorker() || !staff->currentRide.IsNull())
                    continue;
                if (staff->state == PeepState::picked)
                    continue;
                const int64_t distance = DistanceSquared({ staff->x, staff->y }, target.postLocation);
                if (bestWorker == nullptr || distance < bestDistance
                    || (distance == bestDistance && staff->id.ToUnderlying() < bestWorker->id.ToUnderlying()))
                {
                    bestWorker = staff;
                    bestDistance = distance;
                }
            }
            if (bestWorker == nullptr)
                break;

            bestWorker->setWorkerAssignment(target.rideId);
            assignedCount++;
        }
        return assignedCount;
    }

    void SummonWorkers(Ride& ride)
    {
        const int32_t required = GetRequiredOperators(ride);
        if (required <= 0)
            return;

        // Assigned crew start their trip now, under both toggle states:
        // summoning an assigned worker is their assignment taking effect,
        // not auto-assign. (Crew already travelling or at post are left be.)
        for (auto* staff : EntityList<Staff>())
        {
            if (staff->isWorker() && staff->currentRide == ride.id
                && (staff->state == PeepState::patrolling || staff->state == PeepState::falling))
            {
                staff->workerBeginHeadingToPost();
            }
        }

        if (!IsAutoAssignOnOpenEnabled())
            return;

        // Top up from the unassigned pool with the AutoAssignAll selection
        // rules, scoped to this one ride.
        const auto postLocation = GetPostLocation(ride);
        while (CountAssignedOperators(ride) < required)
        {
            Staff* bestWorker = nullptr;
            int64_t bestDistance = 0;
            for (auto* staff : EntityList<Staff>())
            {
                if (!staff->isWorker() || !staff->currentRide.IsNull())
                    continue;
                if (staff->state == PeepState::picked)
                    continue;
                const int64_t distance = DistanceSquared({ staff->x, staff->y }, postLocation);
                if (bestWorker == nullptr || distance < bestDistance
                    || (distance == bestDistance && staff->id.ToUnderlying() < bestWorker->id.ToUnderlying()))
                {
                    bestWorker = staff;
                    bestDistance = distance;
                }
            }
            if (bestWorker == nullptr)
                break;

            bestWorker->setWorkerAssignment(ride.id);
            bestWorker->workerBeginHeadingToPost();
        }
    }
} // namespace OpenRCT2::RideStaffing
