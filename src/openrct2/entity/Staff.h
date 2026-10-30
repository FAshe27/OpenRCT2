/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#pragma once

#include "../core/Money.hpp"
#include "Peep.h"

#include <cstdint>

namespace OpenRCT2::Drawing
{
    enum class Colour : uint8_t;
}

namespace OpenRCT2
{
    class DataSerialiser;
    class PatrolArea;

    struct Ride;
    struct TileElement;
    struct PathElement;

    enum class StaffType : uint8_t
    {
        handyman,
        mechanic,
        security,
        entertainer,
        worker,

        count
    };

    struct Staff : Peep
    {
        static constexpr auto kEntityType = EntityType::staff;

    public:
        PatrolArea* patrolInfo;
        StaffType assignedStaffType;
        uint16_t mechanicTimeSinceCall; // time getting to ride to fix
        int32_t hireDate;
        uint8_t staffOrders;
        uint8_t staffMowingTimeout;

        // Workers: an assignment change requested while this worker is
        // walking out of their old ride through its exit (setWorkerAssignment)
        // is deferred until the walk completes - the egress handler needs the
        // old assignment intact for the whole walk. Runtime-only, like
        // Ride::pendingOpenStatus: deliberately absent from serialise(), so a
        // save made mid-egress loads with no deferral pending and the worker
        // simply stays assigned to the old ride (the close-path semantics).
        // deferredAssignment may itself be null (an unassign defers too).
        RideId deferredAssignment;
        bool hasDeferredAssignment = false;

        // Workers: the worker's home break room (Break Rooms) - the room
        // chosen in the staff window's Break Room dropdown, or null for the
        // nearest-room default. Streamed in the ParkFile entity record
        // only, gated on kBreakRoomsVersion (see ParkFile.cpp); like the
        // other ParkFile-only worker state it is absent from serialise(),
        // so pre-65 saves load it as null (nearest).
        RideId breakRoom;

        // Workers: the break room this worker is currently travelling to.
        // The trip destination lives here rather than in currentRide,
        // which is the worker's assignment and must survive a break.
        // Runtime-only, like deferredAssignment: a save made mid-trip
        // loads with no trip in flight, and the worker think simply sends
        // him again after a fresh idle grace.
        RideId breakTripRoom;

        // Workers: the tick from which an idle worker may leave for his
        // break room (set when he first becomes idle-eligible, D4's grace
        // already added), or 0 while he is not counting. Runtime-only.
        uint32_t breakDepartureTick = 0;

        union // 1st statistic
        {
            uint32_t staffLawnsMown;
            uint32_t staffRidesFixed;
            uint32_t staffGuestsEntertained;
            uint32_t staffTrainsDispatched; // Workers: trains dispatched from their ride while at post
        };
        union // 2nd statistic
        {
            uint32_t staffGardensWatered;
            uint32_t staffRidesInspected;
        };
        union // 3rd statistic
        {
            uint32_t staffLitterSwept;
            uint32_t staffVandalsStopped;
        };
        union // 4th statistic
        {
            uint32_t staffBinsEmptied;
        };

        void update();
        void tick128UpdateStaff();
        bool isMechanic() const;
        bool isEntertainer() const;
        bool isWorker() const;
        bool isPatrolAreaSet(const CoordsXY& coords) const;
        bool isLocationInPatrol(const CoordsXY& loc) const;
        bool isLocationOnPatrolEdge(const CoordsXY& loc) const;
        bool doPathFinding();
        void setHireDate(int32_t value);
        int32_t getHireDate() const;

        bool canIgnoreWideFlag(const CoordsXYZ& staffPos, TileElement* path) const;

        static void resetStats();
        void serialise(DataSerialiser& stream);

        void setWorkerAssignment(RideId rideId);
        bool workerBeginHeadingToPost();
        bool isHiddenAtPost() const;

        // Workers: set the home break room (staff window dropdown, via
        // StaffSetBreakRoomAction). Null restores the nearest-room default.
        void setBreakRoom(RideId roomId);
        // Workers: step out of the break room he is resting in, back onto
        // its frontage path tile, resuming patrolling. The exit funnel for
        // every break-room exit trigger (assignment, summon, room closed).
        void workerLeaveBreakRoom();

        void clearPatrolArea();
        void setPatrolArea(const CoordsXY& coords, bool value);
        void setPatrolArea(const MapRange& range, bool value);
        bool hasPatrolArea() const;

    private:
        void updatePatrolling();
        void updateMowing();
        void updateSweeping();
        void updateEmptyingBin();
        void updateWatering();
        void updateAnswering();
        void updateFixing(int32_t steps);
        bool updateFixingEnterStation(Ride& ride) const;
        bool updateFixingMoveToBrokenDownVehicle(bool firstRun, const Ride& ride);
        bool updateFixingFixVehicle(bool firstRun, const Ride& ride);
        bool updateFixingFixVehicleMalfunction(bool firstRun, const Ride& ride);
        bool updateFixingMoveToStationEnd(bool firstRun, const Ride& ride);
        bool updateFixingFixStationEnd(bool firstRun);
        bool updateFixingMoveToStationStart(bool firstRun, const Ride& ride);
        bool updateFixingFixStationStart(bool firstRun, const Ride& ride);
        bool updateFixingFixStationBrakes(bool firstRun, Ride& ride);
        bool updateFixingMoveToStationExit(bool firstRun, const Ride& ride);
        bool updateFixingFinishFixOrInspect(bool firstRun, int32_t steps, Ride& ride);
        bool updateFixingLeaveByEntranceExit(bool firstRun, const Ride& ride);

        // Mechanics (two-tier breakdowns: the major tier's extended
        // on-site fix dwell inside fixing, fix round 5)
        bool updateFixingFixMajorDwell(bool firstRun, Ride& ride);
        void updateRideInspected(RideId rideIndex);
        void updateHeadingToInspect();

        // Workers (ride/stall operators)
        void tick128UpdateWorker();
        void updateWorkerHeadingToPost();
        void updateOperating();
        void workerLeavePost();
        void applyDeferredAssignment();
        bool workerBeginEgress(const Ride& ride);
        void updateWorkerEgress(const Ride* ride);
        void workerStepOffToAdjacentPath();
        bool doWorkerPathFinding();
        Direction workerDirectionPath(uint8_t validDirections, PathElement* pathElement);

        // Workers (break rooms)
        void tick128WorkerBreakRoomIdle();
        bool workerBeginBreakTrip(RideId roomId);
        void updateOnBreak();

        bool doHandymanPathFinding();
        bool doMechanicPathFinding();
        bool doEntertainerPathFinding();
        bool doMiscPathFinding();
        bool isMechanicHeadingToFixRideBlockingPath();

        Direction handymanDirectionRandSurface(uint8_t validDirections) const;

        void entertainerUpdateNearbyPeeps();
        bool securityGuardPathIsCrowded() const;

        uint8_t getValidPatrolDirections(const CoordsXY& loc) const;
        Direction handymanDirectionToNearestLitter() const;
        uint8_t handymanDirectionToUncutGrass(uint8_t valid_directions) const;
        Direction directionSurface(Direction initialDirection) const;
        Direction directionPath(uint8_t validDirections, PathElement* pathElement) const;
        Direction mechanicDirectionSurface() const;
        Direction mechanicDirectionPathRand(uint8_t pathDirections) const;
        Direction mechanicDirectionPath(uint8_t validDirections, PathElement* pathElement);
        bool updatePatrollingFindWatering();
        bool updatePatrollingFindBin();
        bool updatePatrollingFindSweeping();
        bool updatePatrollingFindGrass();
    };
    static_assert(sizeof(Staff) <= 512);

    enum STAFF_ORDERS
    {
        STAFF_ORDERS_SWEEPING = (1 << 0),
        STAFF_ORDERS_WATER_FLOWERS = (1 << 1),
        STAFF_ORDERS_EMPTY_BINS = (1 << 2),
        STAFF_ORDERS_MOWING = (1 << 3),
        STAFF_ORDERS_INSPECT_RIDES = (1 << 0),
        STAFF_ORDERS_FIX_RIDES = (1 << 1)
    };

    Drawing::Colour StaffGetColour(StaffType staffType);
    GameActions::Result StaffSetColour(StaffType staffType, Drawing::Colour value);

    money64 GetStaffWage(StaffType type);

    const PatrolArea& GetMergedPatrolArea(StaffType type);
} // namespace OpenRCT2
