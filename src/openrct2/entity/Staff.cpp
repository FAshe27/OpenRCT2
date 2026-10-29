/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "Staff.h"

#include "../Context.h"
#include "../Diagnostic.h"
#include "../GameState.h"
#include "../audio/Audio.h"
#include "../core/DataSerialiser.h"
#include "../core/Numerics.hpp"
#include "../entity/EntityList.h"
#include "../entity/EntityRegistry.h"
#include "../localisation/StringIds.h"
#include "../object/ObjectManager.h"
#include "../object/PathAdditionEntry.h"
#include "../object/PeepAnimationsObject.h"
#include "../object/SmallSceneryEntry.h"
#include "../paint/tile_element/Paint.TileElement.h"
#include "../peep/GuestPathfinding.h"
#include "../ride/RideData.h"
#include "../ride/RideStaffing.h"
#include "../ride/TrackData.h"
#include "../ride/TrackIteration.h"
#include "../ride/Vehicle.h"
#include "../ride/ted/TrackElementDescriptor.h"
#include "../scenario/Scenario.h"
#include "../util/Util.h"
#include "../world/Footpath.h"
#include "../world/Map.h"
#include "../world/Scenery.h"
#include "../world/TileElementsView.h"
#include "../world/Wall.h"
#include "../world/tile_element/EntranceElement.h"
#include "../world/tile_element/PathElement.h"
#include "../world/tile_element/Slope.h"
#include "../world/tile_element/SmallSceneryElement.h"
#include "../world/tile_element/SurfaceElement.h"
#include "../world/tile_element/TrackElement.h"
#include "PatrolArea.h"
#include "Peep.h"

#include <cassert>
#include <iterator>

namespace OpenRCT2
{
    template<>
    bool EntityBase::is<Staff>() const
    {
        return type == EntityType::staff;
    }

    /**
     *
     *  rct2: 0x006C0905
     */
    bool Staff::isLocationInPatrol(const CoordsXY& loc) const
    {
        // Check if location is in the park
        if (!MapIsLocationOwnedOrHasRights(loc))
            return false;

        // Check if staff has patrol area
        if (!hasPatrolArea())
            return true;

        return isPatrolAreaSet(loc);
    }

    // Check whether the location x,y is inside and on the edge of the
    // patrol zone for mechanic.
    bool Staff::isLocationOnPatrolEdge(const CoordsXY& loc) const
    {
        bool onZoneEdge = false;
        for (uint8_t neighbourDir = 0; !onZoneEdge && neighbourDir <= 7; neighbourDir++)
        {
            auto neighbourPos = loc + CoordsDirectionDelta[neighbourDir];
            onZoneEdge = !isLocationInPatrol(neighbourPos);
        }
        return onZoneEdge;
    }

    bool Staff::canIgnoreWideFlag(const CoordsXYZ& staffPos, TileElement* path) const
    {
        /* Wide flags can potentially wall off parts of a staff patrol zone
         * for the heuristic search.
         * This function provide doors through such "walls" by defining
         * the conditions under which staff can ignore the wide path flag. */
        /* Staff can ignore the wide flag on a path on the edge of the patrol
         * zone based on its adjacent tiles that are also in the patrol zone
         * but not on the patrol zone edge:
         * Basic points of interest are:
         * - how many such tiles there are;
         * - whether there are connected paths on those tiles;
         * - whether the connected paths have the wide flag set.
         * If there are no such tiles, the path is a concave corner of
         * the patrol zone and the wide flag can be ignored.
         * If there is one such tile, the path is on a straight side of the
         * patrol zone. If this one tile is either a connected wide path or
         * not a connected path, the wide flag can be ignored.
         * If there are two such tiles, the path is a convex corner of the
         * patrol zone. If at most one of these tiles is a connected path or
         * both of these tiles are connected wide paths, the wide flag can be
         * ignored. */

        if (!isLocationOnPatrolEdge(staffPos))
        {
            return false;
        }

        /* Check the connected adjacent paths that are also inside the patrol
         * zone but are not on the patrol zone edge have the wide flag set. */
        uint8_t total = 0;
        uint8_t pathcount = 0;
        uint8_t widecount = 0;
        for (Direction adjac_dir : kAllDirections)
        {
            auto adjacPos = staffPos + CoordsXYZ{ CoordsDirectionDelta[adjac_dir].x, CoordsDirectionDelta[adjac_dir].y, 0 };

            /* Ignore adjacent tiles outside the patrol zone. */
            if (!isLocationInPatrol(adjacPos))
                continue;

            /* Ignore adjacent tiles on the patrol zone edge. */
            if (isLocationOnPatrolEdge(adjacPos))
                continue;

            /* Adjacent tile is inside the patrol zone but not on the
             * patrol zone edge. */
            total++;

            /* Check if path has an edge in adjac_dir */
            if (!(path->asPath()->getEdges() & (1u << adjac_dir)))
            {
                continue;
            }

            if (path->asPath()->isSloped())
            {
                if (path->asPath()->getSlopeDirection() == adjac_dir)
                {
                    adjacPos.z += kPathHeightStep;
                }
            }

            /* Search through all adjacent map elements */
            TileElement* test_element = MapGetFirstElementAt(adjacPos);
            if (test_element == nullptr)
                return false;
            bool pathfound = false;
            bool widefound = false;
            do
            {
                if (test_element->getType() != TileElementType::path)
                {
                    continue;
                }

                /* test_element is a path */
                const auto* adjacentPathElement = test_element->asPath();
                if (!FootpathIsZAndDirectionValid(*adjacentPathElement, adjacPos.z / kCoordsZStep, adjac_dir))
                    continue;

                /* test_element is a connected path */
                if (!pathfound)
                {
                    pathfound = true;
                    pathcount++;
                }

                if (adjacentPathElement->isWide())
                {
                    if (!widefound)
                    {
                        widefound = true;
                        widecount++;
                    }
                }
            } while (!(test_element++)->isLastForTile());
        }

        switch (total)
        {
            case 0: /* Concave corner */
                return true;
            case 1: /* Straight side */
            case 2: /* Convex corner */
                if (pathcount <= total - 1 || widecount == total)
                {
                    return true;
                }
        }

        return false;
    }

    /**
     *
     *  rct2: 0x006C095B
     *  returns 0xF if not in a valid patrol area
     */
    uint8_t Staff::getValidPatrolDirections(const CoordsXY& loc) const
    {
        uint8_t directions = 0;

        if (isLocationInPatrol({ loc.x - kCoordsXYStep, loc.y }))
        {
            directions |= (1 << 0);
        }

        if (isLocationInPatrol({ loc.x, loc.y + kCoordsXYStep }))
        {
            directions |= (1 << 1);
        }

        if (isLocationInPatrol({ loc.x + kCoordsXYStep, loc.y }))
        {
            directions |= (1 << 2);
        }

        if (isLocationInPatrol({ loc.x, loc.y - kCoordsXYStep }))
        {
            directions |= (1 << 3);
        }

        if (directions == 0)
        {
            directions = 0xF;
        }

        return directions;
    }

    /**
     *
     *  rct2: 0x006C1955
     */
    void Staff::resetStats()
    {
        for (auto peep : EntityList<Staff>())
        {
            peep->setHireDate(GetDate().GetMonthsElapsed());
            peep->staffLawnsMown = 0;
            peep->staffRidesFixed = 0;
            peep->staffGardensWatered = 0;
            peep->staffRidesInspected = 0;
            peep->staffLitterSwept = 0;
            peep->staffVandalsStopped = 0;
            peep->staffBinsEmptied = 0;
            peep->staffGuestsEntertained = 0;
        }
    }

    bool Staff::isPatrolAreaSet(const CoordsXY& coords) const
    {
        if (patrolInfo != nullptr)
        {
            return patrolInfo->get(coords);
        }
        return false;
    }

    void Staff::setPatrolArea(const CoordsXY& coords, bool value)
    {
        if (patrolInfo == nullptr)
        {
            if (value)
            {
                patrolInfo = new PatrolArea();
            }
            else
            {
                return;
            }
        }

        patrolInfo->set(coords, value);
    }

    void Staff::setPatrolArea(const MapRange& range, bool value)
    {
        for (int32_t yy = range.getY1(); yy <= range.getY2(); yy += kCoordsXYStep)
        {
            for (int32_t xx = range.getX1(); xx <= range.getX2(); xx += kCoordsXYStep)
            {
                setPatrolArea({ xx, yy }, value);
            }
        }
    }

    void Staff::clearPatrolArea()
    {
        delete patrolInfo;
        patrolInfo = nullptr;
    }

    bool Staff::hasPatrolArea() const
    {
        return patrolInfo == nullptr ? false : !patrolInfo->isEmpty();
    }

    /**
     *
     *  rct2: 0x006BFBE8
     *
     * Returns kInvalidDirection when no nearby litter or unpathable litter
     */
    Direction Staff::handymanDirectionToNearestLitter() const
    {
        // Maximum manhattan distance that litter can be for a handyman to seek to it
        constexpr auto kTileRadius = 3;
        constexpr auto kLookupRadius = kCoordsXYStep * kTileRadius;

        auto nearestLitterDist = std::numeric_limits<int32_t>::max();
        Litter* nearestLitter = nullptr;

        for (int32_t tileX = x - kLookupRadius; tileX <= x + kLookupRadius; tileX += kCoordsXYStep)
        {
            for (int32_t tileY = y - kLookupRadius; tileY <= y + kLookupRadius; tileY += kCoordsXYStep)
            {
                for (auto* litter : EntityTileList<Litter>({ tileX, tileY }))
                {
                    // Calculate Manhattan distance with z-weighting
                    uint16_t distance = std::abs(litter->x - x) + std::abs(litter->y - y) + std::abs(litter->z - z) * 4;

                    if (distance < nearestLitterDist)
                    {
                        nearestLitterDist = distance;
                        nearestLitter = litter;
                    }
                }
            }
        }

        if (nearestLitterDist > kLookupRadius || nearestLitter == nullptr)
        {
            return kInvalidDirection;
        }

        auto litterTile = CoordsXY{ nearestLitter->x, nearestLitter->y }.toTileStart();

        if (!isLocationInPatrol(litterTile))
        {
            return kInvalidDirection;
        }

        Direction nextDirection = DirectionFromTo(CoordsXY(x, y), litterTile.toTileCentre());

        CoordsXY nextTile = litterTile.toTileStart() - CoordsDirectionDelta[nextDirection];

        int16_t nextZ = ((z + kCoordsZStep) & 0xFFF0) / kCoordsZStep;

        TileElement* tileElement = MapGetFirstElementAt(nextTile);
        if (tileElement == nullptr)
            return kInvalidDirection;
        do
        {
            if (tileElement->baseHeight != nextZ)
                continue;
            if (tileElement->getType() == TileElementType::entrance || tileElement->getType() == TileElementType::track)
            {
                return kInvalidDirection;
            }
        } while (!(tileElement++)->isLastForTile());

        nextTile = CoordsXY(x, y).toTileStart() + CoordsDirectionDelta[nextDirection];

        tileElement = MapGetFirstElementAt(nextTile);
        if (tileElement == nullptr)
            return kInvalidDirection;

        do
        {
            if (tileElement->baseHeight != nextZ)
                continue;
            if (tileElement->getType() == TileElementType::entrance || tileElement->getType() == TileElementType::track)
            {
                return kInvalidDirection;
            }
        } while (!(tileElement++)->isLastForTile());

        return nextDirection;
    }

    static bool isHandymanAlreadyServicingTile(const CoordsXY& tile, PeepState state)
    {
        if (state == PeepState::watering)
        {
            for (auto* staff : EntityList<Staff>())
            {
                if (staff->state == PeepState::watering
                    && CoordsXY{ staff->nextLoc } + CoordsDirectionDelta[staff->var37] == tile)
                    return true;
            }
        }
        else
        {
            for (auto* staff : EntityTileList<Staff>(tile))
            {
                if (staff->state == state)
                    return true;
            }
        }
        return false;
    }

    /**
     *
     *  rct2: 0x006BF931
     */
    uint8_t Staff::handymanDirectionToUncutGrass(uint8_t valid_directions) const
    {
        if (!(getNextIsSurface()))
        {
            auto surfaceElement = MapGetSurfaceElementAt(nextLoc);
            if (surfaceElement == nullptr)
                return kInvalidDirection;

            if (nextLoc.z != surfaceElement->getBaseZ())
                return kInvalidDirection;

            if (getNextIsSloped())
            {
                if (surfaceElement->getSlope() != kPathSlopeToLandSlope[getNextDirection()])
                    return kInvalidDirection;
            }
            else if (surfaceElement->getSlope() != kTileSlopeFlat)
                return kInvalidDirection;
        }

        uint8_t chosenDirection = ScenarioRand() & 0x3;
        for (uint8_t i = 0; i < 4; ++i, ++chosenDirection)
        {
            chosenDirection &= 0x3;

            if (!(valid_directions & (1 << chosenDirection)))
            {
                continue;
            }

            CoordsXY chosenTile = CoordsXY{ nextLoc } + CoordsDirectionDelta[chosenDirection];

            if (!MapIsLocationValid(chosenTile))
                continue;

            auto surfaceElement = MapGetSurfaceElementAt(chosenTile);
            if (surfaceElement != nullptr)
            {
                if (std::abs(surfaceElement->getBaseZ() - nextLoc.z) <= 2 * kCoordsZStep)
                {
                    if (surfaceElement->canGrassGrow() && (surfaceElement->getGrassLength() & 0x7) >= GRASS_LENGTH_CLEAR_1)
                    {
                        if (!isHandymanAlreadyServicingTile(chosenTile, PeepState::mowing))
                            return chosenDirection;
                    }
                }
            }
        }
        return kInvalidDirection;
    }

    /**
     *
     *  rct2: 0x006BFD9C
     */
    Direction Staff::handymanDirectionRandSurface(uint8_t validDirections) const
    {
        Direction newDirection = ScenarioRand() % kNumOrthogonalDirections;
        for (int32_t i = 0; i < kNumOrthogonalDirections; ++i, ++newDirection)
        {
            newDirection %= kNumOrthogonalDirections;
            if (!(validDirections & (1 << newDirection)))
                continue;

            CoordsXY chosenTile = CoordsXY{ nextLoc } + CoordsDirectionDelta[newDirection];

            if (MapSurfaceIsBlocked(chosenTile))
                continue;

            break;
        }
        // If it tries all directions this is required
        // to make it back to the first direction and
        // override validDirections
        newDirection %= kNumOrthogonalDirections;
        return newDirection;
    }

    /**
     *
     *  rct2: 0x006BFBA8
     */
    bool Staff::doHandymanPathFinding()
    {
        staffMowingTimeout++;

        Direction litterDirection = kInvalidDirection;
        uint8_t validDirections = getValidPatrolDirections(nextLoc);

        if ((staffOrders & STAFF_ORDERS_SWEEPING) && ((getGameState().currentTicks + id.ToUnderlying()) & 0xFFF) > 110)
        {
            litterDirection = handymanDirectionToNearestLitter();
        }

        Direction newDirection = kInvalidDirection;
        if (litterDirection == kInvalidDirection && (staffOrders & STAFF_ORDERS_MOWING) && staffMowingTimeout >= 12)
        {
            newDirection = handymanDirectionToUncutGrass(validDirections);
        }

        if (newDirection == kInvalidDirection)
        {
            if (getNextIsSurface())
            {
                newDirection = handymanDirectionRandSurface(validDirections);
            }
            else
            {
                auto* pathElement = MapGetPathElementAt(TileCoordsXYZ{ nextLoc });

                if (pathElement == nullptr)
                    return true;

                uint8_t pathDirections = (pathElement->getEdges() & validDirections) & 0xF;
                if (pathDirections == 0)
                {
                    newDirection = handymanDirectionRandSurface(validDirections);
                }
                else
                {
                    bool chooseRandom = true;
                    if (litterDirection != kInvalidDirection && pathDirections & (1 << litterDirection))
                    {
                        // Check whether path is a queue path and connected to a ride
                        bool connectedQueue = (pathElement->isQueue() && !pathElement->getRideIndex().IsNull());
                        // When in a queue path make the probability of following litter much lower (10% instead of 90%)
                        // as handymen often get stuck when there is litter on a normal path next to a queue they are in
                        uint32_t chooseRandomProbability = connectedQueue ? 0xE666 : 0x1999;
                        if ((ScenarioRand() & 0xFFFF) >= chooseRandomProbability)
                        {
                            chooseRandom = false;
                            newDirection = litterDirection;
                        }
                    }
                    else
                    {
                        pathDirections &= ~(1 << DirectionReverse(peepDirection));
                        if (pathDirections == 0)
                        {
                            pathDirections |= 1 << DirectionReverse(peepDirection);
                        }
                    }

                    if (chooseRandom)
                    {
                        do
                        {
                            newDirection = ScenarioRand() & 3;
                        } while ((pathDirections & (1 << newDirection)) == 0);
                    }
                }
            }
        }

        // newDirection can only contain a cardinal direction at this point, no diagonals
        assert(DirectionValid(newDirection));

        CoordsXY chosenTile = CoordsXY{ nextLoc } + CoordsDirectionDelta[newDirection];

        while (!MapIsLocationValid(chosenTile))
        {
            newDirection = handymanDirectionRandSurface(validDirections);
            chosenTile = CoordsXY{ nextLoc } + CoordsDirectionDelta[newDirection];
        }

        peepDirection = newDirection;
        setDestination(chosenTile + CoordsXY{ 16, 16 }, 3);
        if (state == PeepState::queuing)
        {
            destinationTolerance = (ScenarioRand() & 7) + 2;
        }
        return false;
    }

    Direction Staff::directionSurface(Direction initialDirection) const
    {
        uint8_t direction = initialDirection;
        for (int32_t i = 0; i < 3; ++i)
        {
            // Looks left and right from initial direction
            switch (i)
            {
                case 1:
                    direction++;
                    if (ScenarioRand() & 1)
                    {
                        direction -= 2;
                    }
                    break;
                case 2:
                    direction -= 2;
                    break;
            }

            direction &= 3;

            if (WallInTheWay({ nextLoc, nextLoc.z, nextLoc.z + kPeepClearanceHeight }, direction))
                continue;

            if (WallInTheWay({ nextLoc, nextLoc.z, nextLoc.z + kPeepClearanceHeight }, DirectionReverse(direction)))
                continue;

            CoordsXY chosenTile = CoordsXY{ nextLoc } + CoordsDirectionDelta[direction];

            if (!MapSurfaceIsBlocked(chosenTile))
            {
                return direction;
            }
        }
        return initialDirection;
    }

    /**
     *
     *  rct2: 0x006BFF45
     */
    Direction Staff::mechanicDirectionSurface() const
    {
        Direction direction = ScenarioRand() & 3;

        auto ride = GetRide(currentRide);
        if (ride != nullptr && (state == PeepState::answering || state == PeepState::headingToInspection)
            && (ScenarioRand() & 1))
        {
            auto location = ride->getStation(currentRideStation).exit;
            if (location.isNull())
            {
                location = ride->getStation(currentRideStation).entrance;
            }

            direction = DirectionFromTo(CoordsXY(x, y), location.toCoordsXY());
        }

        return directionSurface(direction);
    }

    /**
     *
     *  rct2: 0x006C02D1
     */
    Direction Staff::mechanicDirectionPathRand(uint8_t pathDirections) const
    {
        if (ScenarioRand() & 1)
        {
            if (pathDirections & (1 << peepDirection))
                return peepDirection;
        }

        // Modified from original to spam scenario_rand less
        uint8_t direction = ScenarioRand() & 3;
        for (int32_t i = 0; i < 4; ++i, ++direction)
        {
            direction &= 3;
            if (pathDirections & (1 << direction))
                return direction;
        }
        // This will never happen as pathDirections always has a bit set.
        return peepDirection;
    }

    /**
     *
     *  rct2: 0x006C0121
     */
    Direction Staff::mechanicDirectionPath(uint8_t validDirections, PathElement* pathElement)
    {
        uint32_t pathDirections = pathElement->getEdges();
        pathDirections &= validDirections;

        if (pathDirections == 0)
        {
            return mechanicDirectionSurface();
        }

        // Check if this is dead end - i.e. only way out is the reverse direction.
        pathDirections &= ~(1 << DirectionReverse(peepDirection));
        if (pathDirections == 0)
        {
            pathDirections |= (1 << DirectionReverse(peepDirection));
        }

        Direction direction = Numerics::bitScanForward(pathDirections);
        pathDirections &= ~(1 << direction);
        if (pathDirections == 0)
        {
            if (state != PeepState::answering && state != PeepState::headingToInspection)
            {
                return direction;
            }

            if (subState != 2)
            {
                return direction;
            }
            subState = 3;
        }

        pathDirections |= (1 << direction);

        // Mechanic is heading to ride (either broken down or for inspection).
        auto ride = GetRide(currentRide);
        if (ride != nullptr && (state == PeepState::answering || state == PeepState::headingToInspection))
        {
            /* Find location of the exit for the target ride station
             * or if the ride has no exit, the entrance. */
            TileCoordsXYZD location = ride->getStation(currentRideStation).exit;
            if (location.isNull())
            {
                location = ride->getStation(currentRideStation).entrance;

                // If no entrance is present either. This is an incorrect state.
                if (location.isNull())
                {
                    return mechanicDirectionPathRand(pathDirections);
                }
            }

            const auto goalPos = TileCoordsXYZ{ location };
            Direction pathfindDirection = PathFinding::ChooseDirection(
                TileCoordsXYZ{ nextLoc }, goalPos, *this, false, RideId::GetNull());
            if (pathfindDirection == kInvalidDirection)
            {
                /* Heuristic search failed for all directions.
                 * Reset the PathfindGoal - this means that the PathfindHistory
                 * will be reset in the next call to GuestPathfinding::ChooseDirection().
                 * This lets the heuristic search "try again" in case the player has
                 * edited the path layout or the mechanic was already stuck in the
                 * save game (e.g. with a worse version of the pathfinding). */
                resetPathfindGoal();
                return mechanicDirectionPathRand(pathDirections);
            }

            return pathfindDirection;
        }
        return mechanicDirectionPathRand(pathDirections);
    }

    /**
     *
     *  rct2: 0x006BFF2C
     */
    bool Staff::doMechanicPathFinding()
    {
        // Two-tier breakdowns: on a parts run the mechanic walks the path
        // network to a goal the stock ride-bound logic does not know
        // (outbound: the park entrance; return: this ride's station).
        if (isOnPartsRun())
        {
            return doPartsRunPathFinding();
        }

        uint8_t validDirections = getValidPatrolDirections(nextLoc);
        Direction newDirection = kInvalidDirection;
        if (getNextIsSurface())
        {
            newDirection = mechanicDirectionSurface();
        }
        else
        {
            auto* pathElement = MapGetPathElementAt(TileCoordsXYZ{ nextLoc });
            if (pathElement == nullptr)
                return true;

            newDirection = mechanicDirectionPath(validDirections, pathElement);
        }

        // countof(CoordsDirectionDelta)
        assert(DirectionValid(newDirection));

        CoordsXY chosenTile = CoordsXY{ nextLoc } + CoordsDirectionDelta[newDirection];

        while (!MapIsLocationValid(chosenTile))
        {
            newDirection = mechanicDirectionSurface();
            chosenTile = CoordsXY{ nextLoc } + CoordsDirectionDelta[newDirection];
        }

        peepDirection = newDirection;
        auto tolerance = (ScenarioRand() & 7) + 2;
        setDestination(chosenTile + CoordsXY{ 16, 16 }, tolerance);

        return false;
    }

    /**
     *
     *  rct2: 0x006C050B
     */
    Direction Staff::directionPath(uint8_t validDirections, PathElement* pathElement) const
    {
        uint32_t pathDirections = pathElement->getEdges();
        if (state != PeepState::answering && state != PeepState::headingToInspection)
        {
            pathDirections &= validDirections;
        }

        if (pathDirections == 0)
        {
            return directionSurface(ScenarioRand() & 3);
        }

        pathDirections &= ~(1u << DirectionReverse(peepDirection));
        if (pathDirections == 0)
        {
            pathDirections |= (1u << DirectionReverse(peepDirection));
        }

        Direction direction = Numerics::bitScanForward(pathDirections);
        // If this is the only direction they can go, then go
        if (pathDirections == (1u << direction))
        {
            return direction;
        }

        direction = ScenarioRand() & 3;
        for (uint8_t i = 0; i < kNumOrthogonalDirections; ++i, direction = DirectionNext(direction))
        {
            if (pathDirections & (1u << direction))
                return direction;
        }

        // This will never happen as pathDirections will always have a bit set
        return direction;
    }

    /**
     *
     *  rct2: 0x006C0351
     */
    bool Staff::doMiscPathFinding()
    {
        uint8_t validDirections = getValidPatrolDirections(nextLoc);

        Direction newDirection = kInvalidDirection;
        if (getNextIsSurface())
        {
            newDirection = directionSurface(ScenarioRand() & 3);
        }
        else
        {
            auto* pathElement = MapGetPathElementAt(TileCoordsXYZ{ nextLoc });
            if (pathElement == nullptr)
                return true;

            newDirection = directionPath(validDirections, pathElement);
        }

        CoordsXY chosenTile = CoordsXY{ nextLoc } + CoordsDirectionDelta[newDirection];

        while (!MapIsLocationValid(chosenTile))
        {
            newDirection = directionSurface(ScenarioRand() & 3);
            chosenTile = CoordsXY{ nextLoc } + CoordsDirectionDelta[newDirection];
        }

        peepDirection = newDirection;
        auto tolerance = (ScenarioRand() & 7) + 2;
        setDestination(chosenTile + CoordsXY{ 16, 16 }, tolerance);

        return false;
    }

    bool Staff::isMechanicHeadingToFixRideBlockingPath()
    {
        if (!isMechanic())
            return false;

        auto tileCoords = TileCoordsXYZ(CoordsXYZ{ getDestination(), nextLoc.z });
        auto trackElement = MapGetFirstTileElementWithBaseHeightBetween<TrackElement>(
            { tileCoords, tileCoords.z + kPathHeightStep });
        if (trackElement == nullptr)
            return false;

        auto ride = GetRide(trackElement->getRideIndex());
        if (ride == nullptr)
            return false;

        return ride->id == currentRide;
    }

    /**
     *
     *  rct2: 0x006C086D
     */
    void Staff::entertainerUpdateNearbyPeeps()
    {
        // Iterate over tiles within a 3-tile radius (96 units)
        constexpr auto kTileRadius = 3;
        constexpr auto kLookupRadius = kCoordsXYStep * kTileRadius;

        for (int32_t tileX = x - kLookupRadius; tileX <= x + kLookupRadius; tileX += kCoordsXYStep)
        {
            for (int32_t tileY = y - kLookupRadius; tileY <= y + kLookupRadius; tileY += kCoordsXYStep)
            {
                for (auto* guest : EntityTileList<Guest>({ tileX, tileY }))
                {
                    if (guest->x == kLocationNull)
                        continue;

                    int16_t z_dist = std::abs(z - guest->z);
                    if (z_dist > kTileRadius / 2)
                        continue;

                    int16_t x_dist = std::abs(x - guest->x);
                    int16_t y_dist = std::abs(y - guest->y);

                    if (x_dist > kLookupRadius)
                        continue;

                    if (y_dist > kLookupRadius)
                        continue;

                    if (guest->state == PeepState::walking)
                    {
                        guest->happinessTarget = std::min(guest->happinessTarget + 4, kPeepMaxHappiness);
                        staffGuestsEntertained = AddClamp(staffGuestsEntertained, 1u);
                        windowInvalidateFlags |= PEEP_INVALIDATE_STAFF_STATS;
                    }
                    else if (guest->state == PeepState::queuing)
                    {
                        guest->timeInQueue = std::max(0, guest->timeInQueue - 200);
                        guest->happinessTarget = std::min(guest->happinessTarget + 3, kPeepMaxHappiness);
                        staffGuestsEntertained = AddClamp(staffGuestsEntertained, 1u);
                        windowInvalidateFlags |= PEEP_INVALIDATE_STAFF_STATS;
                    }
                }
            }
        }
    }

    /**
     *
     *  rct2: 0x006C05AE
     */
    bool Staff::doEntertainerPathFinding()
    {
        if (((ScenarioRand() & 0xFFFF) <= 0x4000) && isActionInterruptableSafely())
        {
            action = (ScenarioRand() & 1) ? PeepActionType::wave2 : PeepActionType::joy;
            animationFrameNum = 0;
            animationImageIdOffset = 0;

            updateCurrentAnimationType();
            entertainerUpdateNearbyPeeps();
        }

        return doMiscPathFinding();
    }

    /**
     *
     *  rct2: 0x006BF926
     */
    bool Staff::doPathFinding()
    {
        switch (assignedStaffType)
        {
            case StaffType::handyman:
                return doHandymanPathFinding();
            case StaffType::mechanic:
                return doMechanicPathFinding();
            case StaffType::security:
                return doMiscPathFinding();
            case StaffType::entertainer:
                return doEntertainerPathFinding();
            case StaffType::worker:
                return doWorkerPathFinding();

            default:
                assert(false);
                return false;
        }
    }

    void Staff::setHireDate(int32_t value)
    {
        this->hireDate = value;
    }

    int32_t Staff::getHireDate() const
    {
        return hireDate;
    }

    Drawing::Colour StaffGetColour(StaffType staffType)
    {
        const auto& park = getGameState().park;
        switch (staffType)
        {
            case StaffType::handyman:
                return park.staffHandymanColour;
            case StaffType::mechanic:
                return park.staffMechanicColour;
            case StaffType::security:
                return park.staffSecurityColour;
            case StaffType::entertainer:
                return Drawing::Colour::black;
            case StaffType::worker:
                return park.staffWorkerColour;
            default:
                assert(false);
                return Drawing::Colour::black;
        }
    }

    GameActions::Result StaffSetColour(StaffType staffType, OpenRCT2::Drawing::Colour value)
    {
        auto& park = getGameState().park;
        switch (staffType)
        {
            case StaffType::handyman:
                park.staffHandymanColour = value;
                break;
            case StaffType::mechanic:
                park.staffMechanicColour = value;
                break;
            case StaffType::security:
                park.staffSecurityColour = value;
                break;
            case StaffType::worker:
                park.staffWorkerColour = value;
                break;
            default:
                return GameActions::Result(
                    GameActions::Status::invalidParameters, STR_ERR_INVALID_PARAMETER,
                    STR_ERR_ACTION_INVALID_FOR_THAT_STAFF_TYPE);
        }
        return GameActions::Result();
    }

    /** rct2: 0x009929C8 */
    static constexpr CoordsXY kMowingWaypoints[] = {
        { 28, 28 }, { 28, 4 }, { 20, 4 }, { 20, 28 }, { 12, 28 }, { 12, 4 }, { 4, 4 }, { 4, 28 },
    };

    /**
     *
     *  rct2: 0x006BF567
     */
    void Staff::updateMowing()
    {
        if (!checkForPath())
            return;

        while (true)
        {
            if (auto loc = updateAction(); loc.has_value())
            {
                int16_t checkZ = TileElementHeight(*loc);
                moveTo({ loc.value(), checkZ });
                return;
            }

            var37++;

            if (var37 == 1)
            {
                switchToSpecialSprite(2);
            }

            if (var37 == std::size(kMowingWaypoints))
            {
                stateReset();
                return;
            }

            auto destination = kMowingWaypoints[var37] + nextLoc;
            setDestination(destination);

            if (var37 != 7)
                continue;

            auto surfaceElement = MapGetSurfaceElementAt(nextLoc);
            if (surfaceElement != nullptr && surfaceElement->canGrassGrow())
            {
                surfaceElement->setGrassLength(GRASS_LENGTH_MOWED);
                MapInvalidateTileZoom0({ nextLoc, surfaceElement->getBaseZ(), surfaceElement->getBaseZ() + 16 });
            }
            staffLawnsMown = AddClamp(staffLawnsMown, 1u);
            windowInvalidateFlags |= PEEP_INVALIDATE_STAFF_STATS;
        }
    }

    /**
     *
     *  rct2: 0x006BF7E6
     */
    void Staff::updateWatering()
    {
        staffMowingTimeout = 0;
        if (subState == 0)
        {
            if (!checkForPath())
                return;

            const auto [pathingResult, _] = performNextAction();
            if (!(pathingResult & PATHING_DESTINATION_REACHED))
                return;

            orientation = (var37 & 3) << 3;
            action = PeepActionType::staffWatering;
            animationFrameNum = 0;
            animationImageIdOffset = 0;
            updateCurrentAnimationType();

            subState = 1;
        }
        else if (subState == 1)
        {
            if (!isActionWalking())
            {
                updateAction();
                invalidate();
                return;
            }

            auto actionLoc = CoordsXY{ nextLoc } + CoordsDirectionDelta[var37];

            TileElement* tile_element = MapGetFirstElementAt(actionLoc);
            if (tile_element == nullptr)
                return;

            do
            {
                if (tile_element->getType() != TileElementType::smallScenery)
                    continue;

                if (abs(nextLoc.z - tile_element->getBaseZ()) > 4 * kCoordsZStep)
                    continue;

                const auto* sceneryEntry = tile_element->asSmallScenery()->getEntry();

                if (sceneryEntry == nullptr || !sceneryEntry->flags.has(SmallSceneryFlag::canBeWatered))
                    continue;

                tile_element->asSmallScenery()->setAge(0);
                MapInvalidateTileZoom0({ actionLoc, tile_element->getBaseZ(), tile_element->getClearanceZ() });
                staffGardensWatered = AddClamp(staffGardensWatered, 1u);
                windowInvalidateFlags |= PEEP_INVALIDATE_STAFF_STATS;
            } while (!(tile_element++)->isLastForTile());

            stateReset();
        }
    }

    /**
     *
     *  rct2: 0x006BF6C9
     */
    void Staff::updateEmptyingBin()
    {
        staffMowingTimeout = 0;

        if (subState == 0)
        {
            if (!checkForPath())
                return;

            const auto [pathingResult, _] = performNextAction();
            if (!(pathingResult & PATHING_DESTINATION_REACHED))
                return;

            orientation = (var37 & 3) << 3;
            action = PeepActionType::staffEmptyBin;
            animationFrameNum = 0;
            animationImageIdOffset = 0;
            updateCurrentAnimationType();

            subState = 1;
        }
        else if (subState == 1)
        {
            if (isActionWalking())
            {
                stateReset();
                return;
            }

            updateAction();
            invalidate();

            if (animationFrameNum != 11)
                return;

            TileElement* tile_element = MapGetFirstElementAt(nextLoc);
            if (tile_element == nullptr)
                return;

            for (;; tile_element++)
            {
                if (tile_element->getType() == TileElementType::path)
                {
                    if (nextLoc.z == tile_element->getBaseZ())
                        break;
                }
                if ((tile_element)->isLastForTile())
                {
                    stateReset();
                    return;
                }
            }

            if (!tile_element->asPath()->hasAddition())
            {
                stateReset();
                return;
            }

            auto* pathAddEntry = tile_element->asPath()->getAdditionEntry();
            if (pathAddEntry == nullptr || !pathAddEntry->flags.has(PathAdditionFlag::isBin)
                || tile_element->asPath()->isBroken() || tile_element->asPath()->additionIsGhost())
            {
                stateReset();
                return;
            }

            uint8_t additionStatus = tile_element->asPath()->getAdditionStatus() | ((3 << var37) << var37);
            tile_element->asPath()->setAdditionStatus(additionStatus);

            MapInvalidateTileZoom0({ nextLoc, tile_element->getBaseZ(), tile_element->getClearanceZ() });
            staffBinsEmptied = AddClamp(staffBinsEmptied, 1u);
            windowInvalidateFlags |= PEEP_INVALIDATE_STAFF_STATS;
        }
    }

    /**
     *
     *  rct2: 0x6BF641
     */
    void Staff::updateSweeping()
    {
        staffMowingTimeout = 0;
        if (!checkForPath())
            return;

        if (action == PeepActionType::staffSweep && animationFrameNum == 8)
        {
            // Remove sick at this location
            Litter::removeAt(getLocation());
            staffLitterSwept = AddClamp(staffLitterSwept, 1u);
            windowInvalidateFlags |= PEEP_INVALIDATE_STAFF_STATS;
        }
        if (auto loc = updateAction(); loc.has_value())
        {
            int16_t actionZ = getZOnSlope(loc->x, loc->y);
            moveTo({ loc.value(), actionZ });
            return;
        }

        var37++;
        if (var37 != 2)
        {
            action = PeepActionType::staffSweep;
            animationFrameNum = 0;
            animationImageIdOffset = 0;
            updateCurrentAnimationType();
            return;
        }
        stateReset();
    }

    /**
     *
     *  rct2: 0x006C16D7
     */
    void Staff::updateHeadingToInspect()
    {
        auto ride = GetRide(currentRide);
        if (ride == nullptr)
        {
            setState(PeepState::falling);
            return;
        }

        if (ride->getStation(currentRideStation).exit.isNull())
        {
            ride->flags.unset(RideFlag::dueInspection);
            setState(PeepState::falling);
            return;
        }

        if (ride->mechanicStatus != MechanicStatus::heading || !ride->flags.has(RideFlag::dueInspection))
        {
            setState(PeepState::falling);
            return;
        }

        if (subState == 0)
        {
            mechanicTimeSinceCall = 0;
            resetPathfindGoal();
            subState = 2;
        }

        if (subState <= 3)
        {
            mechanicTimeSinceCall++;
            if (mechanicTimeSinceCall > 2500)
            {
                if (ride->flags.has(RideFlag::dueInspection) && ride->mechanicStatus == MechanicStatus::heading)
                {
                    ride->mechanicStatus = MechanicStatus::calling;
                }
                setState(PeepState::falling);
                return;
            }

            if (!checkForPath())
                return;

            if (shouldWaitForLevelCrossing() && !isMechanicHeadingToFixRideBlockingPath())
                return;

            const auto [pathingResult, rideEntranceExitElement] = performNextAction();
            if (!(pathingResult & PATHING_RIDE_EXIT) && !(pathingResult & PATHING_RIDE_ENTRANCE))
            {
                return;
            }

            if (currentRide != rideEntranceExitElement->asEntrance()->getRideIndex())
                return;

            StationIndex exitIndex = rideEntranceExitElement->asEntrance()->getStationIndex();
            if (currentRideStation != exitIndex)
                return;

            if (pathingResult & PATHING_RIDE_ENTRANCE)
            {
                if (!ride->getStation(exitIndex).exit.isNull())
                {
                    return;
                }
            }

            peepDirection = rideEntranceExitElement->getDirection();

            auto newDestination = CoordsXY{ 16, 16 } + nextLoc + (DirectionOffsets[peepDirection] * 53);
            setDestination(newDestination, 2);
            orientation = peepDirection << 3;

            z = rideEntranceExitElement->baseHeight * 4;
            subState = 4;
            // Falls through into SubState 4
        }

        int16_t delta_y = abs(getLocation().y - getDestination().y);
        if (auto loc = updateAction(); loc.has_value())
        {
            auto newZ = ride->getStation(currentRideStation).getBaseZ();
            if (delta_y < 20)
            {
                newZ += ride->getRideTypeDescriptor().Heights.PlatformHeight;
            }

            moveTo({ loc.value(), newZ });
            return;
        }

        setState(PeepState::inspecting);
        subState = 0;
    }

    /**
     *
     *  rct2: 0x006C0CB8
     */
    void Staff::updateAnswering()
    {
        auto ride = GetRide(currentRide);
        if (ride == nullptr || ride->mechanicStatus != MechanicStatus::heading)
        {
            setState(PeepState::falling);
            return;
        }

        if (subState == 0)
        {
            action = PeepActionType::staffAnswerCall;
            animationFrameNum = 0;
            animationImageIdOffset = 0;

            updateCurrentAnimationType();

            subState = 1;
            PeepWindowStateUpdate(this);
            return;
        }
        if (subState == 1)
        {
            if (isActionWalking())
            {
                subState = 2;
                PeepWindowStateUpdate(this);
                mechanicTimeSinceCall = 0;
                resetPathfindGoal();
                return;
            }
            updateAction();
            invalidate();
            return;
        }
        if (subState <= 3)
        {
            mechanicTimeSinceCall++;
            if (mechanicTimeSinceCall > 2500)
            {
                ride->mechanicStatus = MechanicStatus::calling;
                ride->windowInvalidateFlags.set(RideInvalidateFlag::maintenance);
                setState(PeepState::falling);
                return;
            }

            if (!checkForPath())
                return;

            if (shouldWaitForLevelCrossing() && !isMechanicHeadingToFixRideBlockingPath())
                return;

            const auto [pathingResult, rideEntranceExitElement] = performNextAction();
            if (!(pathingResult & PATHING_RIDE_EXIT) && !(pathingResult & PATHING_RIDE_ENTRANCE))
            {
                return;
            }

            if (currentRide != rideEntranceExitElement->asEntrance()->getRideIndex())
                return;

            StationIndex exitIndex = rideEntranceExitElement->asEntrance()->getStationIndex();
            if (currentRideStation != exitIndex)
                return;

            if (pathingResult & PATHING_RIDE_ENTRANCE)
            {
                if (!ride->getStation(exitIndex).exit.isNull())
                {
                    return;
                }
            }

            peepDirection = rideEntranceExitElement->getDirection();

            int32_t destX = nextLoc.x + 16 + DirectionOffsets[peepDirection].x * 53;
            int32_t destY = nextLoc.y + 16 + DirectionOffsets[peepDirection].y * 53;

            setDestination({ destX, destY }, 2);
            orientation = peepDirection << 3;

            z = rideEntranceExitElement->baseHeight * 4;
            subState = 4;
            // Falls through into SubState 4
        }

        int16_t delta_y = abs(y - getDestination().y);
        if (auto loc = updateAction(); loc.has_value())
        {
            auto newZ = ride->getStation(currentRideStation).getBaseZ();
            if (delta_y < 20)
            {
                newZ += ride->getRideTypeDescriptor().Heights.PlatformHeight;
            }

            moveTo({ loc.value(), newZ });
            return;
        }

        setState(PeepState::fixing);
        subState = 0;
    }

    // Workers (ride/stall operators)
    //
    // A worker's assignment lives in Peep::currentRide (already serialised). Travelling to the
    // post reuses PeepState::headingToInspection, mirroring the mechanic's inspection trip;
    // standing at the post is PeepState::operating. Only workers at post count towards a
    // ride's staffing (see RideStaffing). Workers use stock staff pathfinding throughout;
    // the guest distance fields are never consulted for staff.

    // The subState a worker rides in while walking out of a closed ride
    // through its exit (workerBeginEgress / updateWorkerEgress). The travel
    // subStates are 0/2/3 and the platform step-in is 4, so 5 is free; it
    // is a runtime distinction inside the already-serialised
    // headingToInspection state, not a new PeepState.
    static constexpr uint8_t kWorkerSubStateEgress = 5;

    // The subState a worker rides in while stepping from a break room's
    // frontage tile onto the room tile itself (updateWorkerHeadingToPost).
    static constexpr uint8_t kWorkerSubStateBreakStepIn = 6;

    // How long an idle worker waits before walking to his break room
    // (D4): roughly half a game day. The wait runs on the 128-tick think
    // cadence, so the effective grace is this plus up to 127 ticks.
    static constexpr uint32_t kBreakRoomGraceTicks = 1000;

    void Staff::tick128UpdateWorker()
    {
        if (state == PeepState::picked)
            return;

        // Break rooms: home-room hygiene. A home room that was demolished,
        // or is no longer a break room, reverts to the nearest default.
        if (!breakRoom.IsNull())
        {
            const auto* home = GetRide(breakRoom);
            if (home == nullptr || !RideStaffing::IsBreakRoom(*home))
                breakRoom = RideId::GetNull();
        }

        if (state == PeepState::onBreak)
        {
            // Residence validity, on the think cadence like post validity:
            // the room is derived from the tile under the worker, so a
            // demolished room fails the derivation, and a closed room
            // empties through its frontage like a closed ride's post.
            const auto* room = RideStaffing::GetBreakRoomAtTile(CoordsXY{ x, y });
            if (room == nullptr)
            {
                workerStepOffToAdjacentPath();
                return;
            }
            if (room->status == RideStatus::closed)
            {
                workerLeaveBreakRoom();
                return;
            }

            // A resident whose ride needs him - opened, or gathering for
            // the opening gate - leaves and heads to post, dispatched
            // exactly like the patrolling worker in the branch below.
            auto* assigned = GetRide(currentRide);
            if (assigned != nullptr && RideStaffing::GetRequiredOperators(*assigned) > 0
                && (assigned->status != RideStatus::closed || assigned->pendingOpenStatus != RideStatus::closed))
            {
                workerLeaveBreakRoom();
                workerBeginHeadingToPost();
            }
            return;
        }

        if (!breakTripRoom.IsNull())
        {
            // A break trip in flight owns the worker: the assignment
            // branches below must not read it as an invalid assignment (a
            // break room requires no operators) and yank him out of it.
            // Two things end the ownership early: the trip was interrupted
            // (picked up and put down - he is no longer travelling), or
            // his ride called him to post mid-trip. Both forget the trip
            // and let the assignment branches settle him this think.
            auto* assigned = GetRide(currentRide);
            const bool calledToPost = assigned != nullptr && RideStaffing::GetRequiredOperators(*assigned) > 0
                && (assigned->status != RideStatus::closed || assigned->pendingOpenStatus != RideStatus::closed);
            if (!calledToPost && state == PeepState::headingToInspection)
                return;

            breakTripRoom = RideId::GetNull();
            breakDepartureTick = 0;
            if (state == PeepState::headingToInspection)
                setState(PeepState::falling);
        }

        auto* ride = GetRide(currentRide);
        const bool validAssignment = ride != nullptr && RideStaffing::GetRequiredOperators(*ride) > 0;

        if (!validAssignment)
        {
            // Assignment removed, or the ride was demolished / no longer needs operators.
            if (state == PeepState::operating || state == PeepState::headingToInspection)
            {
                if (ride != nullptr)
                    workerLeavePost();
                else
                    setState(PeepState::falling);
            }
            currentRide = RideId::GetNull();
            tick128WorkerBreakRoomIdle();
            return;
        }

        // A ride with a pending open request (Workers opening gate) is still
        // RideStatus::closed, but its crew must gather and hold post so the
        // request can promote; treat it as open for crew purposes.
        const bool awaitingCrew = ride->pendingOpenStatus != RideStatus::closed;

        if (state == PeepState::operating && ride->status == RideStatus::closed && !awaitingCrew)
        {
            workerLeavePost();
            return;
        }

        if ((state == PeepState::patrolling || state == PeepState::falling)
            && (ride->status != RideStatus::closed || awaitingCrew))
        {
            workerBeginHeadingToPost();
            return;
        }

        tick128WorkerBreakRoomIdle();
    }

    // The break-room idle rule (D4/D5): a worker with nothing to do -
    // unassigned, or assigned to a closed ride that is not gathering for
    // the opening gate - who has stood idle for the grace period walks to
    // his break room and rests. Runs at the exits of tick128UpdateWorker,
    // after the assignment branches have had their say: a worker they
    // dispatched, hold at post, or are moving is never idle here. Testing
    // and broken-down crews stay at post (their rides are not closed, or
    // they are standing operating), exactly as D5 rules.
    void Staff::tick128WorkerBreakRoomIdle()
    {
        if (state != PeepState::patrolling)
        {
            breakDepartureTick = 0;
            return;
        }

        // Assigned to a ride that is open, or gathering its crew for the
        // opening gate: the assignment owns him even while he waits.
        if (auto* ride = GetRide(currentRide); ride != nullptr && RideStaffing::GetRequiredOperators(*ride) > 0
            && (ride->status != RideStatus::closed || ride->pendingOpenStatus != RideStatus::closed))
        {
            breakDepartureTick = 0;
            return;
        }

        const auto currentTicks = getGameState().currentTicks;
        if (breakDepartureTick == 0)
        {
            // First idle think: start the grace. Whether the park has a
            // room at all is not worth distinguishing here - the
            // resolution below fails the same way and restarts it.
            breakDepartureTick = currentTicks + kBreakRoomGraceTicks;
            return;
        }
        if (currentTicks < breakDepartureTick)
            return;

        // Resolve the destination once, at trip start (D2/D3): the home
        // room when it can take him, else the nearest room with capacity.
        // When no room can take him - none built, all closed, all full -
        // the grace restarts instead of retrying every think: the
        // decision is once per idle transition, never a doorway retry.
        auto* target = RideStaffing::FindBreakRoomTripTarget(*this);
        if (target == nullptr)
        {
            breakDepartureTick = currentTicks + kBreakRoomGraceTicks;
            return;
        }

        breakDepartureTick = 0;
        workerBeginBreakTrip(target->id);
    }

    bool Staff::workerBeginBreakTrip(RideId roomId)
    {
        breakTripRoom = roomId;
        currentRideStation = StationIndex::FromUnderlying(0);
        setState(PeepState::headingToInspection);
        subState = 0;
        resetPathfindGoal();
        return true;
    }

    bool Staff::workerBeginHeadingToPost()
    {
        auto* ride = GetRide(currentRide);
        if (ride == nullptr)
            return false;

        if (ride->getRideTypeDescriptor().flags.has(RtdFlag::isShopOrFacility))
        {
            currentRideStation = StationIndex::FromUnderlying(0);
        }
        else
        {
            // Post at the station with the fewest operators already heading there or at post.
            bool foundStation = false;
            auto bestStation = StationIndex::FromUnderlying(0);
            int32_t bestCount = 0;
            const auto stations = ride->getStations();
            for (size_t i = 0; i < stations.size(); i++)
            {
                const auto stationIndex = StationIndex::FromUnderlying(static_cast<uint8_t>(i));
                const auto& station = ride->getStation(stationIndex);
                if (station.entrance.isNull() && station.exit.isNull())
                    continue;

                int32_t count = 0;
                for (auto* other : EntityList<Staff>())
                {
                    if (other != this && other->isWorker() && other->currentRide == ride->id
                        && other->currentRideStation == stationIndex
                        && (other->state == PeepState::operating || other->state == PeepState::headingToInspection))
                    {
                        count++;
                    }
                }
                if (!foundStation || count < bestCount)
                {
                    foundStation = true;
                    bestStation = stationIndex;
                    bestCount = count;
                }
            }
            if (!foundStation)
                return false;
            currentRideStation = bestStation;
        }

        setState(PeepState::headingToInspection);
        subState = 0;
        resetPathfindGoal();
        return true;
    }

    void Staff::workerLeavePost()
    {
        // Stall operators stand on a path tile (the stall's frontage tile):
        // there is nowhere to step off to, they resume patrolling where they stand.
        auto* assignedRide = GetRide(currentRide);
        if (assignedRide != nullptr && assignedRide->getRideTypeDescriptor().flags.has(RtdFlag::isShopOrFacility))
        {
            auto* tileElement = MapGetFirstElementAt(CoordsXY{ x, y });
            if (tileElement != nullptr)
            {
                do
                {
                    if (tileElement->getType() == TileElementType::path && !tileElement->isGhost())
                    {
                        setState(PeepState::patrolling);
                        subState = 0;
                        resetPathfindGoal();
                        setDestination({ x, y }, 2);
                        return;
                    }
                } while (!(tileElement++)->isLastForTile());
            }
            // Not standing on a path (e.g. a v1 save caught them on the stall
            // tile itself): fall through to the step-off below.
        }

        if (state == PeepState::headingToInspection && subState == kWorkerSubStateEgress)
        {
            // Already walking out when a second leave-post request lands
            // (the assignment was pulled mid-egress): the post is gone, so
            // complete the departure at once with the step-off below rather
            // than starting a second egress towards a stale station.
            workerStepOffToAdjacentPath();
            return;
        }

        // A ride operator at post leaves through their station's exit, the
        // way guests do. The v1 behaviour stepped sideways off the platform
        // onto whatever tile neighboured the slot - for an elevated station
        // that meant a fall (or a teleport down) onto the ground under the
        // ride, grass included.
        if (state == PeepState::operating && assignedRide != nullptr && workerBeginEgress(*assignedRide))
        {
            return;
        }

        // Travelling workers (never reached the post - they are on the path
        // network already) and operators whose ride has no usable exit:
        // step off onto an adjacent path tile.
        workerStepOffToAdjacentPath();
    }

    // Starts the walk out of a closed ride: from the platform slot back to
    // the point just inside the station's exit, where the egress handler
    // (updateWorkerEgress) steps the worker onto the exit tile's path.
    // The door is the same one the trip in used - workerDirectionPath
    // pathfinds to the station's exit, or to its entrance when the station
    // has no separate exit, and the step-in crosses that element - so
    // there is no in/out asymmetry: both directions use exit-else-entrance
    // of the station the worker is posted at (their currentRideStation).
    // Returns false when the station has neither an exit nor an entrance
    // (not yet placed, or deleted under them); the caller falls back to
    // the adjacent-path step-off.
    bool Staff::workerBeginEgress(const Ride& ride)
    {
        const auto& station = ride.getStation(currentRideStation);
        auto exitLoc = station.exit;
        if (exitLoc.isNull())
        {
            exitLoc = station.entrance;
        }
        if (exitLoc.isNull())
        {
            return false;
        }

        // The point just inside the exit where guests leaving the ride are
        // placed (Guest::updateRidePrepareForExit): the exit tile's
        // centre, shifted back against the exit's direction.
        auto destination = exitLoc.toCoordsXY().toTileCentre();
        destination.x -= DirectionOffsets[exitLoc.direction].x * 20;
        destination.y -= DirectionOffsets[exitLoc.direction].y * 20;

        setState(PeepState::headingToInspection);
        subState = kWorkerSubStateEgress;
        mechanicTimeSinceCall = 0;
        setDestination(destination, 2);
        return true;
    }

    // Per-tick egress: walk to the exit's inner point, then step onto the
    // path element in the exit tile - the same snap a guest leaving the
    // ride gets in Guest::updateRideLeaveExit - and resume patrolling,
    // from where stock staff pathfinding walks the worker away along the
    // path. The worker stays assigned throughout; only the at-post state
    // ends. Every failure path leaves the worker in a stock recovery
    // (step-off or falling), never wedged in the egress subState. An
    // assignment change deferred by setWorkerAssignment is applied on
    // every exit below (applyDeferredAssignment), so it lands exactly
    // once whichever way the walk ends.
    void Staff::updateWorkerEgress(const Ride* ride)
    {
        if (ride == nullptr)
        {
            // The ride was demolished mid-egress: the same stock recovery
            // as a worker caught at post by a demolition. A deferred
            // assignment change still takes effect - its target is another
            // ride, or none.
            applyDeferredAssignment();
            setState(PeepState::falling);
            return;
        }

        mechanicTimeSinceCall++;
        if (mechanicTimeSinceCall > 250)
        {
            // The walk out is a few dozen units at platform height; if
            // anything has gone wrong, complete the departure at once
            // rather than wedge here. The deferral applies after the
            // step-off: the step-off still reads the old ride's station
            // height.
            workerStepOffToAdjacentPath();
            applyDeferredAssignment();
            return;
        }

        int16_t xyDistance = 0;
        if (auto loc = updateAction(xyDistance); loc.has_value())
        {
            // Mirror the guests' in-exit walk (Guest::updateRideInExit):
            // at platform height until the exit's inner edge is close,
            // then at standing height.
            int32_t newZ = z;
            if (xyDistance >= 16)
            {
                newZ = ride->getStation(currentRideStation).getBaseZ() + ride->getRideTypeDescriptor().Heights.PlatformHeight;
            }
            moveTo({ loc.value(), newZ });
            return;
        }

        // Arrived at the exit's inner point: step onto the path element in
        // this tile at a matching height, as a leaving guest does.
        bool snapped = false;
        for (auto* pathElement : TileElementsView<PathElement>(CoordsXY{ x, y }))
        {
            int32_t height = MapHeightFromSlope(CoordsXY{ x, y }, pathElement->getSlopeDirection(), pathElement->isSloped());
            height += pathElement->getBaseZ();
            const int32_t zDiff = z - height;
            if (zDiff > 0 || zDiff < -16)
                continue;
            moveTo({ x, y, height });
            snapped = true;
            break;
        }
        if (!snapped)
        {
            // The exit tile's path is gone (deleted under the exit):
            // complete the departure with the step-off instead.
            workerStepOffToAdjacentPath();
            applyDeferredAssignment();
            return;
        }

        setState(PeepState::patrolling);
        subState = 0;
        // A deferred assignment change takes effect now the walk is
        // complete; the goal reset below then serves the new assignment.
        applyDeferredAssignment();
        resetPathfindGoal();
        setDestination({ x, y }, 2);
    }

    // The v1 departure, kept as the fallback for workers with no exit to
    // leave through: step onto an adjacent genuine path tile, then resume
    // patrolling. One v1 defect is fixed here: the step used to accept a
    // path element at ANY height, so an elevated platform could teleport
    // the worker down to a path running underneath the ride. A candidate
    // must now sit within the guest-interaction height window of the
    // worker's standing height ([-16, +8], the window GetStallPostTile
    // applies), and queue tiles do not count as a landing. When nothing
    // qualifies, the standard falling recovery applies - the worker drops
    // to the surface below and patrols from there, as stock staff do.
    void Staff::workerStepOffToAdjacentPath()
    {
        int32_t referenceZ = z;
        if (auto* assignedRide = GetRide(currentRide);
            assignedRide != nullptr && !assignedRide->getRideTypeDescriptor().flags.has(RtdFlag::isShopOrFacility))
        {
            referenceZ = assignedRide->getStation(currentRideStation).getBaseZ();
        }

        const auto currentTile = TileCoordsXY{ CoordsXY{ x, y } };
        for (Direction dir = 0; dir < kNumOrthogonalDirections; dir++)
        {
            const auto neighbour = TileCoordsXY(currentTile.toCoordsXY() + CoordsDirectionDelta[dir]);
            auto* tileElement = MapGetFirstElementAt(neighbour);
            if (tileElement == nullptr)
                continue;
            do
            {
                if (tileElement->getType() == TileElementType::path && !tileElement->isGhost()
                    && !tileElement->asPath()->isQueue())
                {
                    const int32_t dz = tileElement->getBaseZ() - referenceZ;
                    if (dz < -16 || dz > 8)
                        continue;
                    const auto dest = neighbour.toCoordsXY() + CoordsXY{ 16, 16 };
                    moveTo(CoordsXYZ{ dest, tileElement->getBaseZ() });
                    setState(PeepState::patrolling);
                    subState = 0;
                    resetPathfindGoal();
                    return;
                }
            } while (!(tileElement++)->isLastForTile());
        }

        // No adjacent path (e.g. the ride was demolished around us): standard recovery.
        setState(PeepState::falling);
    }

    // The tile a stall operator stands at: a path tile on one of the stall's
    // entrance-connection sides - the same sides RideShopConnected tests for
    // path connectivity, and the tile guests stand on to buy (a guest's step
    // toward the stall tile is what fires PeepInteractWithShop). Connection
    // sides are tried first; if none of them fronts a path, any side does, so
    // oddly-built stalls still get a post. A side qualifies when its tile
    // holds a non-ghost path element at a height guests could interact from
    // (the same height window performNextAction's element scan applies).
    static std::optional<TileCoordsXY> GetStallPostTile(const Ride& ride)
    {
        const auto shopTile = TileCoordsXY(ride.getStation().start);
        auto* tileElement = MapGetFirstElementAt(shopTile);
        if (tileElement == nullptr)
            return std::nullopt;

        TrackElement* trackElement = nullptr;
        do
        {
            if (tileElement->getType() == TileElementType::track && tileElement->asTrack()->getRideIndex() == ride.id)
            {
                trackElement = tileElement->asTrack();
                break;
            }
        } while (!(tileElement++)->isLastForTile());
        if (trackElement == nullptr)
            return std::nullopt;

        const auto& ted = TrackMetadata::GetTrackElementDescriptor(trackElement->getTrackType());
        uint8_t connectionSides = ted.sequenceData.sequences[0].getEntranceConnectionSides();
        connectionSides = Numerics::rol4(connectionSides, trackElement->getDirection());

        for (int32_t pass = 0; pass < 2; pass++)
        {
            for (Direction side = 0; side < kNumOrthogonalDirections; side++)
            {
                const bool isConnectionSide = (connectionSides & (1 << side)) != 0;
                if ((pass == 0) != isConnectionSide)
                    continue;

                const auto candidate = shopTile + TileDirectionDelta[side];
                auto* element = MapGetFirstElementAt(candidate);
                if (element == nullptr)
                    continue;
                do
                {
                    if (element->getType() == TileElementType::path && !element->isGhost())
                    {
                        const int32_t dz = trackElement->getBaseZ() - element->getBaseZ();
                        if (dz >= -16 && dz <= 8)
                            return candidate;
                    }
                } while (!(element++)->isLastForTile());
            }
        }
        return std::nullopt;
    }

    void Staff::updateWorkerHeadingToPost()
    {
        // A break trip rides in this state too (Break Rooms): its
        // destination is the room in breakTripRoom, not the assignment in
        // currentRide, and the shop branch below serves both - a break
        // room is a shop-class facility whose post is its frontage tile.
        const bool toBreakRoom = !breakTripRoom.IsNull();
        auto* ride = GetRide(toBreakRoom ? breakTripRoom : currentRide);

        // Egress rides in this state too (workerBeginEgress): the ride
        // being closed is the whole point of the walk out, so it is
        // handled before the closed-ride abort below.
        if (subState == kWorkerSubStateEgress)
        {
            updateWorkerEgress(ride);
            return;
        }

        // A pending-open ride is still RideStatus::closed, but its crew is
        // meant to be travelling to it (Workers opening gate). A break
        // room never has a pending open, so a closed room aborts the trip
        // like a demolished one.
        if (ride == nullptr || (ride->status == RideStatus::closed && ride->pendingOpenStatus == RideStatus::closed))
        {
            if (toBreakRoom)
                breakTripRoom = RideId::GetNull();
            setState(PeepState::falling);
            return;
        }

        const bool isShop = ride->getRideTypeDescriptor().flags.has(RtdFlag::isShopOrFacility);

        if (subState == 0)
        {
            mechanicTimeSinceCall = 0;
            resetPathfindGoal();
            subState = 2;
        }

        if (isShop)
        {
            // Stall operators take post on the stall's frontage path tile,
            // never on the stall tile itself. v1 stepped the worker onto the
            // stall tile, but staff bounce off shop tiles in
            // PeepInteractWithShop while travelling, and the step-in state sat
            // outside the per-tick block below, so the step-in moved once and
            // then wedged - with the give-up timer inside the same block, it
            // could never even time out. A path tile needs no special movement
            // at all: normal pathfinding walks there, and arrival is the tile
            // test below, re-checked every tick.
            mechanicTimeSinceCall++;
            if (mechanicTimeSinceCall > 2500)
            {
                // Could not reach the post; resume patrolling, the next think retries.
                if (toBreakRoom)
                    breakTripRoom = RideId::GetNull();
                setState(PeepState::falling);
                return;
            }

            if (subState == kWorkerSubStateBreakStepIn)
            {
                // Stepping from the room's frontage tile onto the room
                // tile itself (the destination was set on arrival at the
                // frontage, below): a straight-line walk at the room's
                // base height, like the ride platform step-in. The
                // give-up timer above covers this subState too.
                if (auto loc = updateAction(); loc.has_value())
                {
                    moveTo({ loc.value(), ride->getStation().getBaseZ() });
                    return;
                }

                // On the room tile. Capacity is checked again here - the
                // room may have filled during the walk in; a worker who
                // cannot enter steps back out to the frontage and waits
                // there, and the idle rule re-sends him after a fresh
                // grace rather than retrying at the doorway.
                breakTripRoom = RideId::GetNull();
                if (RideStaffing::CountWorkersOnBreak(*ride) < RideStaffing::kBreakRoomCapacity)
                {
                    setState(PeepState::onBreak);
                    subState = 0;
                }
                else
                {
                    breakDepartureTick = getGameState().currentTicks + kBreakRoomGraceTicks;
                    workerLeaveBreakRoom();
                }
                return;
            }

            if (!checkForPath())
                return;

            if (shouldWaitForLevelCrossing())
                return;

            const auto postTile = GetStallPostTile(*ride);
            if (!postTile.has_value())
            {
                // No path fronts the stall, so nobody can staff it yet; fall
                // back to patrolling and let the think retry if a path appears.
                setState(PeepState::falling);
                return;
            }

            // (x, y are world coordinates: the TileCoordsXY must be built
            // from a CoordsXY so it converts - building it from the raw
            // pair stores world numbers as tile numbers and the test can
            // never be true.)
            if (TileCoordsXY{ CoordsXY{ x, y } } == postTile.value())
            {
                if (toBreakRoom)
                {
                    // Arrived at the room's frontage: step onto the room
                    // tile itself and rest inside, undrawn. The step-in
                    // subState walks the last tile like a platform step-in.
                    const auto roomTile = TileCoordsXY(ride->getStation().start);
                    setDestination(roomTile.toCoordsXY() + CoordsXY{ 16, 16 }, 2);
                    subState = kWorkerSubStateBreakStepIn;
                    return;
                }

                // Face the stall across the counter and take post.
                const auto shopTile = TileCoordsXY(ride->getStation().start);
                for (Direction dir = 0; dir < kNumOrthogonalDirections; dir++)
                {
                    if (postTile.value() + TileDirectionDelta[dir] == shopTile)
                    {
                        peepDirection = dir;
                        orientation = peepDirection << 3;
                        break;
                    }
                }
                setState(PeepState::operating);
                subState = 0;
                return;
            }

            // Travel. Any other subState - including the step-in subState 4 a
            // v1 save can carry - is normalised to plain travel, so an in-flight
            // v1 stall worker recovers instead of wedging.
            subState = 2;
            performNextAction();
            return;
        }

        if (subState <= 3)
        {
            mechanicTimeSinceCall++;
            if (mechanicTimeSinceCall > 2500)
            {
                // Could not reach the post; resume patrolling, the next think retries.
                setState(PeepState::falling);
                return;
            }

            if (!checkForPath())
                return;

            if (shouldWaitForLevelCrossing())
                return;

            const auto [pathingResult, rideEntranceExitElement] = performNextAction();
            if (!(pathingResult & PATHING_RIDE_EXIT) && !(pathingResult & PATHING_RIDE_ENTRANCE))
            {
                return;
            }

            if (currentRide != rideEntranceExitElement->asEntrance()->getRideIndex())
                return;

            StationIndex exitIndex = rideEntranceExitElement->asEntrance()->getStationIndex();
            if (currentRideStation != exitIndex)
                return;

            if (pathingResult & PATHING_RIDE_ENTRANCE)
            {
                if (!ride->getStation(exitIndex).exit.isNull())
                {
                    return;
                }
            }

            peepDirection = rideEntranceExitElement->getDirection();

            // Spread this station's operators along the platform, ranked by entity id so the
            // slots are stable while the crew stays the same.
            int32_t rank = 0;
            int32_t crew = 1;
            for (auto* other : EntityList<Staff>())
            {
                if (other == this || !other->isWorker() || other->currentRide != currentRide
                    || other->currentRideStation != currentRideStation)
                    continue;
                if (other->state != PeepState::operating && other->state != PeepState::headingToInspection)
                    continue;
                crew++;
                if (other->id.ToUnderlying() < id.ToUnderlying())
                    rank++;
            }
            const int32_t along = (2 * rank - (crew - 1)) * 12;
            const Direction platformDirection = (peepDirection + 1) & 3;
            const auto alongOffset = CoordsXY{ DirectionOffsets[platformDirection].x * along / 32,
                                               DirectionOffsets[platformDirection].y * along / 32 };

            auto newDestination = CoordsXY{ 16, 16 } + nextLoc + (DirectionOffsets[peepDirection] * 53) + alongOffset;
            setDestination(newDestination, 2);
            orientation = peepDirection << 3;

            z = rideEntranceExitElement->baseHeight * 4;
            subState = 4;
            // Falls through into the step-in below
        }

        int16_t delta_y = abs(getLocation().y - getDestination().y);
        if (auto loc = updateAction(); loc.has_value())
        {
            auto newZ = ride->getStation(currentRideStation).getBaseZ();
            if (delta_y < 20)
            {
                newZ += ride->getRideTypeDescriptor().Heights.PlatformHeight;
            }

            moveTo({ loc.value(), newZ });
            return;
        }

        setState(PeepState::operating);
        subState = 0;
    }

    void Staff::updateOperating()
    {
        auto* ride = GetRide(currentRide);
        if (ride == nullptr)
        {
            setState(PeepState::falling);
            return;
        }

        // Stand at post. Post validity (ride closed, assignment removed, ride demolished)
        // is re-checked every 128 ticks in tick128UpdateWorker.
        if (auto loc = updateAction(); loc.has_value())
        {
            moveTo({ loc.value(), z });
        }
        invalidate();
    }

    bool Staff::isHiddenAtPost() const
    {
        if (!isWorker())
            return false;

        // Resting inside a break room: undrawn, like an operator at a
        // hidden post - the room is a building and the worker is in it.
        if (state == PeepState::onBreak)
            return true;

        if (state != PeepState::operating || currentRide.IsNull())
            return false;
        const auto* ride = GetRide(currentRide);
        return ride != nullptr && !RideStaffing::AreOperatorsVisibleAtPost(*ride);
    }

    // Resting inside a break room: stand at the room tile's centre,
    // undrawn (isHiddenAtPost). Residence validity - room closed or
    // demolished - is re-checked every 128 ticks in tick128UpdateWorker,
    // exactly like post validity for updateOperating.
    void Staff::updateOnBreak()
    {
        if (auto loc = updateAction(); loc.has_value())
        {
            moveTo({ loc.value(), z });
        }
        invalidate();
    }

    // The one way out of a break room: step back onto the room's frontage
    // path tile (the tile GetStallPostTile computes for the facility) and
    // resume patrolling there. Residents are undrawn, so the step reads
    // as the worker emerging from the room. Every exit trigger funnels
    // through here: an assignment instruction, a summon, the room
    // closing, a home-room change. When no frontage can be found (the
    // path was deleted under the room) the adjacent-path step-off is the
    // fallback, and stock falling recovery behind it.
    void Staff::workerLeaveBreakRoom()
    {
        if (const auto* room = RideStaffing::GetBreakRoomAtTile(CoordsXY{ x, y }); room != nullptr)
        {
            if (const auto postTile = GetStallPostTile(*room); postTile.has_value())
            {
                for (auto* pathElement : TileElementsView<PathElement>(postTile->toCoordsXY()))
                {
                    if (pathElement->isGhost())
                        continue;

                    const auto dest = postTile->toCoordsXY() + CoordsXY{ 16, 16 };
                    moveTo(CoordsXYZ{ dest, pathElement->getBaseZ() });
                    setState(PeepState::patrolling);
                    subState = 0;
                    resetPathfindGoal();
                    setDestination(dest, 2);
                    return;
                }
            }
        }

        workerStepOffToAdjacentPath();
    }

    bool Staff::doWorkerPathFinding()
    {
        if (state != PeepState::headingToInspection || (currentRide.IsNull() && breakTripRoom.IsNull()))
        {
            // Not travelling to a post: wander like other unassigned staff.
            return doMiscPathFinding();
        }

        uint8_t validDirections = getValidPatrolDirections(nextLoc);
        Direction newDirection = kInvalidDirection;
        if (getNextIsSurface())
        {
            newDirection = mechanicDirectionSurface();
        }
        else
        {
            auto* pathElement = MapGetPathElementAt(TileCoordsXYZ{ nextLoc });
            if (pathElement == nullptr)
                return true;

            newDirection = workerDirectionPath(validDirections, pathElement);
        }

        assert(DirectionValid(newDirection));

        CoordsXY chosenTile = CoordsXY{ nextLoc } + CoordsDirectionDelta[newDirection];

        while (!MapIsLocationValid(chosenTile))
        {
            newDirection = mechanicDirectionSurface();
            chosenTile = CoordsXY{ nextLoc } + CoordsDirectionDelta[newDirection];
        }

        peepDirection = newDirection;
        auto tolerance = (ScenarioRand() & 7) + 2;
        setDestination(chosenTile + CoordsXY{ 16, 16 }, tolerance);

        return false;
    }

    Direction Staff::workerDirectionPath(uint8_t validDirections, PathElement* pathElement)
    {
        uint32_t pathDirections = pathElement->getEdges();
        pathDirections &= validDirections;

        if (pathDirections == 0)
        {
            return mechanicDirectionSurface();
        }

        // Check if this is dead end - i.e. only way out is the reverse direction.
        pathDirections &= ~(1 << DirectionReverse(peepDirection));
        if (pathDirections == 0)
        {
            pathDirections |= (1 << DirectionReverse(peepDirection));
        }

        Direction direction = Numerics::bitScanForward(pathDirections);
        pathDirections &= ~(1 << direction);
        if (pathDirections == 0)
        {
            if (state != PeepState::headingToInspection)
            {
                return direction;
            }

            if (subState != 2)
            {
                return direction;
            }
            subState = 3;
        }

        pathDirections |= (1 << direction);

        // Worker is heading to their assigned ride or stall - or, on a
        // break trip, to their break room (the destination in
        // breakTripRoom, which the goal logic below treats exactly like a
        // stall: a break room is a shop-class facility).
        auto* ride = GetRide(!breakTripRoom.IsNull() ? breakTripRoom : currentRide);
        if (ride != nullptr && state == PeepState::headingToInspection)
        {
            TileCoordsXYZ goalPos;
            if (ride->getRideTypeDescriptor().flags.has(RtdFlag::isShopOrFacility))
            {
                // Head for the frontage post tile, not the stall tile: the
                // stall tile is not on the path network, so aiming at it made
                // the heuristic gradient fight the last step (and the shop
                // bounce reversed the worker at the counter). Fall back to the
                // stall tile only when no path fronts the stall at all.
                if (const auto postTile = GetStallPostTile(*ride); postTile.has_value())
                {
                    goalPos = TileCoordsXYZ{ CoordsXYZ{ postTile->toCoordsXY(), ride->getStation().getBaseZ() } };
                }
                else
                {
                    goalPos = TileCoordsXYZ{ ride->getStation().getStart() };
                }
            }
            else
            {
                /* Find location of the exit for the target ride station
                 * or if the ride has no exit, the entrance. */
                TileCoordsXYZD location = ride->getStation(currentRideStation).exit;
                if (location.isNull())
                {
                    location = ride->getStation(currentRideStation).entrance;

                    // If no entrance is present either. This is an incorrect state.
                    if (location.isNull())
                    {
                        return mechanicDirectionPathRand(pathDirections);
                    }
                }
                goalPos = TileCoordsXYZ{ location };
            }

            Direction pathfindDirection = PathFinding::ChooseDirection(
                TileCoordsXYZ{ nextLoc }, goalPos, *this, false, RideId::GetNull());
            if (pathfindDirection == kInvalidDirection)
            {
                /* Heuristic search failed for all directions.
                 * Reset the PathfindGoal - this means that the PathfindHistory
                 * will be reset in the next call to GuestPathfinding::ChooseDirection().
                 * This lets the heuristic search "try again" in case the player has
                 * edited the path layout or the worker was already stuck. */
                resetPathfindGoal();
                return mechanicDirectionPathRand(pathDirections);
            }

            return pathfindDirection;
        }
        return mechanicDirectionPathRand(pathDirections);
    }

    void Staff::setWorkerAssignment(RideId rideId)
    {
        // Break-room exits first: an assignment instruction is an exit
        // trigger. A resting worker emerges onto the room's frontage and
        // the normal flow below takes it from there (he is patrolling by
        // the time it runs); a worker still travelling to a room abandons
        // the trip - his state stays headingToInspection, so the leave
        // logic below settles him as a traveller whose post changed, and
        // the think re-dispatches from there.
        if (state == PeepState::onBreak)
        {
            workerLeaveBreakRoom();
        }
        else if (!breakTripRoom.IsNull())
        {
            breakTripRoom = RideId::GetNull();
            breakDepartureTick = 0;
        }

        if (currentRide == rideId)
        {
            // Reaffirming the current assignment also cancels a deferred move
            // away from it: the latest instruction wins.
            hasDeferredAssignment = false;
            return;
        }

        // A new instruction supersedes any deferred one still in flight.
        hasDeferredAssignment = false;

        // Stepping down from a post needs the old ride's geometry, so leave before switching.
        if (state == PeepState::operating || state == PeepState::headingToInspection)
        {
            if (GetRide(currentRide) != nullptr)
                workerLeavePost();
            else
                setState(PeepState::falling);
        }

        if (state == PeepState::headingToInspection && subState == kWorkerSubStateEgress)
        {
            // The leave began the walk out through the station exit. The
            // egress handler needs currentRide - and the egress subState -
            // intact for the whole walk, so the switch itself is deferred
            // and applied where the walk ends (applyDeferredAssignment,
            // called from updateWorkerEgress). Applying it here, as this
            // function used to, overwrote the subState the leave had just
            // set and dropped the worker where he stood.
            deferredAssignment = rideId;
            hasDeferredAssignment = true;
            return;
        }

        currentRide = rideId;
        currentRideStation = StationIndex::FromUnderlying(0);
        subState = 0;
        resetPathfindGoal();
    }

    void Staff::setBreakRoom(RideId roomId)
    {
        breakRoom = roomId;

        // A direct instruction re-targets a worker who is already resting
        // or already travelling to a room (the design's idle/at-break
        // re-target rule). Everyone else just has the field changed: the
        // idle rule reads it when it next resolves a trip.
        if (state == PeepState::onBreak)
        {
            const auto* resting = RideStaffing::GetBreakRoomAtTile(CoordsXY{ x, y });
            if (!roomId.IsNull() && (resting == nullptr || resting->id != roomId))
            {
                // A specific new home that can take him: move. (Choosing
                // "Nearest" while resting never moves him - he is already
                // in a room, and re-resolving could only churn.)
                auto* newHome = GetRide(roomId);
                if (newHome != nullptr && RideStaffing::IsBreakRoom(*newHome) && newHome->status != RideStatus::closed
                    && RideStaffing::CountWorkersOnBreak(*newHome) < RideStaffing::kBreakRoomCapacity)
                {
                    workerLeaveBreakRoom();
                    workerBeginBreakTrip(newHome->id);
                }
            }
            return;
        }

        if (!breakTripRoom.IsNull() && state == PeepState::headingToInspection)
        {
            // Turn the trip around to the newly chosen room (nearest
            // resolution for the "Nearest" choice). No room can take him:
            // abandon the trip - the heading update settles him as a
            // traveller whose destination went away.
            auto* target = RideStaffing::FindBreakRoomTripTarget(*this);
            if (target == nullptr)
            {
                breakTripRoom = RideId::GetNull();
                setState(PeepState::falling);
            }
            else
            {
                breakTripRoom = target->id;
            }
        }
    }

    // Applies an assignment change that setWorkerAssignment deferred while
    // this worker was walking out of their old ride. Called from every exit
    // of updateWorkerEgress, so a recorded change is applied exactly once,
    // whichever way the walk ends. The pathfind goal is the caller's
    // business: the completion exit resets it as part of resuming patrol,
    // and the recovery exits hand the worker to stock recovery, which
    // re-goals him when he is next dispatched.
    void Staff::applyDeferredAssignment()
    {
        if (!hasDeferredAssignment)
            return;

        hasDeferredAssignment = false;
        currentRide = deferredAssignment;
        deferredAssignment = RideId::GetNull();
        currentRideStation = StationIndex::FromUnderlying(0);
    }

    /** rct2: 0x00992A5C */
    static constexpr CoordsXY kWateringUseOffsets[] = {
        { 3, 16 }, { 16, 29 }, { 29, 16 }, { 16, 3 }, { 3, 29 }, { 29, 29 }, { 29, 3 }, { 3, 3 },
    };

    /**
     *
     *  rct2: 0x006BF483
     */
    bool Staff::updatePatrollingFindWatering()
    {
        if (!(staffOrders & STAFF_ORDERS_WATER_FLOWERS))
            return false;

        uint8_t chosen_position = ScenarioRand() & 7;
        for (int32_t i = 0; i < 8; ++i, ++chosen_position)
        {
            chosen_position &= 7;

            auto chosenLoc = CoordsXY{ nextLoc } + CoordsDirectionDelta[chosen_position];

            TileElement* tile_element = MapGetFirstElementAt(chosenLoc);

            // This seems to happen in some SV4 files.
            if (tile_element == nullptr)
            {
                continue;
            }

            do
            {
                if (tile_element->getType() != TileElementType::smallScenery)
                {
                    continue;
                }

                auto z_diff = abs(nextLoc.z - tile_element->getBaseZ());

                if (z_diff >= 4 * kCoordsZStep)
                {
                    continue;
                }

                auto* sceneryEntry = tile_element->asSmallScenery()->getEntry();

                if (sceneryEntry == nullptr || !sceneryEntry->flags.has(SmallSceneryFlag::canBeWatered))
                {
                    continue;
                }

                if (tile_element->asSmallScenery()->getAge() < kSceneryWitherAgeThreshold2)
                {
                    if (chosen_position >= 4)
                    {
                        continue;
                    }

                    if (tile_element->asSmallScenery()->getAge() < kSceneryWitherAgeThreshold1)
                    {
                        continue;
                    }
                }

                if (isHandymanAlreadyServicingTile(chosenLoc, PeepState::watering))
                    continue;

                setState(PeepState::watering);
                var37 = chosen_position;

                subState = 0;
                auto destination = kWateringUseOffsets[chosen_position] + getLocation().toTileStart();
                setDestination(destination, 3);

                return true;
            } while (!(tile_element++)->isLastForTile());
        }
        return false;
    }

    /**
     *
     *  rct2: 0x006BF3A1
     */
    bool Staff::updatePatrollingFindBin()
    {
        if (!(staffOrders & STAFF_ORDERS_EMPTY_BINS))
            return false;

        if (getNextIsSurface())
            return false;

        TileElement* tileElement = MapGetFirstElementAt(nextLoc);
        if (tileElement == nullptr)
            return false;

        for (;; tileElement++)
        {
            if (tileElement->getType() == TileElementType::path && (tileElement->getBaseZ() == nextLoc.z))
                break;

            if (tileElement->isLastForTile())
                return false;
        }

        if (!tileElement->asPath()->hasAddition())
            return false;
        auto* pathAddEntry = tileElement->asPath()->getAdditionEntry();
        if (pathAddEntry == nullptr)
            return false;

        if (!pathAddEntry->flags.has(PathAdditionFlag::isBin))
            return false;

        if (tileElement->asPath()->isBroken())
            return false;

        if (tileElement->asPath()->additionIsGhost())
            return false;

        uint8_t bin_positions = tileElement->asPath()->getEdges();
        uint8_t bin_quantity = tileElement->asPath()->getAdditionStatus();
        uint8_t chosen_position = 0;

        for (; chosen_position < 4; ++chosen_position)
        {
            if (!(bin_positions & 1) && !(bin_quantity & 3))
                break;
            bin_positions >>= 1;
            bin_quantity >>= 2;
        }

        if (chosen_position == 4)
            return false;

        if (isHandymanAlreadyServicingTile(CoordsXY{ nextLoc }, PeepState::emptyingBin))
            return false;

        var37 = chosen_position;
        setState(PeepState::emptyingBin);

        subState = 0;
        auto destination = BinUseOffsets[chosen_position] + getLocation().toTileStart();
        setDestination(destination, 3);
        return true;
    }

    /**
     *
     *  rct2: 0x006BF322
     */
    bool Staff::updatePatrollingFindGrass()
    {
        if (!(staffOrders & STAFF_ORDERS_MOWING))
            return false;

        if (staffMowingTimeout < 12)
            return false;

        if (!(getNextIsSurface()))
            return false;

        auto surfaceElement = MapGetSurfaceElementAt(nextLoc);
        if (surfaceElement != nullptr && surfaceElement->canGrassGrow())
        {
            if ((surfaceElement->getGrassLength() & 0x7) >= GRASS_LENGTH_CLEAR_1
                && !isHandymanAlreadyServicingTile(CoordsXY{ nextLoc }, PeepState::mowing))
            {
                setState(PeepState::mowing);
                var37 = 0;
                // Original code used .y for both x and y. Changed to .x to make more sense (both x and y are 28)

                auto destination = kMowingWaypoints[0] + nextLoc;
                setDestination(destination, 3);
                return true;
            }
        }
        return false;
    }

    /**
     *
     *  rct2: 0x006BF295
     */
    bool Staff::updatePatrollingFindSweeping()
    {
        if (!(staffOrders & STAFF_ORDERS_SWEEPING))
            return false;
        auto quad = EntityTileList<Litter>({ x, y });
        for (auto litter : quad)
        {
            uint16_t z_diff = abs(z - litter->z);

            if (z_diff >= 16)
                continue;

            if (isHandymanAlreadyServicingTile(litter->getLocation(), PeepState::sweeping))
                continue;

            setState(PeepState::sweeping);

            var37 = 0;
            setDestination(litter->getLocation(), 5);
            return true;
        }

        return false;
    }

    bool Staff::securityGuardPathIsCrowded() const
    {
        // Iterate over tiles within a 3-tile radius (96 units)
        constexpr auto kTileRadius = 3;
        constexpr auto kLookupRadius = kCoordsXYStep * kTileRadius;
        constexpr auto kSecurityPathCrowdedThreshold = 20;

        int16_t guestCount = 0;

        for (int32_t tileX = x - kLookupRadius; tileX <= x + kLookupRadius; tileX += kCoordsXYStep)
        {
            for (int32_t tileY = y - kLookupRadius; tileY <= y + kLookupRadius; tileY += kCoordsXYStep)
            {
                for (auto* guest : EntityTileList<Guest>({ tileX, tileY }))
                {
                    if (guest->x == kLocationNull)
                        continue;

                    int16_t zDist = std::abs(z - guest->z);
                    if (zDist > kTileRadius / 2)
                        continue;

                    int16_t xDist = std::abs(x - guest->x);
                    if (xDist > kLookupRadius)
                        continue;

                    int16_t yDist = std::abs(y - guest->y);
                    if (yDist > kLookupRadius)
                        continue;

                    if (!guest->isActionWalking())
                        continue;

                    guestCount++;
                    if (guestCount >= kSecurityPathCrowdedThreshold)
                        return true;
                }
            }
        }

        return false;
    }

    void Staff::tick128UpdateStaff()
    {
        if (isWorker())
        {
            tick128UpdateWorker();
            return;
        }

        if (assignedStaffType != StaffType::security)
            return;

        // Alternate between walking animations based on crowd size
        auto newAnimationGroup = PeepAnimationGroup::normal;
        if (state == PeepState::patrolling && securityGuardPathIsCrowded())
            newAnimationGroup = PeepAnimationGroup::alternate;

        if (animationGroup == newAnimationGroup)
            return;

        animationGroup = newAnimationGroup;
        animationImageIdOffset = 0;
        walkingAnimationFrameNum = 0;
        if (action < PeepActionType::idle)
            action = PeepActionType::walking;

        auto& objManager = GetContext()->GetObjectManager();
        auto* animObj = objManager.GetLoadedObject<PeepAnimationsObject>(animationObjectIndex);

        // NB: security staff have two animations groups: one regular, and one slow-walking
        peepFlags.unset(PeepFlag::slowWalk);
        if (animObj->IsSlowWalking(newAnimationGroup))
        {
            peepFlags.set(PeepFlag::slowWalk);
        }

        animationType = PeepAnimationType::invalid;
        updateCurrentAnimationType();
    }

    bool Staff::isMechanic() const
    {
        return assignedStaffType == StaffType::mechanic;
    }

    bool Staff::isEntertainer() const
    {
        return assignedStaffType == StaffType::entertainer;
    }

    bool Staff::isWorker() const
    {
        return assignedStaffType == StaffType::worker;
    }

    void Staff::update()
    {
        if (peepFlags.has(PeepFlag::positionFrozen))
        {
            if (!peepFlags.has(PeepFlag::animationFrozen))
            {
                // This is circumventing other logic, so only update every few ticks
                if ((getGameState().currentTicks & 3) == 0)
                {
                    if (isActionWalking())
                        updateWalkingAnimation();
                    else
                        updateActionAnimation();
                    invalidate();
                }
            }
            return;
        }
        else if (peepFlags.has(PeepFlag::animationFrozen))
        {
            // Animation is frozen while position is not. This allows a peep to walk
            // around without its sprite being updated, which looks very glitchy.
            // We'll just remove the flag and continue as normal, in this case.
            peepFlags.unset(PeepFlag::animationFrozen);
        }

        // Walking speed logic
        const auto stepsToTake = getStepsToTake();
        const auto carryCheck = stepProgress + stepsToTake;
        stepProgress = carryCheck;

        if (carryCheck <= 255)
        {
            // No-op: Keep replay working for now, can be eliminate with a replay update.
        }
        else
        {
            // Loc68FD2F
            switch (state)
            {
                case PeepState::falling:
                    updateFalling();
                    break;
                case PeepState::one:
                    update1();
                    break;
                case PeepState::onRide:
                    // No action
                    break;
                case PeepState::picked:
                    updatePicked();
                    break;
                case PeepState::patrolling:
                    updatePatrolling();
                    break;
                case PeepState::mowing:
                    updateMowing();
                    break;
                case PeepState::sweeping:
                    updateSweeping();
                    break;
                case PeepState::answering:
                    updateAnswering();
                    break;
                case PeepState::fixing:
                    updateFixing(stepsToTake);
                    break;
                case PeepState::inspecting:
                    updateFixing(stepsToTake);
                    break;
                case PeepState::emptyingBin:
                    updateEmptyingBin();
                    break;
                case PeepState::watering:
                    updateWatering();
                    break;
                case PeepState::headingToInspection:
                    if (isWorker())
                        updateWorkerHeadingToPost();
                    else
                        updateHeadingToInspect();
                    break;
                case PeepState::operating:
                    updateOperating();
                    break;
                case PeepState::onBreak:
                    updateOnBreak();
                    break;
                default:
                    // TODO reset to default state
                    assert(false);
                    break;
            }
        }
    }

    /**
     *
     *  rct2: 0x006BF1FD
     */
    void Staff::updatePatrolling()
    {
        if (!checkForPath())
            return;

        if (shouldWaitForLevelCrossing() && !isMechanicHeadingToFixRideBlockingPath())
        {
            updateWaitingAtCrossing();
            return;
        }

        const auto [pathingResult, _] = performNextAction();
        if (!(pathingResult & PATHING_DESTINATION_REACHED))
            return;

        if (getNextIsSurface())
        {
            auto surfaceElement = MapGetSurfaceElementAt(nextLoc);

            if (surfaceElement != nullptr)
            {
                int32_t water_height = surfaceElement->getWaterHeight();
                if (water_height > 0)
                {
                    moveTo({ x, y, water_height });
                    setState(PeepState::falling);
                    return;
                }
            }
        }

        if (assignedStaffType != StaffType::handyman)
            return;

        if (updatePatrollingFindSweeping())
            return;

        if (updatePatrollingFindGrass())
            return;

        if (updatePatrollingFindBin())
            return;

        updatePatrollingFindWatering();
    }

    enum
    {
        PEEP_FIXING_ENTER_STATION = 0,
        PEEP_FIXING_MOVE_TO_BROKEN_DOWN_VEHICLE = 1,
        PEEP_FIXING_FIX_VEHICLE_CLOSED_RESTRAINTS = 2,
        PEEP_FIXING_FIX_VEHICLE_CLOSED_DOORS = 3,
        PEEP_FIXING_FIX_VEHICLE_OPEN_RESTRAINTS = 4,
        PEEP_FIXING_FIX_VEHICLE_OPEN_DOORS = 5,
        PEEP_FIXING_FIX_VEHICLE_MALFUNCTION = 6,
        PEEP_FIXING_MOVE_TO_STATION_END = 7,
        PEEP_FIXING_FIX_STATION_END = 8,
        PEEP_FIXING_MOVE_TO_STATION_START = 9,
        PEEP_FIXING_FIX_STATION_START = 10,
        PEEP_FIXING_FIX_STATION_BRAKES = 11,
        PEEP_FIXING_MOVE_TO_STATION_EXIT = 12,
        PEEP_FIXING_FINISH_FIX_OR_INSPECT = 13,
        PEEP_FIXING_LEAVE_BY_ENTRANCE_EXIT = 14,

        // Two-tier breakdowns, major tier: the parts run, inside fixing.
        // Values sit above every stock substate - subState streams raw in
        // saves, so stock values never move. The detour is entered
        // deliberately from updateFixing's advancement logic, never by
        // mask order alone (see kFixingPartsRunSubstates).
        PEEP_FIXING_LEAVE_PLATFORM_FOR_PARTS = 15,
        PEEP_FIXING_GO_TO_PARTS = 16,
        PEEP_FIXING_FETCH_PARTS = 17,
        PEEP_FIXING_RETURN_FROM_PARTS = 18,
        PEEP_FIXING_REENTER_STATION = 19,
    };

    // The parts-run detour substates, OR-ed into a breakdown's sequence
    // mask when the breakdown is major (RideFlag::breakdownMajor). They
    // sort above the stock substates, so mask advancement on its own can
    // never wander from a stock beat into the detour - and once the
    // re-enter substate hands the sequence back to the remaining stock
    // beats, advancement steps over the stock beats only, because the
    // finish and leave substates (13, 14) end the job below them.
    static constexpr uint32_t kFixingPartsRunSubstates = (1 << PEEP_FIXING_LEAVE_PLATFORM_FOR_PARTS)
        | (1 << PEEP_FIXING_GO_TO_PARTS) | (1 << PEEP_FIXING_FETCH_PARTS) | (1 << PEEP_FIXING_RETURN_FROM_PARTS)
        | (1 << PEEP_FIXING_REENTER_STATION);

    // The substates that perform a fix, as opposed to moving, fetching or
    // finishing: a major breakdown's parts run is entered when the first
    // of these completes.
    static bool FixingSubstateIsFixBeat(int32_t subState)
    {
        switch (subState)
        {
            case PEEP_FIXING_FIX_VEHICLE_CLOSED_RESTRAINTS:
            case PEEP_FIXING_FIX_VEHICLE_CLOSED_DOORS:
            case PEEP_FIXING_FIX_VEHICLE_OPEN_RESTRAINTS:
            case PEEP_FIXING_FIX_VEHICLE_OPEN_DOORS:
            case PEEP_FIXING_FIX_VEHICLE_MALFUNCTION:
            case PEEP_FIXING_FIX_STATION_END:
            case PEEP_FIXING_FIX_STATION_START:
            case PEEP_FIXING_FIX_STATION_BRAKES:
                return true;
            default:
                return false;
        }
    }

    /**
     * FixingSubstatesForBreakdown[] defines the applicable peep sub_states for
     * mechanics fixing a ride. The array is indexed by breakdownReason:
     * - indexes 0-7 are the 8 breakdown reasons (see BREAKDOWN_* in Ride.h)
     *   when fixing a broken down ride;
     * - index 8 is for inspecting a ride.
     */
    // clang-format off
    static constexpr uint32_t FixingSubstatesForBreakdown[9] = {
        ( // Breakdown::safetyCutOut
            (1 << PEEP_FIXING_MOVE_TO_STATION_END) |
            (1 << PEEP_FIXING_FIX_STATION_END) |
            (1 << PEEP_FIXING_MOVE_TO_STATION_START) |
            (1 << PEEP_FIXING_FIX_STATION_START) |
            (1 << PEEP_FIXING_MOVE_TO_STATION_EXIT) |
            (1 << PEEP_FIXING_FINISH_FIX_OR_INSPECT) |
            (1 << PEEP_FIXING_LEAVE_BY_ENTRANCE_EXIT)
        ),
        ( // Breakdown::restraintsStuckClosed
            (1 << PEEP_FIXING_MOVE_TO_BROKEN_DOWN_VEHICLE) |
            (1 << PEEP_FIXING_FIX_VEHICLE_CLOSED_RESTRAINTS) |
            (1 << PEEP_FIXING_MOVE_TO_STATION_EXIT) |
            (1 << PEEP_FIXING_FINISH_FIX_OR_INSPECT) |
            (1 << PEEP_FIXING_LEAVE_BY_ENTRANCE_EXIT)
        ),
        ( // Breakdown::restraintsStuckOpen
            (1 << PEEP_FIXING_MOVE_TO_BROKEN_DOWN_VEHICLE) |
            (1 << PEEP_FIXING_FIX_VEHICLE_OPEN_RESTRAINTS) |
            (1 << PEEP_FIXING_MOVE_TO_STATION_EXIT) |
            (1 << PEEP_FIXING_FINISH_FIX_OR_INSPECT) |
            (1 << PEEP_FIXING_LEAVE_BY_ENTRANCE_EXIT)
        ),
        ( // Breakdown::doorsStuckClosed
            (1 << PEEP_FIXING_MOVE_TO_BROKEN_DOWN_VEHICLE) |
            (1 << PEEP_FIXING_FIX_VEHICLE_CLOSED_DOORS) |
            (1 << PEEP_FIXING_MOVE_TO_STATION_EXIT) |
            (1 << PEEP_FIXING_FINISH_FIX_OR_INSPECT) |
            (1 << PEEP_FIXING_LEAVE_BY_ENTRANCE_EXIT)
        ),
        ( // Breakdown::doorsStuckOpen
            (1 << PEEP_FIXING_MOVE_TO_BROKEN_DOWN_VEHICLE) |
            (1 << PEEP_FIXING_FIX_VEHICLE_OPEN_DOORS) |
            (1 << PEEP_FIXING_MOVE_TO_STATION_EXIT) |
            (1 << PEEP_FIXING_FINISH_FIX_OR_INSPECT) |
            (1 << PEEP_FIXING_LEAVE_BY_ENTRANCE_EXIT)
        ),
        ( // Breakdown::vehicleMalfunction
            (1 << PEEP_FIXING_MOVE_TO_BROKEN_DOWN_VEHICLE) |
            (1 << PEEP_FIXING_FIX_VEHICLE_MALFUNCTION) |
            (1 << PEEP_FIXING_MOVE_TO_STATION_EXIT) |
            (1 << PEEP_FIXING_FINISH_FIX_OR_INSPECT) |
            (1 << PEEP_FIXING_LEAVE_BY_ENTRANCE_EXIT)
        ),
        ( // Breakdown::brakesFailure
            (1 << PEEP_FIXING_MOVE_TO_STATION_START) |
            (1 << PEEP_FIXING_FIX_STATION_BRAKES) |
            (1 << PEEP_FIXING_MOVE_TO_STATION_EXIT) |
            (1 << PEEP_FIXING_FINISH_FIX_OR_INSPECT) |
            (1 << PEEP_FIXING_LEAVE_BY_ENTRANCE_EXIT)
        ),
        ( // Breakdown::controlFailure
            (1 << PEEP_FIXING_MOVE_TO_STATION_END) |
            (1 << PEEP_FIXING_FIX_STATION_END) |
            (1 << PEEP_FIXING_MOVE_TO_STATION_START) |
            (1 << PEEP_FIXING_FIX_STATION_START) |
            (1 << PEEP_FIXING_MOVE_TO_STATION_EXIT) |
            (1 << PEEP_FIXING_FINISH_FIX_OR_INSPECT) |
            (1 << PEEP_FIXING_LEAVE_BY_ENTRANCE_EXIT)
        ),
        ( // INSPECTION
            (1 << PEEP_FIXING_MOVE_TO_STATION_END) |
            (1 << PEEP_FIXING_FIX_STATION_END) |
            (1 << PEEP_FIXING_MOVE_TO_STATION_START) |
            (1 << PEEP_FIXING_FIX_STATION_START) |
            (1 << PEEP_FIXING_MOVE_TO_STATION_EXIT) |
            (1 << PEEP_FIXING_FINISH_FIX_OR_INSPECT) |
            (1 << PEEP_FIXING_LEAVE_BY_ENTRANCE_EXIT)
        ),
    };
    // clang-format on

    /**
     *
     *  rct2: 0x006C0E8B
     * Also used by inspecting.
     */
    void Staff::updateFixing(int32_t steps)
    {
        auto ride = GetRide(currentRide);
        if (ride == nullptr)
        {
            setState(PeepState::falling);
            return;
        }

        bool progressToNextSubstate = true;
        bool firstRun = true;

        if ((state == PeepState::inspecting) && (ride->flags.hasAny(RideFlag::breakdownPending, RideFlag::brokenDown)))
        {
            // Ride has broken down since Mechanic was called to inspect it.
            // Mechanic identifies the breakdown and switches to fixing it.
            state = PeepState::fixing;
        }

        while (progressToNextSubstate)
        {
            switch (subState)
            {
                case PEEP_FIXING_ENTER_STATION:
                    nextFlags &= ~PEEP_NEXT_FLAG_IS_SLOPED;
                    progressToNextSubstate = updateFixingEnterStation(*ride);
                    break;

                case PEEP_FIXING_MOVE_TO_BROKEN_DOWN_VEHICLE:
                    progressToNextSubstate = updateFixingMoveToBrokenDownVehicle(firstRun, *ride);
                    break;

                case PEEP_FIXING_FIX_VEHICLE_CLOSED_RESTRAINTS:
                case PEEP_FIXING_FIX_VEHICLE_CLOSED_DOORS:
                case PEEP_FIXING_FIX_VEHICLE_OPEN_RESTRAINTS:
                case PEEP_FIXING_FIX_VEHICLE_OPEN_DOORS:
                    progressToNextSubstate = updateFixingFixVehicle(firstRun, *ride);
                    break;

                case PEEP_FIXING_FIX_VEHICLE_MALFUNCTION:
                    progressToNextSubstate = updateFixingFixVehicleMalfunction(firstRun, *ride);
                    break;

                case PEEP_FIXING_MOVE_TO_STATION_END:
                    progressToNextSubstate = updateFixingMoveToStationEnd(firstRun, *ride);
                    break;

                case PEEP_FIXING_FIX_STATION_END:
                    progressToNextSubstate = updateFixingFixStationEnd(firstRun);
                    break;

                case PEEP_FIXING_MOVE_TO_STATION_START:
                    progressToNextSubstate = updateFixingMoveToStationStart(firstRun, *ride);
                    break;

                case PEEP_FIXING_FIX_STATION_START:
                    progressToNextSubstate = updateFixingFixStationStart(firstRun, *ride);
                    break;

                case PEEP_FIXING_FIX_STATION_BRAKES:
                    progressToNextSubstate = updateFixingFixStationBrakes(firstRun, *ride);
                    break;

                case PEEP_FIXING_MOVE_TO_STATION_EXIT:
                    progressToNextSubstate = updateFixingMoveToStationExit(firstRun, *ride);
                    break;

                case PEEP_FIXING_FINISH_FIX_OR_INSPECT:
                    progressToNextSubstate = updateFixingFinishFixOrInspect(firstRun, steps, *ride);
                    break;

                case PEEP_FIXING_LEAVE_BY_ENTRANCE_EXIT:
                    progressToNextSubstate = updateFixingLeaveByEntranceExit(firstRun, *ride);
                    break;

                case PEEP_FIXING_LEAVE_PLATFORM_FOR_PARTS:
                    progressToNextSubstate = updateFixingLeavePlatformForParts(firstRun, *ride);
                    break;

                case PEEP_FIXING_GO_TO_PARTS:
                    progressToNextSubstate = updateFixingGoToParts(firstRun);
                    break;

                case PEEP_FIXING_FETCH_PARTS:
                    progressToNextSubstate = updateFixingFetchParts(firstRun);
                    break;

                case PEEP_FIXING_RETURN_FROM_PARTS:
                    progressToNextSubstate = updateFixingReturnFromParts(firstRun, *ride);
                    break;

                case PEEP_FIXING_REENTER_STATION:
                    progressToNextSubstate = updateFixingReenterStation(firstRun, *ride);
                    break;

                default:
                    LOG_ERROR("Invalid substate");
                    progressToNextSubstate = false;
            }

            firstRun = false;

            if (!progressToNextSubstate)
            {
                break;
            }

            int32_t newSubState = subState;
            uint32_t sub_state_sequence_mask = FixingSubstatesForBreakdown[8];

            if (state != PeepState::inspecting)
            {
                sub_state_sequence_mask = FixingSubstatesForBreakdown[EnumValue(ride->breakdownReasonPending)];
                if (ride->flags.has(RideFlag::breakdownMajor))
                {
                    sub_state_sequence_mask |= kFixingPartsRunSubstates;
                }
            }

            // Two-tier breakdowns: on a major breakdown the first fix
            // beat is followed by the parts run, not by the remaining
            // stock beats. The detour substates sort above the stock
            // ones, so the mask scan below would step past the remaining
            // beats first - enter the detour deliberately instead. Once
            // the parts are fetched (RideFlag::breakdownPartsFetched,
            // set by the re-enter substate) the stock sequence resumes
            // and this cannot fire again; on a minor breakdown the tier
            // bit is unset and the sequence is stock, byte for byte.
            if (state == PeepState::fixing && ride->flags.has(RideFlag::breakdownMajor)
                && !ride->flags.has(RideFlag::breakdownPartsFetched) && subState < PEEP_FIXING_LEAVE_PLATFORM_FOR_PARTS
                && FixingSubstateIsFixBeat(subState))
            {
                subState = PEEP_FIXING_LEAVE_PLATFORM_FOR_PARTS;
                continue;
            }

            do
            {
                newSubState++;
            } while ((sub_state_sequence_mask & (1 << newSubState)) == 0);

            subState = newSubState & 0xFF;
        }
    }

    /**
     * rct2: 0x006C0EEC
     * fixing SubState: enter_station - applies to fixing all break down reasons and ride inspections.
     */
    bool Staff::updateFixingEnterStation(Ride& ride) const
    {
        ride.mechanicStatus = MechanicStatus::fixing;
        ride.windowInvalidateFlags.set(RideInvalidateFlag::maintenance);

        return true;
    }

    /**
     * rct2: 0x006C0F09
     * fixing SubState: move_to_broken_down_vehicle - applies to fixing all vehicle specific breakdown reasons
     * - see FixingSubstatesForBreakdown[]
     */
    bool Staff::updateFixingMoveToBrokenDownVehicle(bool firstRun, const Ride& ride)
    {
        if (!firstRun)
        {
            Vehicle* vehicle = RideGetBrokenVehicle(ride);
            if (vehicle == nullptr)
            {
                return true;
            }

            while (true)
            {
                if (vehicle->IsHead())
                {
                    break;
                }

                auto trackType = vehicle->GetTrackType();
                if (trackTypeIsStation(trackType))
                {
                    break;
                }

                vehicle = getGameState().entities.getEntity<Vehicle>(vehicle->prev_vehicle_on_ride);
                if (vehicle == nullptr)
                {
                    return true;
                }
            }

            CoordsXY offset = DirectionOffsets[peepDirection];
            auto destination = (offset * -12) + vehicle->getLocation();
            setDestination(destination, 2);
        }

        if (auto loc = updateAction(); loc.has_value())
        {
            moveTo({ loc.value(), z });
            return false;
        }

        return true;
    }

    /**
     * rct2: 0x006C0FD3
     * fixing SubState: fix_vehicle - applies to fixing vehicle with:
     * 1. restraints stuck closed,
     * 2. doors stuck closed,
     * 3. restrains stuck open,
     * 4. doors stuck open.
     * - see FixingSubstatesForBreakdown[]
     */
    bool Staff::updateFixingFixVehicle(bool firstRun, const Ride& ride)
    {
        if (!firstRun)
        {
            orientation = peepDirection << 3;

            action = (ScenarioRand() & 1) ? PeepActionType::staffFix2 : PeepActionType::staffFix;
            animationImageIdOffset = 0;
            animationFrameNum = 0;
            updateCurrentAnimationType();
        }

        if (isActionWalking())
        {
            return true;
        }

        updateAction();
        invalidate();

        uint8_t actionFrame = (action == PeepActionType::staffFix) ? 0x25 : 0x50;
        if (animationFrameNum != actionFrame)
        {
            return false;
        }

        Vehicle* vehicle = RideGetBrokenVehicle(ride);
        if (vehicle == nullptr)
        {
            return true;
        }

        vehicle->flags.unset(VehicleFlag::carIsBroken);

        return false;
    }

    /**
     * rct2: 0x006C107B
     * fixing SubState: fix_vehicle_malfunction - applies fixing to vehicle malfunction.
     * - see FixingSubstatesForBreakdown[]
     */
    bool Staff::updateFixingFixVehicleMalfunction(bool firstRun, const Ride& ride)
    {
        if (!firstRun)
        {
            orientation = peepDirection << 3;
            action = PeepActionType::staffFix3;
            animationImageIdOffset = 0;
            animationFrameNum = 0;

            updateCurrentAnimationType();
        }

        if (isActionWalking())
        {
            return true;
        }

        updateAction();
        invalidate();

        if (animationFrameNum != 0x65)
        {
            return false;
        }

        Vehicle* vehicle = RideGetBrokenVehicle(ride);
        if (vehicle == nullptr)
        {
            return true;
        }

        vehicle->flags.unset(VehicleFlag::trainIsBroken);

        return false;
    }

    /** rct2: 0x00992A3C */
    static constexpr CoordsXY kStationFixingOffsets[] = {
        { -12, 0 },
        { 0, 12 },
        { 12, 0 },
        { 0, -12 },
    };

    /**
     * rct2: 0x006C1114
     * fixing SubState: move_to_station_end - applies to fixing station specific breakdowns: safety cut-out, control failure,
     * inspection.
     * - see FixingSubstatesForBreakdown[]
     */
    bool Staff::updateFixingMoveToStationEnd(bool firstRun, const Ride& ride)
    {
        if (!firstRun)
        {
            if (ride.getRideTypeDescriptor().flags.has(RtdFlag::hasSinglePieceStation)
                || !ride.getRideTypeDescriptor().flags.has(RtdFlag::hasTrack))
            {
                return true;
            }

            auto stationPos = ride.getStation(currentRideStation).getStart();
            if (stationPos.isNull())
            {
                return true;
            }

            auto tileElement = MapGetTrackElementAt(stationPos);
            if (tileElement == nullptr)
            {
                LOG_ERROR("Couldn't find tile_element");
                return false;
            }

            int32_t trackDirection = tileElement->getDirection();
            CoordsXY offset = kStationFixingOffsets[trackDirection];

            stationPos.x += 16 + offset.x;
            if (offset.x == 0)
            {
                stationPos.x = getDestination().x;
            }

            stationPos.y += 16 + offset.y;
            if (offset.y == 0)
            {
                stationPos.y = getDestination().y;
            }

            setDestination(stationPos, 2);
        }

        if (auto loc = updateAction(); loc.has_value())
        {
            moveTo({ loc.value(), z });
            return false;
        }

        return true;
    }

    /**
     * rct2: 0x006C11F5
     * fixing SubState: fix_station_end - applies to fixing station specific breakdowns: safety cut-out, control failure,
     * inspection.
     * - see FixingSubstatesForBreakdown[]
     */
    bool Staff::updateFixingFixStationEnd(bool firstRun)
    {
        if (!firstRun)
        {
            orientation = peepDirection << 3;
            action = PeepActionType::staffCheckBoard;
            animationFrameNum = 0;
            animationImageIdOffset = 0;

            updateCurrentAnimationType();
        }

        if (isActionWalking())
        {
            return true;
        }

        updateAction();
        invalidate();

        return false;
    }

    /**
     * rct2: 0x006C1239
     * fixing SubState: move_to_station_start
     * 1. applies to fixing station specific breakdowns: safety cut-out, control failure,
     * 2. applies to fixing brake failure,
     * 3. applies to inspection.
     * - see FixingSubstatesForBreakdown[]
     */
    bool Staff::updateFixingMoveToStationStart(bool firstRun, const Ride& ride)
    {
        if (!firstRun)
        {
            if (ride.getRideTypeDescriptor().flags.has(RtdFlag::hasSinglePieceStation)
                || !ride.getRideTypeDescriptor().flags.has(RtdFlag::hasTrack))
            {
                return true;
            }

            auto stationPosition = ride.getStation(currentRideStation).getStart();
            if (stationPosition.isNull())
            {
                return true;
            }

            CoordsXYE input;
            input.x = stationPosition.x;
            input.y = stationPosition.y;
            input.element = MapGetTrackElementAtFromRide({ input.x, input.y, stationPosition.z }, currentRide);
            if (input.element == nullptr)
            {
                return true;
            }

            Direction stationDirection = 0;
            TrackBeginEnd trackBeginEnd;
            while (trackBlockGetPrevious(input, &trackBeginEnd))
            {
                if (trackBeginEnd.begin_element->asTrack()->isStation())
                {
                    input.x = trackBeginEnd.begin_x;
                    input.y = trackBeginEnd.begin_y;
                    input.element = trackBeginEnd.begin_element;

                    stationDirection = trackBeginEnd.begin_element->getDirection();
                    continue;
                }

                break;
            }

            // Loc6C12ED:
            auto destination = CoordsXY{ input.x + 16, input.y + 16 };
            auto offset = kStationFixingOffsets[stationDirection];

            destination.x -= offset.x;
            if (offset.x == 0)
            {
                destination.x = getDestination().x;
            }

            destination.y -= offset.y;
            if (offset.y == 0)
            {
                destination.y = getDestination().y;
            }

            setDestination(destination, 2);
        }

        if (auto loc = updateAction(); loc.has_value())
        {
            moveTo({ loc.value(), z });
            return false;
        }

        return true;
    }

    /**
     * rct2: 0x006C1368
     * fixing SubState: fix_station_start
     * 1. applies to fixing station specific breakdowns: safety cut-out, control failure,
     * 2. applies to inspection.
     * - see FixingSubstatesForBreakdown[]
     */
    bool Staff::updateFixingFixStationStart(bool firstRun, const Ride& ride)
    {
        if (!firstRun)
        {
            if (ride.getRideTypeDescriptor().flags.has(RtdFlag::hasSinglePieceStation)
                || !ride.getRideTypeDescriptor().flags.has(RtdFlag::hasTrack))
            {
                return true;
            }

            orientation = peepDirection << 3;

            action = PeepActionType::staffFix;
            animationFrameNum = 0;
            animationImageIdOffset = 0;

            updateCurrentAnimationType();
        }

        if (isActionWalking())
        {
            return true;
        }

        updateAction();

        return false;
    }

    /**
     * rct2: 0x006C13CE
     * fixing SubState: fix_station_brakes - applies to fixing brake failure
     * - see FixingSubstatesForBreakdown[]
     */
    bool Staff::updateFixingFixStationBrakes(bool firstRun, Ride& ride)
    {
        if (!firstRun)
        {
            orientation = peepDirection << 3;

            action = PeepActionType::staffFixGround;
            animationFrameNum = 0;
            animationImageIdOffset = 0;

            updateCurrentAnimationType();
        }

        if (isActionWalking())
        {
            return true;
        }

        updateAction();
        invalidate();

        if (animationFrameNum == 0x28)
        {
            ride.mechanicStatus = MechanicStatus::hasFixedStationBrakes;
            ride.windowInvalidateFlags.set(RideInvalidateFlag::maintenance);
        }

        if (animationFrameNum == 0x13 || animationFrameNum == 0x19 || animationFrameNum == 0x1F || animationFrameNum == 0x25
            || animationFrameNum == 0x2B)
        {
            Audio::Play3D(Audio::SoundId::mechanicFix, getLocation());
        }

        return false;
    }

    /**
     * rct2: 0x006C1474
     * fixing SubState: move_to_station_exit - applies to fixing all failures & inspections
     * - see FixingSubstatesForBreakdown[]
     */
    bool Staff::updateFixingMoveToStationExit(bool firstRun, const Ride& ride)
    {
        if (!firstRun)
        {
            auto stationPosition = ride.getStation(currentRideStation).exit.toCoordsXY();
            if (stationPosition.isNull())
            {
                stationPosition = ride.getStation(currentRideStation).entrance.toCoordsXY();

                if (stationPosition.isNull())
                {
                    return true;
                }
            }

            stationPosition = stationPosition.toTileCentre();

            CoordsXY stationPlatformDirection = DirectionOffsets[peepDirection];
            stationPosition.x += stationPlatformDirection.x * 20;
            stationPosition.y += stationPlatformDirection.y * 20;

            setDestination(stationPosition, 2);
        }

        if (auto loc = updateAction(); loc.has_value())
        {
            moveTo({ loc.value(), z });
            return false;
        }

        return true;
    }

    /**
     * rct2: 0x006C1504
     * fixing SubState: finish_fix_or_inspect - applies to fixing all failures & inspections
     * - see FixingSubstatesForBreakdown[]
     */
    bool Staff::updateFixingFinishFixOrInspect(bool firstRun, int32_t steps, Ride& ride)
    {
        if (!firstRun)
        {
            if (state == PeepState::inspecting)
            {
                updateRideInspected(currentRide);

                staffRidesInspected = AddClamp(staffRidesInspected, 1u);
                windowInvalidateFlags |= PEEP_INVALIDATE_STAFF_STATS;
                ride.mechanicStatus = MechanicStatus::undefined;
                return true;
            }

            staffRidesFixed = AddClamp(staffRidesFixed, 1u);
            windowInvalidateFlags |= PEEP_INVALIDATE_STAFF_STATS;

            orientation = peepDirection << 3;
            action = PeepActionType::staffAnswerCall2;
            animationFrameNum = 0;
            animationImageIdOffset = 0;

            updateCurrentAnimationType();
        }

        if (!isActionWalking())
        {
            updateAction();
            invalidate();
            return false;
        }

        RideFixBreakdown(ride, steps);
        ride.mechanicStatus = MechanicStatus::undefined;
        return true;
    }

    /**
     * rct2: 0x006C157E
     * fixing SubState: leave_by_entrance_exit - applies to fixing all failures & inspections
     * - see FixingSubstatesForBreakdown[]
     */
    bool Staff::updateFixingLeaveByEntranceExit(bool firstRun, const Ride& ride)
    {
        if (!firstRun)
        {
            auto exitPosition = ride.getStation(currentRideStation).exit.toCoordsXY();
            if (exitPosition.isNull())
            {
                exitPosition = ride.getStation(currentRideStation).entrance.toCoordsXY();

                if (exitPosition.isNull())
                {
                    setState(PeepState::falling);
                    return false;
                }
            }

            exitPosition = exitPosition.toTileCentre();

            CoordsXY ebx_direction = DirectionOffsets[peepDirection];
            exitPosition.x -= ebx_direction.x * 19;
            exitPosition.y -= ebx_direction.y * 19;

            setDestination(exitPosition, 2);
        }

        int16_t xy_distance;
        if (auto loc = updateAction(xy_distance); loc.has_value())
        {
            auto stationHeight = ride.getStation(currentRideStation).getBaseZ();
            if (xy_distance >= 16)
            {
                stationHeight += ride.getRideTypeDescriptor().Heights.PlatformHeight;
            }

            moveTo({ loc.value(), stationHeight });
            return false;
        }
        setState(PeepState::falling);
        return false;
    }

    // The parts source for a major breakdown's parts run (two-tier
    // breakdowns): the park entrance nearest to hand. It always exists
    // and is pathable by definition - staff spawn there - so v1 needs no
    // placeable workshop and no new save state for the destination.
    static std::optional<TileCoordsXYZD> GetNearestParkEntrance(const CoordsXY& from)
    {
        std::optional<TileCoordsXYZD> best;
        int64_t bestDistance = 0;
        const auto fromTile = TileCoordsXY{ from };
        for (const auto& entrance : getGameState().park.entrances)
        {
            const int64_t dx = entrance.x - fromTile.x;
            const int64_t dy = entrance.y - fromTile.y;
            const int64_t distance = dx * dx + dy * dy;
            if (!best.has_value() || distance < bestDistance)
            {
                best = entrance;
                bestDistance = distance;
            }
        }
        return best;
    }

    bool Staff::isOnPartsRun() const
    {
        return state == PeepState::fixing && (subState == PEEP_FIXING_GO_TO_PARTS || subState == PEEP_FIXING_RETURN_FROM_PARTS);
    }

    // Pathfinding for the parts run's two network walks, modelled on the
    // worker post-trip pair (doWorkerPathFinding / workerDirectionPath):
    // same step choice, same destination tolerance draw as every other
    // staff trip - the run adds no new random expressions, it reuses the
    // staff pathfinding vocabulary wholesale.
    bool Staff::doPartsRunPathFinding()
    {
        uint8_t validDirections = getValidPatrolDirections(nextLoc);
        Direction newDirection = kInvalidDirection;
        if (getNextIsSurface())
        {
            newDirection = mechanicDirectionSurface();
        }
        else
        {
            auto* pathElement = MapGetPathElementAt(TileCoordsXYZ{ nextLoc });
            if (pathElement == nullptr)
                return true;

            newDirection = partsRunDirectionPath(validDirections, pathElement);
        }

        assert(DirectionValid(newDirection));

        CoordsXY chosenTile = CoordsXY{ nextLoc } + CoordsDirectionDelta[newDirection];

        while (!MapIsLocationValid(chosenTile))
        {
            newDirection = mechanicDirectionSurface();
            chosenTile = CoordsXY{ nextLoc } + CoordsDirectionDelta[newDirection];
        }

        peepDirection = newDirection;
        auto tolerance = (ScenarioRand() & 7) + 2;
        setDestination(chosenTile + CoordsXY{ 16, 16 }, tolerance);

        return false;
    }

    Direction Staff::partsRunDirectionPath(uint8_t validDirections, PathElement* pathElement)
    {
        uint32_t pathDirections = pathElement->getEdges();
        pathDirections &= validDirections;

        if (pathDirections == 0)
        {
            return mechanicDirectionSurface();
        }

        // Check if this is dead end - i.e. only way out is the reverse direction.
        pathDirections &= ~(1 << DirectionReverse(peepDirection));
        if (pathDirections == 0)
        {
            pathDirections |= (1 << DirectionReverse(peepDirection));
        }

        Direction direction = Numerics::bitScanForward(pathDirections);
        pathDirections &= ~(1 << direction);
        if (pathDirections == 0)
        {
            return direction;
        }

        pathDirections |= (1 << direction);

        // The goal: outbound, the parts source (the nearest park
        // entrance); returning, the assigned ride's station exit, or its
        // entrance when the station has no separate exit - the same goal
        // the stock mechanic pathfinding walks to.
        std::optional<TileCoordsXYZ> goalPos;
        if (subState == PEEP_FIXING_GO_TO_PARTS)
        {
            if (const auto entrance = GetNearestParkEntrance(CoordsXY{ x, y }); entrance.has_value())
            {
                goalPos = TileCoordsXYZ{ entrance.value() };
            }
        }
        else if (auto* ride = GetRide(currentRide); ride != nullptr)
        {
            TileCoordsXYZD location = ride->getStation(currentRideStation).exit;
            if (location.isNull())
            {
                location = ride->getStation(currentRideStation).entrance;
            }
            if (!location.isNull())
            {
                goalPos = TileCoordsXYZ{ location };
            }
        }

        if (!goalPos.has_value())
        {
            return mechanicDirectionPathRand(pathDirections);
        }

        Direction pathfindDirection = PathFinding::ChooseDirection(
            TileCoordsXYZ{ nextLoc }, goalPos.value(), *this, false, RideId::GetNull());
        if (pathfindDirection == kInvalidDirection)
        {
            /* Heuristic search failed for all directions: reset the goal
             * so the next call retries fresh, as in mechanicDirectionPath. */
            resetPathfindGoal();
            return mechanicDirectionPathRand(pathDirections);
        }

        return pathfindDirection;
    }

    /**
     * fixing SubState (major tier): leave_platform_for_parts - walk to
     * the station exit's inner point (the stock move-to-exit handler),
     * then step onto the path element in the exit tile, the same snap a
     * guest leaving the ride gets (and the worker egress uses). Stock's
     * leave_by_entrance_exit cannot serve here: it ends the fixing
     * state, and the parts run must stay in fixing or the ride-side
     * status update would re-call a second mechanic mid-run.
     */
    bool Staff::updateFixingLeavePlatformForParts(bool firstRun, const Ride& ride)
    {
        if (!updateFixingMoveToStationExit(firstRun, ride))
        {
            return false;
        }

        // At the exit's inner point: step onto the exit tile's path.
        for (auto* pathElement : TileElementsView<PathElement>(CoordsXY{ x, y }))
        {
            int32_t height = MapHeightFromSlope(CoordsXY{ x, y }, pathElement->getSlopeDirection(), pathElement->isSloped());
            height += pathElement->getBaseZ();
            const int32_t zDiff = z - height;
            if (zDiff > 0 || zDiff < -16)
                continue;
            moveTo({ x, y, height });
            resetPathfindGoal();
            return true;
        }

        // The exit tile's path is gone (deleted under the exit): stock
        // recovery - the ride re-calls and a mechanic restarts the job.
        setState(PeepState::falling);
        return false;
    }

    /**
     * fixing SubState (major tier): go_to_parts - walk the path network
     * to the parts source (the nearest park entrance). Movement is
     * performNextAction with doPartsRunPathFinding choosing the steps;
     * arrival is the entrance tile test, re-checked every tick like the
     * stall post arrival. The give-up timer is the answering one: a
     * mechanic who cannot get there falls back and the ride re-calls,
     * restarting the job - never wedged in the substate.
     */
    bool Staff::updateFixingGoToParts(bool firstRun)
    {
        if (!firstRun)
        {
            mechanicTimeSinceCall = 0;
            resetPathfindGoal();
        }

        mechanicTimeSinceCall++;
        if (mechanicTimeSinceCall > 2500)
        {
            setState(PeepState::falling);
            return false;
        }

        const auto entrance = GetNearestParkEntrance(CoordsXY{ x, y });
        if (!entrance.has_value())
        {
            // No park entrance exists (a degenerate park - staff spawn
            // there, so it should not happen): abandon the run.
            setState(PeepState::falling);
            return false;
        }

        if (TileCoordsXY{ CoordsXY{ x, y } } == TileCoordsXY{ entrance->x, entrance->y })
        {
            return true;
        }

        if (!checkForPath())
            return false;

        if (shouldWaitForLevelCrossing())
            return false;

        performNextAction();
        return false;
    }

    /**
     * fixing SubState (major tier): fetch_parts - dwell at the parts
     * source, fetching. Frame-driven like every other fix beat: the
     * dwell lasts one staffFixGround animation.
     */
    bool Staff::updateFixingFetchParts(bool firstRun)
    {
        if (!firstRun)
        {
            orientation = peepDirection << 3;
            action = PeepActionType::staffFixGround;
            animationFrameNum = 0;
            animationImageIdOffset = 0;
            updateCurrentAnimationType();
        }

        if (isActionWalking())
        {
            return true;
        }

        updateAction();
        invalidate();
        return false;
    }

    /**
     * fixing SubState (major tier): return_from_parts - walk the path
     * network back to the ride's station. Arrival is the entrance/exit
     * element interaction, the same test updateAnswering uses, and hands
     * over to the re-enter substate with the platform destination set,
     * exactly as the answering trip into fixing does.
     */
    bool Staff::updateFixingReturnFromParts(bool firstRun, const Ride& ride)
    {
        if (!firstRun)
        {
            mechanicTimeSinceCall = 0;
            resetPathfindGoal();
        }

        mechanicTimeSinceCall++;
        if (mechanicTimeSinceCall > 2500)
        {
            setState(PeepState::falling);
            return false;
        }

        if (!checkForPath())
            return false;

        if (shouldWaitForLevelCrossing())
            return false;

        const auto [pathingResult, rideEntranceExitElement] = performNextAction();
        if (!(pathingResult & PATHING_RIDE_EXIT) && !(pathingResult & PATHING_RIDE_ENTRANCE))
        {
            return false;
        }

        if (currentRide != rideEntranceExitElement->asEntrance()->getRideIndex())
            return false;

        StationIndex exitIndex = rideEntranceExitElement->asEntrance()->getStationIndex();
        if (currentRideStation != exitIndex)
            return false;

        if (pathingResult & PATHING_RIDE_ENTRANCE)
        {
            if (!ride.getStation(exitIndex).exit.isNull())
            {
                return false;
            }
        }

        peepDirection = rideEntranceExitElement->getDirection();

        int32_t destX = nextLoc.x + 16 + DirectionOffsets[peepDirection].x * 53;
        int32_t destY = nextLoc.y + 16 + DirectionOffsets[peepDirection].y * 53;

        setDestination({ destX, destY }, 2);
        orientation = peepDirection << 3;

        z = rideEntranceExitElement->baseHeight * 4;
        subState = PEEP_FIXING_REENTER_STATION;
        return false;
    }

    /**
     * fixing SubState (major tier): reenter_station - walk from the
     * station entrance/exit onto the platform (the answering trip's
     * step-in shape), then hand the sequence back to the stock beats
     * that remain: the parts are marked fetched on the ride, and the
     * substate is set to the breakdown's first fix beat, so the
     * advancement scan in updateFixing lands on the next stock substate
     * after it and the job finishes exactly as a minor one does from
     * there - same remaining beats, same finish, same leave.
     */
    bool Staff::updateFixingReenterStation(bool firstRun, Ride& ride)
    {
        int16_t delta_y = abs(y - getDestination().y);
        if (auto loc = updateAction(); loc.has_value())
        {
            auto newZ = ride.getStation(currentRideStation).getBaseZ();
            if (delta_y < 20)
            {
                newZ += ride.getRideTypeDescriptor().Heights.PlatformHeight;
            }

            moveTo({ loc.value(), newZ });
            return false;
        }

        ride.flags.set(RideFlag::breakdownPartsFetched);

        const uint32_t stockMask = FixingSubstatesForBreakdown[EnumValue(ride.breakdownReasonPending)];
        int32_t firstBeat = PEEP_FIXING_ENTER_STATION;
        for (int32_t candidate = 0; candidate <= PEEP_FIXING_LEAVE_BY_ENTRANCE_EXIT; candidate++)
        {
            if ((stockMask & (1 << candidate)) != 0 && FixingSubstateIsFixBeat(candidate))
            {
                firstBeat = candidate;
                break;
            }
        }
        subState = firstBeat;
        return true;
    }

    /**
     * rct2: 0x6B7588
     */
    void Staff::updateRideInspected(RideId rideIndex)
    {
        auto ride = GetRide(rideIndex);
        if (ride != nullptr)
        {
            ride->flags.unset(RideFlag::dueInspection);
            ride->reliability += ((100 - ride->reliabilityPercentage) / 4) * (ScenarioRand() & 0xFF);
            ride->lastInspection = 0;
            ride->windowInvalidateFlags.set(
                RideInvalidateFlag::maintenance, RideInvalidateFlag::main, RideInvalidateFlag::list);
        }
    }

    money64 GetStaffWage(StaffType type)
    {
        switch (type)
        {
            default:
            case StaffType::handyman:
                return 50.00_GBP;
            case StaffType::mechanic:
                return 80.00_GBP;
            case StaffType::security:
                return 60.00_GBP;
            case StaffType::entertainer:
                return 55.00_GBP;
            case StaffType::worker:
                return 50.00_GBP;
        }
    }

    void Staff::serialise(DataSerialiser& stream)
    {
        Peep::serialise(stream);
        stream << assignedStaffType;
        stream << mechanicTimeSinceCall;
        stream << hireDate;
        stream << staffOrders;
        stream << staffMowingTimeout;
        stream << staffLawnsMown;      // union with staffRidesFixed, staffGuestsEntertained
        stream << staffGardensWatered; // union with staffRidesInspected
        stream << staffLitterSwept;    // union with staffVandalsStopped
        stream << staffBinsEmptied;
    }
} // namespace OpenRCT2
