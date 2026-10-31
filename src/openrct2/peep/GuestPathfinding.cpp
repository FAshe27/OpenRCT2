/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "GuestPathfinding.h"

#include "../Diagnostic.h"
#include "../GameState.h"
#include "../config/Config.h"
#include "../core/Guard.hpp"
#include "../entity/Guest.h"
#include "../entity/Staff.h"
#include "../object/PathAdditionEntry.h"
#include "../profiling/Profiling.h"
#include "../ride/RideData.h"
#include "../scenario/Scenario.h"
#include "../world/Footpath.h"
#include "../world/Map.h"
#include "../world/TileElementsView.h"
#include "../world/Wall.h"
#include "../world/tile_element/BannerElement.h"
#include "../world/tile_element/EntranceElement.h"
#include "../world/tile_element/PathElement.h"
#include "../world/tile_element/TileElement.h"
#include "../world/tile_element/TrackElement.h"
#include "GuestFieldRouting.h"

#include <cassert>
#include <cstring>
#include <limits>

namespace OpenRCT2::PathFinding
{
    // The search limits the maximum junctions by certain conditions.
    // Guest-spread change (personal fork): the whole guest ladder is raised
    // by 2 (staff untouched), so ordinary guests get the junction budget
    // stock reserves for map holders / guests leaving the park, and every
    // intra-guest bonus (map +2, leaving +2, leaving-and-lost +3) is kept.
    // Total search work stays bounded by the unchanged tiles-checked cap
    // in ChooseDirection (15,000 for guests).
    static constexpr uint8_t kMaxJunctionsStaff = 8;
    static constexpr uint8_t kMaxJunctionsGuest = 7;
    static constexpr uint8_t kMaxJunctionsGuestWithMap = 9;
    static constexpr uint8_t kMaxJunctionsGuestLeavingPark = 9;
    static constexpr uint8_t kMaxJunctionsGuestLeavingParkLost = 10;

    // Maximum amount of junctions.
    static constexpr uint8_t kMaxJunctions = std::max(
        { kMaxJunctionsStaff, kMaxJunctionsGuest, kMaxJunctionsGuestWithMap, kMaxJunctionsGuestLeavingPark,
          kMaxJunctionsGuestLeavingParkLost });

    struct PathFindingState
    {
        int8_t junctionCount;
        int8_t maxJunctions;
        int32_t countTilesChecked;
        // TODO: Move them, those are query parameters not really state, but for now its easier to pass it down.
        bool ignoreForeignQueues;
        RideId queueRideIndex;
        // A junction history for the peep path finding heuristic search.
        struct
        {
            TileCoordsXYZ location;
            Direction direction;
        } history[kMaxJunctions + 1];
    };

    static int32_t GuestSurfacePathFinding(Peep& peep);

    enum class PathSearchResult
    {
        deadEnd,      // Path is a dead end, i.e. < 2 edges.
        wide,         // Path with wide flag set.
        thin,         // Path is simple.
        junction,     // Path is a junction, i.e. > 2 edges.
        rideQueue,    // Queue path connected to a ride.
        rideEntrance, // Map element is a ride entrance.
        rideExit,     // Map element is a ride exit.
        parkExit,     // Park entrance / exit (map element is a park entrance/exit).
        shopEntrance, // Map element is a shop entrance.
        other,        // Path is other than the above.
        loop,         // Loop detected.
        limitReached, // Search limit reached without reaching path end.
        failed,       // No path element found.
    };

#pragma region Pathfinding Logging
    // In case this is set to true it will enable code paths that log path finding. The peep will additionally
    // require to have PeepFlag::debugPathfinding set in PeepFlags in order to activate logging.
    static constexpr bool kLogPathfinding = false;

    template<typename... TArgs>
    static void LogPathfinding([[maybe_unused]] const Peep* peep, [[maybe_unused]] const char* format, TArgs&&... args)
    {
        if constexpr (kLogPathfinding)
        {
            if (!peep->peepFlags.has(PeepFlag::debugPathfinding))
                return;

            char buffer[256];
            snprintf(buffer, sizeof(buffer), format, std::forward<TArgs>(args)...);

            if (peep != nullptr)
            {
                LOG_INFO("[%05u:%s] %s", peep->id.ToUnderlying(), peep->getName().c_str(), buffer);
            }
            else
            {
                LOG_INFO("%s", buffer);
            }
        }
    }

    static constexpr const char* PathSearchToString(PathSearchResult pathFindSearchResult)
    {
        switch (pathFindSearchResult)
        {
            case PathSearchResult::deadEnd:
                return "DeadEnd";
            case PathSearchResult::wide:
                return "Wide";
            case PathSearchResult::thin:
                return "Thin";
            case PathSearchResult::junction:
                return "Junction";
            case PathSearchResult::rideQueue:
                return "RideQueue";
            case PathSearchResult::rideEntrance:
                return "RideEntrance";
            case PathSearchResult::rideExit:
                return "RideExit";
            case PathSearchResult::parkExit:
                return "ParkEntryExit";
            case PathSearchResult::shopEntrance:
                return "ShopEntrance";
            case PathSearchResult::limitReached:
                return "LimitReached";
            case PathSearchResult::other:
                return "Other";
            case PathSearchResult::loop:
                return "Loop";
            case PathSearchResult::failed:
                return "Failed";
                // The default case is omitted intentionally.
        }

        return "Unknown";
    }
#pragma endregion

    static const TileElement* GetBannerOnPath(const TileElement* pathElement)
    {
        // This is an improved version of original.
        // That only checked for one fence in the way.
        if (pathElement->isLastForTile())
            return nullptr;

        const TileElement* bannerElement = pathElement + 1;
        do
        {
            // Path on top, so no banners
            if (bannerElement->getType() == TileElementType::path)
                return nullptr;
            // Found a banner
            if (bannerElement->getType() == TileElementType::banner)
                return bannerElement;
            // Last element so there can't be any other banners
            if (bannerElement->isLastForTile())
                return nullptr;

        } while (bannerElement++ != nullptr);

        return nullptr;
    }

    static int32_t BannerClearPathEdges(bool ignoreBanners, const PathElement* pathElement, int32_t edges)
    {
        if (ignoreBanners)
            return edges;
        const TileElement* bannerElement = GetBannerOnPath(reinterpret_cast<const TileElement*>(pathElement));
        if (bannerElement != nullptr)
        {
            do
            {
                edges &= bannerElement->asBanner()->getAllowedEdges();
            } while ((bannerElement = GetBannerOnPath(bannerElement)) != nullptr);
        }
        return edges;
    }

    /**
     * Gets the connected edges of a path that are permitted (i.e. no 'no entry' signs)
     */
    static int32_t PathGetPermittedEdges(bool ignoreBanners, const PathElement* pathElement)
    {
        return BannerClearPathEdges(ignoreBanners, pathElement, pathElement->getEdgesAndCorners()) & 0x0F;
    }

    /**
     * Guest-spread change (personal fork): returns true if the path element the peep is
     * about to step onto (at the given destination tile) has a path addition - a bench,
     * lamp, litter bin etc. - that is drawn on the given edge of that tile.
     *
     * A PathElement stores only WHICH addition it carries (hasAddition / getAdditionEntry),
     * not an edge. The edge is derived exactly the way paintPathAddition derives it when
     * drawing: an addition is drawn on every edge of the tile that is NOT connected to a
     * neighbouring path, i.e. (pathElement->getEdges() ^ 0x0F), where the edge bit for a
     * direction is (1 << direction). The one exception is the jumping fountain, which is
     * drawn in the centre of the tile rather than on any edge, so it never counts here.
     * The destination path element is picked with the same Z / slope test the pathfinding
     * search itself uses (FootpathIsZAndDirectionValid against the peep's next tile Z).
     */
    static bool GuestSpreadPathAdditionOnEdge(const CoordsXY& destTile, const Peep& peep, Direction direction, Direction edge)
    {
        TileElement* tileElement = MapGetFirstElementAt(destTile);
        if (tileElement == nullptr)
        {
            return false;
        }

        const int32_t destZ = TileCoordsXYZ(peep.nextLoc).z;
        do
        {
            if (tileElement->isGhost())
                continue;
            if (tileElement->getType() != TileElementType::path)
                continue;

            const auto* pathElement = tileElement->asPath();
            if (!FootpathIsZAndDirectionValid(*pathElement, destZ, direction))
                continue;

            // This is the path element the peep is stepping onto; no other element on
            // this tile can be the destination path.
            if (!pathElement->hasAddition() || pathElement->additionIsGhost())
            {
                return false;
            }
            const auto* additionEntry = pathElement->getAdditionEntry();
            if (additionEntry == nullptr || additionEntry->draw_type == PathAdditionDrawType::jumpingFountain)
            {
                return false;
            }
            return ((pathElement->getEdges() ^ 0x0F) & (1 << edge)) != 0;
        } while (!(tileElement++)->isLastForTile());

        return false;
    }

    /**
     * Crowd pack (personal fork): returns true - and the exit direction in
     * outExitDirection - if the path element the peep is about to step onto (at the
     * given destination tile) FORCES the peep's next step: after removing the reverse
     * edge (back where the peep came from), exactly one permitted edge remains.
     *
     * This is a pure peek, modelled on FootpathElementDestInDir above: it reads the
     * destination tile's path element with the same Z / slope test the search uses
     * (FootpathIsZAndDirectionValid against the peep's next tile Z, exactly as
     * GuestSpreadPathAdditionOnEdge does) and enumerates that element's permitted
     * edges with PathGetPermittedEdges (which already clears 'no entry' banner
     * edges; guests never ignore banners - see FootpathElementDestinationInDirection).
     * It calls no ChooseDirection, runs no search, draws no ScenarioRand and writes
     * nothing, so it cannot disturb pathfinding state or determinism.
     *
     * "Forced" is deliberately refused for tiles where a single permitted exit does
     * not reliably mean a single next step: sloped tiles (the slope itself steers
     * the peep and the Z test above is only exact for the entry direction), queue
     * tiles (queue movement is its own system) and wide tiles (plaza-style tiles
     * are not single-file corridors).
     */
    static bool GuestSpreadForcedExitDirection(
        const CoordsXY& destTile, const Peep& peep, Direction entryDirection, Direction& outExitDirection)
    {
        TileElement* tileElement = MapGetFirstElementAt(destTile);
        if (tileElement == nullptr)
        {
            return false;
        }

        const int32_t destZ = TileCoordsXYZ(peep.nextLoc).z;
        do
        {
            if (tileElement->isGhost())
                continue;
            if (tileElement->getType() != TileElementType::path)
                continue;

            const auto* pathElement = tileElement->asPath();
            if (!FootpathIsZAndDirectionValid(*pathElement, destZ, entryDirection))
                continue;

            // This is the path element the peep is stepping onto.
            if (pathElement->isSloped() || pathElement->isQueue() || pathElement->isWide())
            {
                return false;
            }

            int32_t edges = PathGetPermittedEdges(false, pathElement);
            edges &= ~(1 << DirectionReverse(entryDirection));
            if (edges == 0 || (edges & (edges - 1)) != 0)
            {
                // Dead end (the peep will turn back) or a junction (a real choice).
                return false;
            }

            for (Direction dir : kAllDirections)
            {
                if (edges & (1 << dir))
                {
                    outExitDirection = dir;
                    return true;
                }
            }
            return false;
        } while (!(tileElement++)->isLastForTile());

        return false;
    }

    // The per-guest lane preference behind the lane spread in
    // PeepMoveOneTile, exposed as GuestSpreadSignedLane (see the header).
    // This is the same peepId-hash arithmetic as the lane block in
    // PeepMoveOneTile - same constants, same mixes, same conversion - and
    // the two MUST stay in step: if the lane tuning ever changes, change
    // both. The lane block itself is deliberately untouched; this is a
    // read-only seam for the field routing's corridor scoring, not a
    // refactor of the lane system.
    int32_t GuestSpreadSignedLane(uint32_t peepId, Direction direction)
    {
        constexpr int32_t kGuestSpreadLaneMax = 8;
        constexpr int32_t kGuestSpreadKeepRightPercent = 65;
        const uint32_t laneHash = peepId * 2654435761u;
        const uint32_t laneMix = laneHash ^ (laneHash >> 16);
        const int32_t laneMagnitude = static_cast<int32_t>(laneMix % static_cast<uint32_t>(kGuestSpreadLaneMax + 1));
        const uint32_t sideHashRaw = peepId * 2246822519u;
        const uint32_t sideMix = sideHashRaw ^ (sideHashRaw >> 16);
        const bool laneKeepsRight = static_cast<int32_t>(sideMix % 100) < kGuestSpreadKeepRightPercent;
        const int32_t laneRight = laneKeepsRight ? laneMagnitude : -laneMagnitude;
        return (direction == 2 || direction == 3) ? laneRight : -laneRight;
    }

    // Wide path lanes (personal fork): when set, PeepMoveOneTile only records the direction instead of moving the
    // peep, so the lane code can ask which way the regular pathfinding would go from a tile (see "Wide path lanes").
    static bool _captureMoveDirection = false;
    static Direction _capturedMoveDirection = kInvalidDirection;
    // Set while CalculateNextDestinationDefault runs in capture mode and reaches a dead end; the dead-end checks
    // are left to the lane code, which knows whether the guest really is at a dead end.
    static bool _capturedDeadEnd = false;
    // Set while CalculateNextDestinationDefault runs in capture mode and takes the only edge left, rather than choosing
    // between edges (e.g. after the wide-edge strip); the lane code doesn't treat such a move as a routing choice.
    static bool _capturedForced = false;

    /**
     *
     *  rct2: 0x0069524E
     */
    static int32_t PeepMoveOneTile(Direction direction, Peep& peep)
    {
        assert(DirectionValid(direction));
        if (_captureMoveDirection)
        {
            _capturedMoveDirection = direction;
            return 0;
        }
        auto newTile = CoordsXY{ CoordsXY{ peep.nextLoc } + CoordsDirectionDelta[direction] }.toTileCentre();

        if (newTile.x >= kMaximumMapSizeBig || newTile.y >= kMaximumMapSizeBig)
        {
            // This could loop!
            return GuestSurfacePathFinding(peep);
        }

        peep.peepDirection = direction;
        if (peep.state != PeepState::queuing)
        {
            // When peeps are walking along a path, we would like them to be spread out across the width of the path,
            // instead of all walking along the exact centre line of the path.
            //
            // Setting a random DestinationTolerance does not work very well for this. It means that peeps will make
            // their new pathfinding decision at a random time, and so will distribute a bit when they are turning
            // corners (which is good); but, as they walk along a straight path, they will - eventually - have had a
            // low tolerance value which forced them back to the centre of the path, where they stay until they turn
            // a corner.
            //
            // What we want instead is to apply that randomness in the direction they are walking ONLY, and keep their
            // other coordinate constant.
            //
            // Guest-spread change (personal fork, based on v0.5.6): the stock game clamps the
            // sideways coordinate to the centre line +/- 3, which funnels every peep through a
            // 7-unit band in the middle of a 32-unit tile - the classic conga line. Instead, give
            // each peep their own "lane" across the width of the path: a sideways offset of
            // -8..+8, derived deterministically from their peepId (a Knuth multiplicative
            // hash, so consecutive guests land in unrelated lanes). Because the lane never
            // changes, a peep walks a straight line parallel to the centre line - no weaving -
            // and corners still vary naturally, as the lane axis swaps at a turn. Deriving it
            // from peepId means no save-game field and no extra ScenarioRand() calls, so
            // saved games and multiplayer stay deterministic. Queues are untouched: this whole
            // block is skipped while queuing, so queue lines stay single file as they must.
            // To change the spread, change kGuestSpreadLaneMax (8 = feet land 8..24 of 32).
            //
            // Furniture-aware lanes (v1.1): at the widest lanes a peep's sprite overlaps the
            // benches and lamps that line the edges of a path tile, so they appear to walk
            // straight through them. So when the destination tile carries a path addition on
            // the peep's own side of the path, the lane is pulled in to
            // kGuestSpreadFurnitureLaneMax for that one step; the clear side of the path - and
            // every unfurnished tile - keeps the full spread. If both sides are furnished the
            // peep is pulled in either way, which falls out of the same per-side test.

            const int8_t offset = (ScenarioRand() & 7) - 3;
            constexpr int32_t kGuestSpreadLaneMax = 8;
            constexpr int32_t kGuestSpreadFurnitureLaneMax = 5;
            // Right-hand traffic (crowd pack, personal fork): v1/v1.1 picked the lane in
            // a fixed compass space, so half the park effectively drove on the left.
            // Real crowds mostly keep right, so the lane is now picked in "right-positive"
            // space - positive means towards the peep's OWN right hand, relative to
            // travel - and converted back to the old signed lane at application time.
            // The peep's underlying preference (laneRight) is one stable per-guest value
            // that never flips; only the conversion changes with direction, because a
            // peep's right hand points a different compass way after they turn:
            //   heading -X (0): right is -Y    heading +Y (1): right is -X
            //   heading +X (2): right is +Y    heading -Y (3): right is +X
            // A positive old-style lane leans towards +Y on X-travel and +X on Y-travel
            // (see the furniture comment below), so the conversion is +laneRight for
            // directions 2/3 and -laneRight for directions 0/1. The full +/-8 range
            // stays reachable in both spaces, the furniture clamp below still operates
            // on the final signed lane exactly as in v1.1, and the draw is still a
            // pure peepId hash - no save field, no ScenarioRand.
            // Magnitude and side are drawn from two differently-mixed hashes of the
            // same peepId so they stay independent: magnitude is uniform 0..8, and
            // about 65% of peeps keep right while 35% wander left (the centre lane,
            // magnitude 0, is side-neutral). Tune via kGuestSpreadKeepRightPercent.
            constexpr int32_t kGuestSpreadKeepRightPercent = 65;
            const uint32_t laneHash = peep.peepId * 2654435761u;
            const uint32_t laneMix = laneHash ^ (laneHash >> 16);
            const int32_t laneMagnitude = static_cast<int32_t>(laneMix % static_cast<uint32_t>(kGuestSpreadLaneMax + 1));
            const uint32_t sideHashRaw = peep.peepId * 2246822519u;
            const uint32_t sideMix = sideHashRaw ^ (sideHashRaw >> 16);
            const bool laneKeepsRight = static_cast<int32_t>(sideMix % 100) < kGuestSpreadKeepRightPercent;
            const int32_t laneRight = laneKeepsRight ? laneMagnitude : -laneMagnitude;
            int32_t lane = (direction == 2 || direction == 3) ? laneRight : -laneRight;

            if (lane > kGuestSpreadFurnitureLaneMax || lane < -kGuestSpreadFurnitureLaneMax)
            {
                // The peep's lane sits on one side of the centre line. The sideways axis is Y
                // when moving along X (directions 0/2) and X when moving along Y (1/3), and
                // the lane is added to that axis below, so a positive lane always leans
                // towards direction 1 (+Y) resp. direction 2 (+X), a negative lane towards
                // the reverse direction.
                const Direction plusSide = (direction == 0 || direction == 2) ? 1 : 2;
                const Direction laneSide = (lane > 0) ? plusSide : DirectionReverse(plusSide);
                if (GuestSpreadPathAdditionOnEdge(newTile, peep, direction, laneSide))
                {
                    lane = std::clamp(lane, -kGuestSpreadFurnitureLaneMax, kGuestSpreadFurnitureLaneMax);
                }
            }
            if (direction == 0 || direction == 2)
            {
                // Peep is moving along X, so apply the offset to the X position of the
                // destination and the peep's lane to its Y position (newTile is already
                // the centre of the destination tile).
                newTile.x += offset;
                newTile.y += lane;
            }
            else
            {
                // Peep is moving along Y, so apply the offset to the Y position of the
                // destination and the peep's lane to its X position.
                newTile.x += lane;
                newTile.y += offset;
            }

            // Corner cutting (crowd pack, personal fork): a peep heading round a bend
            // does not really walk out to the centre of the corner tile and turn on
            // the spot - they clip the inside of the bend. When the tile just entered
            // FORCES a single 90-degree exit (see GuestSpreadForcedExitDirection -
            // a pure peek, no search, no choices), shift this step's destination a
            // few units towards the inside corner of that turn: the corner between
            // the entry edge and the exit edge, whose direction from the tile centre
            // is (exit unit vector - entry unit vector). Straight forced exits
            // (corridors) and junctions get no cut, and neither do dead ends.
            // The cut composes with the lane above: with |lane| <= 8, |offset| <= 4
            // and a cut of 4, the destination stays inside the entered tile with at
            // least 4 units of margin on every side (proven exhaustively in the
            // crowd harness). Queues are untouched: like the lane, this sits inside
            // the not-queuing block.
            //
            // Fence-aware cut (personal fork): the cut pulls the peep towards the
            // inside corner of the turn - the tile corner where the entered tile,
            // the tile behind it, the tile ahead and the diagonal tile meet - so
            // the additions that can stand in the cut's way are drawn on the two
            // boundary lines forming that corner, and those live on the NEIGHBOUR
            // tiles. A path addition is only ever drawn on an UNCONNECTED edge of
            // its own tile (see GuestSpreadPathAdditionOnEdge), and the entered
            // tile's two inside edges are its entry and exit edges, which are
            // connected by construction - an addition can never be drawn on them,
            // so testing the entered tile for them can never fire. (The original
            // crowd-pack version of this check did exactly that, and peeps cut
            // straight through fences standing on the inside of a bend.) If any
            // inside-corner edge of the three neighbour tiles carries an addition,
            // the cut is skipped for this step and the peep walks the full corner,
            // stock-style. The test reuses the lane clamp's helper unchanged: same
            // paint-derived edge semantics, same jumping-fountain and ghost
            // exemptions.
            constexpr int32_t kGuestSpreadCornerCut = 4;
            Direction forcedExit = direction;
            if (GuestSpreadForcedExitDirection(newTile, peep, direction, forcedExit) && forcedExit != direction
                && forcedExit != DirectionReverse(direction))
            {
                const auto entryDelta = CoordsDirectionDelta[direction];
                const auto exitDelta = CoordsDirectionDelta[forcedExit];
                const int32_t cutX = ((exitDelta.x - entryDelta.x) / kCoordsXYStep) * kGuestSpreadCornerCut;
                const int32_t cutY = ((exitDelta.y - entryDelta.y) / kCoordsXYStep) * kGuestSpreadCornerCut;
                if (cutX != 0 || cutY != 0)
                {
                    // Centres of the three tiles that share the inside corner
                    // with the entered tile: behind (entry), ahead (exit) and
                    // diagonal. The entered tile's own centre is the same
                    // expression that produced newTile above.
                    const CoordsXY cornerCentre = CoordsXY{ CoordsXY{ peep.nextLoc } + entryDelta }.toTileCentre();
                    const CoordsXY entryCentre = cornerCentre - entryDelta;
                    const CoordsXY exitCentre = cornerCentre + exitDelta;
                    const CoordsXY diagCentre = entryCentre + exitDelta;
                    const bool insideFenced = GuestSpreadPathAdditionOnEdge(entryCentre, peep, direction, forcedExit)
                        || GuestSpreadPathAdditionOnEdge(exitCentre, peep, forcedExit, DirectionReverse(direction))
                        || GuestSpreadPathAdditionOnEdge(diagCentre, peep, forcedExit, direction)
                        || GuestSpreadPathAdditionOnEdge(diagCentre, peep, forcedExit, DirectionReverse(forcedExit));
                    if (!insideFenced)
                    {
                        newTile.x += cutX;
                        newTile.y += cutY;
                    }
                }
            }
        }
        peep.setDestination(newTile, 2);

        return 0;
    }

    /**
     *
     *  rct2: 0x00694C41
     */
    static int32_t GuestSurfacePathFinding(Peep& peep)
    {
        auto pathPos = CoordsXYRangedZ{ peep.nextLoc, peep.nextLoc.z, peep.nextLoc.z + kPathClearance };
        Direction randDirection = ScenarioRand() & 3;

        if (!WallInTheWay(pathPos, randDirection))
        {
            pathPos.x += CoordsDirectionDelta[randDirection].x;
            pathPos.y += CoordsDirectionDelta[randDirection].y;
            Direction backwardsDirection = DirectionReverse(randDirection);

            if (!WallInTheWay(pathPos, backwardsDirection))
            {
                if (!MapSurfaceIsBlocked(pathPos))
                {
                    return PeepMoveOneTile(randDirection, peep);
                }
            }
        }

        randDirection++;
        uint8_t rand_backwards = ScenarioRand() & 1;
        if (rand_backwards)
        {
            randDirection -= 2;
        }
        randDirection &= 3;

        pathPos.x = peep.nextLoc.x;
        pathPos.y = peep.nextLoc.y;
        if (!WallInTheWay(pathPos, randDirection))
        {
            pathPos.x += CoordsDirectionDelta[randDirection].x;
            pathPos.y += CoordsDirectionDelta[randDirection].y;
            Direction backwardsDirection = DirectionReverse(randDirection);

            if (!WallInTheWay(pathPos, backwardsDirection))
            {
                if (!MapSurfaceIsBlocked(pathPos))
                {
                    return PeepMoveOneTile(randDirection, peep);
                }
            }
        }

        randDirection -= 2;
        randDirection &= 3;

        pathPos.x = peep.nextLoc.x;
        pathPos.y = peep.nextLoc.y;
        if (!WallInTheWay(pathPos, randDirection))
        {
            pathPos.x += CoordsDirectionDelta[randDirection].x;
            pathPos.y += CoordsDirectionDelta[randDirection].y;
            Direction backwardsDirection = DirectionReverse(randDirection);

            if (!WallInTheWay(pathPos, backwardsDirection))
            {
                if (!MapSurfaceIsBlocked(pathPos))
                {
                    return PeepMoveOneTile(randDirection, peep);
                }
            }
        }

        randDirection--;
        if (rand_backwards)
        {
            randDirection += 2;
        }
        randDirection &= 3;
        return PeepMoveOneTile(randDirection, peep);
    }

    /**
     *
     * Returns:
     *   PathSearchResult::Wide - path with wide flag set
     *   PathSearchResult::RideQueue - queue path connected to a ride
     *   PathSearchResult::Other - other path than the above
     *   PathSearchResult::Failed - no path element found
     *
     *  rct2: 0x00694BAE
     *
     * Returns the type of the next footpath tile a peep can get to from x,y,z /
     * inputTileElement in the given direction.
     */
    static PathSearchResult FootpathElementNextInDirection(
        TileCoordsXYZ loc, PathElement* pathElement, Direction chosenDirection)
    {
        if (pathElement->isSloped())
        {
            if (pathElement->getSlopeDirection() == chosenDirection)
            {
                loc.z += 2;
            }
        }

        loc += TileDirectionDelta[chosenDirection];
        TileElement* nextTileElement = MapGetFirstElementAt(loc);
        do
        {
            if (nextTileElement == nullptr)
                break;
            if (nextTileElement->isGhost())
                continue;
            if (nextTileElement->getType() != TileElementType::path)
                continue;
            const auto* nextPathElement = nextTileElement->asPath();
            if (!FootpathIsZAndDirectionValid(*nextPathElement, loc.z, chosenDirection))
                continue;
            if (nextPathElement->isWide())
                return PathSearchResult::wide;
            // Only queue tiles that are connected to a ride are returned as ride queues.
            if (nextPathElement->isQueue() && !nextPathElement->getRideIndex().IsNull())
                return PathSearchResult::rideQueue;

            return PathSearchResult::other;
        } while (!(nextTileElement++)->isLastForTile());

        return PathSearchResult::failed;
    }

    /**
     *
     * Returns:
     *   PathSearchResult::DeadEnd - path is a dead end, i.e. < 2 edges
     *   PathSearchResult::Wide - path with wide flag set
     *   PathSearchResult::Junction - path is a junction, i.e. > 2 edges
     *   PathSearchResult::RideEntrance - map element is a ride entrance
     *   PathSearchResult::RideExit - map element is a ride exit
     *   PathSearchResult::ParkExit - park entrance / exit (map element is a park entrance/exit)
     *   PathSearchResult::ShopEntrance - map element is a shop entrance
     *   PathSearchResult::LimitReached - search limit reached without reaching path end
     *   PathSearchResult::Failed - no path element found
     * For return values RideEntrance, RideExit & ShopEntrance the rideIndex is stored in outRideIndex.
     *
     *  rct2: 0x006949B9
     *
     * This is the recursive portion of FootpathElementDestinationInDirection().
     */
    static PathSearchResult FootpathElementDestInDir(
        bool ignoreBanners, TileCoordsXYZ loc, Direction chosenDirection, RideId* outRideIndex, int32_t level)
    {
        Direction direction;

        if (level > 25)
            return PathSearchResult::limitReached;

        loc += TileDirectionDelta[chosenDirection];
        for (auto* tileElement : TileElementsView(loc))
        {
            if (tileElement->isGhost())
                continue;

            switch (tileElement->getType())
            {
                case TileElementType::track:
                {
                    if (loc.z != tileElement->baseHeight)
                        continue;
                    RideId rideIndex = tileElement->asTrack()->getRideIndex();
                    auto ride = GetRide(rideIndex);
                    if (ride != nullptr && ride->getRideTypeDescriptor().flags.has(RtdFlag::isShopOrFacility))
                    {
                        *outRideIndex = rideIndex;
                        return PathSearchResult::shopEntrance;
                    }
                }
                break;
                case TileElementType::entrance:
                    if (loc.z != tileElement->baseHeight)
                        continue;
                    switch (tileElement->asEntrance()->getEntranceType())
                    {
                        case EntranceType::rideEntrance:
                            direction = tileElement->getDirection();
                            if (direction == chosenDirection)
                            {
                                *outRideIndex = tileElement->asEntrance()->getRideIndex();
                                return PathSearchResult::rideEntrance;
                            }
                            break;
                        case EntranceType::rideExit:
                            direction = tileElement->getDirection();
                            if (direction == chosenDirection)
                            {
                                *outRideIndex = tileElement->asEntrance()->getRideIndex();
                                return PathSearchResult::rideExit;
                            }
                            break;
                        case EntranceType::parkEntrance:
                            return PathSearchResult::parkExit;
                    }
                    break;
                case TileElementType::path:
                {
                    const auto* pathElement = tileElement->asPath();
                    if (!FootpathIsZAndDirectionValid(*pathElement, loc.z, chosenDirection))
                        continue;
                    if (tileElement->asPath()->isWide())
                        return PathSearchResult::wide;

                    uint8_t edges = PathGetPermittedEdges(ignoreBanners, pathElement);
                    edges &= ~(1 << DirectionReverse(chosenDirection));
                    loc.z = tileElement->baseHeight;

                    for (Direction dir : kAllDirections)
                    {
                        if (!(edges & (1 << dir)))
                            continue;

                        edges &= ~(1 << dir);
                        if (edges != 0)
                            return PathSearchResult::junction;

                        if (tileElement->asPath()->isSloped())
                        {
                            if (tileElement->asPath()->getSlopeDirection() == dir)
                            {
                                loc.z += 2;
                            }
                        }
                        return FootpathElementDestInDir(ignoreBanners, loc, dir, outRideIndex, level + 1);
                    }
                    return PathSearchResult::deadEnd;
                }
                default:
                    break;
            }
        }

        return PathSearchResult::failed;
    }

    /**
     * Returns:
     *   PathSearchResult::DeadEnd - path is a dead end, i.e. < 2 edges
     *   PathSearchResult::Wide - path with wide flag set
     *   PathSearchResult::Junction - path is a junction, i.e. > 2 edges
     *   PathSearchResult::RideEntrance - map element is a ride entrance
     *   PathSearchResult::RideExit - map element is a ride exit
     *   PathSearchResult::ParkExit - ark entrance / exit (map element is a park entrance/exit
     *   PathSearchResult::ShopEntrance - map element is a shop entrance
     *   PathSearchResult::LimitReached - search limit reached without reaching path end
     *   PathSearchResult::Failed - no path element found
     * For return values RideEntrance, RideExit & ShopEntrance the rideIndex is stored in outRideIndex.
     *
     *  rct2: 0x006949A4
     *
     * Returns the destination tile type a peep can get to from x,y,z /
     * inputTileElement in the given direction following single width paths only
     * and stopping as soon as a path junction is encountered.
     * Note that a junction is a path with > 2 reachable neighbouring path tiles,
     * so wide paths have LOTS of junctions.
     * This is useful for finding out what is at the end of a short single
     * width path, for example that leads from a ride exit back to the main path.
     */
    static PathSearchResult FootpathElementDestinationInDirection(
        TileCoordsXYZ loc, PathElement* pathElement, Direction chosenDirection, RideId* outRideIndex)
    {
        if (pathElement->isSloped())
        {
            if (pathElement->getSlopeDirection() == chosenDirection)
            {
                loc.z += 2;
            }
        }

        // This function is only called for guests, never ignore the banners.
        return FootpathElementDestInDir(false, loc, chosenDirection, outRideIndex, 0);
    }

    /**
     *
     *  rct2: 0x00695225
     */
    static int32_t GuestPathfindAimless(Peep& peep, uint8_t edges)
    {
        if (ScenarioRand() & 1)
        {
            // If possible go straight
            if (edges & (1 << peep.peepDirection))
            {
                return PeepMoveOneTile(peep.peepDirection, peep);
            }
        }

        while (true)
        {
            Direction direction = ScenarioRand() & 3;
            // Otherwise go in a random direction allowed from the tile.
            if (edges & (1 << direction))
            {
                return PeepMoveOneTile(direction, peep);
            }
        }
    }

    /**
     *
     *  rct2: 0x0069A60A
     */
    static uint8_t PeepPathfindGetMaxNumberJunctions(Peep& peep)
    {
        auto* guest = peep.as<Guest>();
        if (guest == nullptr)
        {
            // Peep can be only Staff and Guest, so when not a Guest it means its Staff.
            return kMaxJunctionsStaff;
        }

        bool isLeavingPark = guest->peepFlags.has(PeepFlag::leavingPark);
        if (isLeavingPark && guest->guestIsLostCountdown < 90)
        {
            return kMaxJunctionsGuestLeavingParkLost;
        }

        if (isLeavingPark)
            return kMaxJunctionsGuestLeavingPark;

        if (guest->hasItem(ShopItem::map))
            return kMaxJunctionsGuestWithMap;

        return kMaxJunctionsGuest;
    }

    /**
     * Returns if the path as xzy is a 'thin' junction.
     * A junction is considered 'thin' if it has more than 2 edges
     * leading to/from non-wide path elements; edges leading to/from non-path
     * elements (e.g. ride/shop entrances) or ride queues are not counted,
     * since entrances and ride queues coming off a path should not result in
     * the path being considered a junction.
     */
    static bool PathIsThinJunction(PathElement* path, const TileCoordsXYZ& loc)
    {
        PROFILED_FUNCTION();

        uint32_t edges = path->getEdges();

        int32_t testEdge = Numerics::bitScanForward(edges);
        if (testEdge == -1)
            return false;

        bool isThinJunction = false;
        int32_t thinCount = 0;
        do
        {
            auto nextFootpathResult = FootpathElementNextInDirection(loc, path, testEdge);

            /* Ignore non-paths (e.g. ride entrances, shops), wide paths
             * and ride queues (per ignoreQueues) when counting
             * neighbouring tiles. */
            if (nextFootpathResult != PathSearchResult::failed && nextFootpathResult != PathSearchResult::wide
                && nextFootpathResult != PathSearchResult::rideQueue)
            {
                thinCount++;
            }

            if (thinCount > 2)
            {
                isThinJunction = true;
                break;
            }
            edges &= ~(1 << testEdge);
        } while ((testEdge = Numerics::bitScanForward(edges)) != -1);
        return isThinJunction;
    }

    static int32_t CalculateHeuristicPathingScore(const TileCoordsXYZ& loc1, const TileCoordsXYZ& loc2)
    {
        auto xDelta = abs(loc1.x - loc2.x) * 32;
        auto yDelta = abs(loc1.y - loc2.y) * 32;
        auto zDelta = abs(loc1.z - loc2.z) * 2;

        if (xDelta < yDelta)
            xDelta >>= 4;
        else
            yDelta >>= 4;

        return xDelta + yDelta + zDelta;
    }

    /**
     * Searches for the tile with the best heuristic score within the search limits
     * starting from the given tile x,y,z and going in the given direction test_edge.
     * The best heuristic score is tracked and returned in the call parameters
     * along with the corresponding tile location and search path telemetry
     * (junctions passed through and directions taken).
     *
     * The primary heuristic used is distance from the goal; the secondary
     * heuristic used (when the primary heuristic gives equal scores) is the number
     * of steps. i.e. the search gets as close as possible to the goal in as few
     * steps as possible.
     *
     * Each tile is checked to determine if the goal is reached.
     * When the goal is not reached the search result is only updated at the END
     * of each search path (some map element that is not a path or a path at which
     * a search limit is reached), NOT at each step along the way.
     * This means that the search ignores thin paths that are "no through paths"
     * no matter how close to the goal they get, but will follow possible "through
     * paths".
     *
     * The implementation is a depth first search of the path layout in xyz
     * according to the search limits.
     * Unlike an A* search, which tracks for each tile a heuristic score (a
     * function of the xyz distances to the goal) and cost of reaching that tile
     * (steps to the tile), a single best result "so far" (best heuristic score
     * with least cost) is tracked via the score parameter.
     * With this approach, explicit loop detection is necessary to limit the
     * search space, and each alternate route through the same tile can be
     * returned as the best result, rather than only the shortest route with A*.
     *
     * The parameters that hold the best search result so far are:
     *   - score - the least heuristic distance from the goal
     *   - endSteps - the least number of steps that achieve the score.
     *
     * The following parameters provide telemetry information on best search path so far:
     *   - endXYZ tracks the end location of the search path.
     *   - endSteps tracks the number of steps to the end of the search path.
     *   - endJunctions tracks the number of junctions passed through in the
     *     search path.
     *   - junctionList[] and directionList[] track the junctions and
     *     corresponding directions of the search path.
     * Other than debugging purposes, these could potentially be used to visualise
     * the pathfinding on the map.
     *
     * The parameters/variables that limit the search space are:
     *   - counter (param) - number of steps walked in the current search path;
     *   - _peepPathFindTilesChecked (variable) - cumulative number of tiles that can be
     *     checked in the entire search;
     *   - _peepPathFindNumJunctions (variable) - number of thin junctions that can be
     *     checked in a single search path;
     *
     * Other global variables/state that affect the search space are:
     *   - Wide paths - to handle broad paths (> 1 tile wide), the search navigates
     *     along non-wide (or 'thin' paths) and stops as soon as it encounters a
     *     wide path. This means peeps heading for a destination will only leave
     *     thin paths if walking 1 tile onto a wide path is closer than following
     *     non-wide paths;
     *   - gPeepPathFindIgnoreForeignQueues
     *   - gPeepPathFindQueueRideIndex - the ride the peep is heading for
     *   - _peepPathFindHistory - the search path telemetry consisting of the
     *     starting point and all thin junctions with directions navigated
     *     in the current search path - also used to detect path loops.
     *
     * The score is only updated when:
     *   - the goal is reached;
     *   - a wide tile is encountered with a better search result - the goal may
     *     still be reachable from here (only if the current tile is also wide);
     *   - a junction is encountered with a better search result and
     *     maxNumJunctions is exceeded - the goal may still be reachable from here;
     *   - returning from a recursive call if a search limit (i.e. either
     *     maxNumStep or maxTilesChecked) was reached and the current tile has a
     *     better search result and the goal may still be reachable from here
     *     (i.e. not a dead end path tile).
     *
     *  rct2: 0x0069A997
     */
    static void PeepPathfindHeuristicSearch(
        PathFindingState& state, TileCoordsXYZ loc, const TileCoordsXYZ& goal, const Peep& peep,
        TileElement* currentTileElement, const bool inPatrolArea, uint8_t numSteps, uint16_t* endScore, Direction testEdge,
        uint8_t* endJunctions, TileCoordsXYZ junctionList[16], uint8_t directionList[16], TileCoordsXYZ* endXYZ,
        uint8_t* endSteps)
    {
        PathSearchResult searchResult = PathSearchResult::failed;

        bool currentElementIsWide = currentTileElement->asPath()->isWide();
        if (currentElementIsWide)
        {
            const Staff* staff = peep.as<Staff>();
            if (staff != nullptr && staff->canIgnoreWideFlag(loc.toCoordsXYZ(), currentTileElement))
                currentElementIsWide = false;
        }

        loc += TileDirectionDelta[testEdge];

        ++numSteps;
        state.countTilesChecked--;

        /* If this is where the search started this is a search loop and the
         * current search path ends here.
         * Return without updating the parameters (best result so far). */
        if (state.history[0].location == loc)
        {
            LogPathfinding(&peep, "Return from %d,%d,%d; Steps: %u; At start", loc.x >> 5, loc.y >> 5, loc.z, numSteps);
            return;
        }

        bool nextInPatrolArea = inPatrolArea;
        auto* staff = peep.as<Staff>();
        if (staff != nullptr && staff->isMechanic())
        {
            nextInPatrolArea = staff->isLocationInPatrol(loc.toCoordsXY());
            if (inPatrolArea && !nextInPatrolArea)
            {
                /* The mechanic will leave his patrol area by taking
                 * the test_edge so the current search path ends here.
                 * Return without updating the parameters (best result so far). */
                LogPathfinding(
                    &peep, "Return from %d,%d,%d; Steps: %u; Left patrol area", loc.x >> 5, loc.y >> 5, loc.z, numSteps);
                return;
            }
        }

        /* Get the next map element of interest in the direction of testEdge. */
        bool found = false;
        TileElement* tileElement = MapGetFirstElementAt(loc);
        if (tileElement == nullptr)
        {
            return;
        }
        do
        {
            /* Look for all map elements that the peep could walk onto while
             * navigating to the goal, including the goal tile. */

            if (tileElement->isGhost())
                continue;

            RideId rideIndex = RideId::GetNull();
            switch (tileElement->getType())
            {
                case TileElementType::track:
                {
                    if (loc.z != tileElement->baseHeight)
                        continue;
                    /* For peeps heading for a shop, the goal is the shop
                     * tile. */
                    rideIndex = tileElement->asTrack()->getRideIndex();
                    auto ride = GetRide(rideIndex);
                    if (ride == nullptr || !ride->getRideTypeDescriptor().flags.has(RtdFlag::isShopOrFacility))
                        continue;

                    found = true;
                    searchResult = PathSearchResult::shopEntrance;
                    break;
                }
                case TileElementType::entrance:
                    if (loc.z != tileElement->baseHeight)
                        continue;
                    Direction direction;
                    searchResult = PathSearchResult::other;
                    switch (tileElement->asEntrance()->getEntranceType())
                    {
                        case EntranceType::rideEntrance:
                            /* For peeps heading for a ride without a queue, the
                             * goal is the ride entrance tile.
                             * For mechanics heading for the ride entrance
                             * (in the case when the station has no exit),
                             * the goal is the ride entrance tile. */
                            direction = tileElement->getDirection();
                            if (direction == testEdge)
                            {
                                /* The rideIndex will be useful for
                                 * adding transport rides later. */
                                rideIndex = tileElement->asEntrance()->getRideIndex();
                                searchResult = PathSearchResult::rideEntrance;
                                found = true;
                                break;
                            }
                            continue; // Ride entrance is not facing the right direction.
                        case EntranceType::parkEntrance:
                            /* For peeps leaving the park, the goal is the park
                             * entrance/exit tile. */
                            searchResult = PathSearchResult::parkExit;
                            found = true;
                            break;
                        case EntranceType::rideExit:
                            /* For mechanics heading for the ride exit, the
                             * goal is the ride exit tile. */
                            direction = tileElement->getDirection();
                            if (direction == testEdge)
                            {
                                searchResult = PathSearchResult::rideExit;
                                found = true;
                                break;
                            }
                            continue; // Ride exit is not facing the right direction.
                        default:
                            continue;
                    }
                    break;
                case TileElementType::path:
                {
                    const auto* pathElement = tileElement->asPath();
                    /* For peeps heading for a ride with a queue, the goal is the last
                     * queue path.
                     * Otherwise, peeps walk on path tiles to get to the goal. */
                    if (!FootpathIsZAndDirectionValid(*pathElement, loc.z, testEdge))
                        continue;

                    // Path may be sloped, so set z to path base height.
                    loc.z = tileElement->baseHeight;

                    if (pathElement->isWide())
                    {
                        /* Check if staff can ignore this wide flag. */
                        if (staff == nullptr || !staff->canIgnoreWideFlag(loc.toCoordsXYZ(), tileElement))
                        {
                            searchResult = PathSearchResult::wide;
                            found = true;
                            break;
                        }
                    }

                    searchResult = PathSearchResult::thin;

                    uint8_t numEdges = std::popcount(pathElement->getEdges());

                    if (numEdges < 2)
                    {
                        searchResult = PathSearchResult::deadEnd;
                    }
                    else if (numEdges > 2)
                    {
                        searchResult = PathSearchResult::junction;
                    }
                    else
                    { // numEdges == 2
                        if (pathElement->isQueue() && pathElement->getRideIndex() != state.queueRideIndex)
                        {
                            if (state.ignoreForeignQueues && !pathElement->getRideIndex().IsNull())
                            {
                                // Path is a queue we aren't interested in
                                /* The rideIndex will be useful for
                                 * adding transport rides later. */
                                rideIndex = pathElement->getRideIndex();
                                searchResult = PathSearchResult::rideQueue;
                            }
                        }
                    }
                    found = true;
                }
                break;
                default:
                    continue;
            }

            LogPathfinding(
                &peep, "Checking map element at %d,%d,%d; Type: %s; Steps: %u", loc.x >> 5, loc.y >> 5, loc.z,
                PathSearchToString(searchResult), numSteps);

            /* At this point tileElement is of interest to the pathfinding. */

            /* Should we check that this tileElement is connected in the
             * reverse direction? For some tileElement types this was
             * already done above (e.g. ride entrances), but for others not.
             * Ignore for now. */

            // Calculate the heuristic score of this map element.
            uint16_t newScore = CalculateHeuristicPathingScore(loc, goal);

            /* If this map element is the search goal the current search path ends here. */
            if (newScore == 0)
            {
                /* If the search result is better than the best so far (in the parameters),
                 * then update the parameters with this search before continuing to the next map element. */
                if (newScore < *endScore || (newScore == *endScore && numSteps < *endSteps))
                {
                    // Update the search results
                    *endScore = newScore;
                    *endSteps = numSteps;
                    // Update the end x,y,z
                    *endXYZ = loc;
                    // Update the telemetry
                    *endJunctions = state.maxJunctions - state.junctionCount;
                    for (uint8_t junctInd = 0; junctInd < *endJunctions; junctInd++)
                    {
                        uint8_t histIdx = state.maxJunctions - junctInd;
                        junctionList[junctInd].x = state.history[histIdx].location.x;
                        junctionList[junctInd].y = state.history[histIdx].location.y;
                        junctionList[junctInd].z = state.history[histIdx].location.z;
                        directionList[junctInd] = state.history[histIdx].direction;
                    }
                }
                LogPathfinding(
                    &peep, "Search path ends at %d,%d,%d; Steps: %u; At goal; Score: %d", loc.x >> 5, loc.y >> 5, loc.z,
                    numSteps, newScore);
                continue;
            }

            /* At this point the map element tile is not the goal. */

            /* If this map element is not a path, the search cannot be continued.
             * Continue to the next map element without updating the parameters (best result so far). */
            if (searchResult != PathSearchResult::deadEnd && searchResult != PathSearchResult::thin
                && searchResult != PathSearchResult::junction && searchResult != PathSearchResult::wide)
            {
                LogPathfinding(
                    &peep, "Search path ends at %d,%d,%d; Steps: %u; Not a path", loc.x >> 5, loc.y >> 5, loc.z, numSteps);
                continue;
            }

            /* At this point the map element is a path. */

            /* If this is a wide path the search ends here. */
            if (searchResult == PathSearchResult::wide)
            {
                /* Ignore Wide paths as continuing paths UNLESS
                 * the current path is also Wide (and, for staff, not ignored).
                 * This permits a peep currently on a wide path to
                 * cross other wide paths to reach a thin path.
                 *
                 * So, if the current path is also wide the goal could
                 * still be reachable from here.
                 * If the search result is better than the best so far
                 * (in the parameters), then update the parameters with
                 * this search before continuing to the next map element. */
                if (currentElementIsWide && (newScore < *endScore || (newScore == *endScore && numSteps < *endSteps)))
                {
                    // Update the search results
                    *endScore = newScore;
                    *endSteps = numSteps;
                    // Update the end x,y,z
                    *endXYZ = loc;
                    // Update the telemetry
                    *endJunctions = state.maxJunctions - state.junctionCount;
                    for (uint8_t junctInd = 0; junctInd < *endJunctions; junctInd++)
                    {
                        uint8_t histIdx = state.maxJunctions - junctInd;
                        junctionList[junctInd].x = state.history[histIdx].location.x;
                        junctionList[junctInd].y = state.history[histIdx].location.y;
                        junctionList[junctInd].z = state.history[histIdx].location.z;
                        directionList[junctInd] = state.history[histIdx].direction;
                    }
                }
                LogPathfinding(
                    &peep, "Search path ends at %d,%d,%d; Steps: %u; Wide path; Score: %d", loc.x >> 5, loc.y >> 5, loc.z,
                    numSteps, newScore);
                continue;
            }

            /* At this point the map element is a non-wide path.*/

            /* Get all the permitted_edges of the map element. */
            Guard::Assert(tileElement->asPath() != nullptr);
            uint32_t edges = PathGetPermittedEdges(staff != nullptr, tileElement->asPath());

            LogPathfinding(
                &peep, "Path element at %d,%d,%d; Steps: %u; Edges (0123):%d%d%d%d; Reverse: %d", loc.x >> 5, loc.y >> 5, loc.z,
                numSteps, edges & 1, (edges & 2) >> 1, (edges & 4) >> 2, (edges & 8) >> 3, testEdge ^ 2);

            /* Remove the reverse edge (i.e. the edge back to the previous map element.) */
            edges &= ~(1 << DirectionReverse(testEdge));

            int32_t nextTestEdge = Numerics::bitScanForward(edges);

            /* If there are no other edges the current search ends here.
             * Continue to the next map element without updating the parameters (best result so far). */
            if (nextTestEdge == -1)
            {
                LogPathfinding(
                    &peep, "Search path ends at %d,%d,%d; Steps: %u; No more edges/dead end", loc.x >> 5, loc.y >> 5, loc.z,
                    numSteps);
                continue;
            }

            /* Check if either of the search limits has been reached:
             * - max number of steps or max tiles checked. */
            if (numSteps >= 200 || state.countTilesChecked <= 0)
            {
                /* The current search ends here.
                 * The path continues, so the goal could still be reachable from here.
                 * If the search result is better than the best so far (in the parameters),
                 * then update the parameters with this search before continuing to the next map element. */
                if (newScore < *endScore || (newScore == *endScore && numSteps < *endSteps))
                {
                    // Update the search results
                    *endScore = newScore;
                    *endSteps = numSteps;
                    // Update the end x,y,z
                    *endXYZ = loc;
                    // Update the telemetry
                    *endJunctions = state.maxJunctions - state.junctionCount;
                    for (uint8_t junctInd = 0; junctInd < *endJunctions; junctInd++)
                    {
                        uint8_t histIdx = state.maxJunctions - junctInd;
                        junctionList[junctInd].x = state.history[histIdx].location.x;
                        junctionList[junctInd].y = state.history[histIdx].location.y;
                        junctionList[junctInd].z = state.history[histIdx].location.z;
                        directionList[junctInd] = state.history[histIdx].direction;
                    }
                }
                LogPathfinding(
                    &peep, "Search path ends at %d,%d,%d; Steps: %u; Search limit reached; Score: %d", loc.x >> 5, loc.y >> 5,
                    loc.z, numSteps, newScore);
                continue;
            }

            bool isThinJunction = false;
            if (searchResult == PathSearchResult::junction)
            {
                /* Check if this is a thin junction. And perform additional
                 * necessary checks. */
                isThinJunction = PathIsThinJunction(tileElement->asPath(), loc);

                if (isThinJunction)
                {
                    /* The current search path is passing through a thin
                     * junction on this map element. Only 'thin' junctions
                     * are counted towards the junction search limit. */

                    /* First check if going through the junction would be
                     * a loop.  If so, the current search path ends here.
                     * Path finding loop detection can take advantage of both the
                     * peep.PathfindHistory - loops through remembered junctions
                     *     the peep has already passed through getting to its
                     *     current position while on the way to its current goal;
                     * _peepPathFindHistory - loops in the current search path. */
                    bool pathLoop = false;
                    /* Check the peep.PathfindHistory to see if this junction has
                     * already been visited by the peep while heading for this goal. */
                    for (auto& pathfindHistory : peep.pathfindHistory)
                    {
                        if (pathfindHistory == loc)
                        {
                            if (pathfindHistory.direction == 0)
                            {
                                /* If all directions have already been tried while
                                 * heading to this goal, this is a loop. */
                                pathLoop = true;
                            }
                            else
                            {
                                /* The peep remembers walking through this junction
                                 * before, but has not yet tried all directions.
                                 * Limit the edges to search to those not yet tried. */
                                edges &= pathfindHistory.direction;
                            }
                            break;
                        }
                    }

                    if (!pathLoop)
                    {
                        /* Check the _peepPathFindHistory to see if this junction has been
                         * previously passed through in the current search path.
                         * i.e. this is a loop in the current search path. */
                        for (int32_t junctionNum = state.junctionCount + 1; junctionNum <= state.maxJunctions; junctionNum++)
                        {
                            if (state.history[junctionNum].location == loc)
                            {
                                pathLoop = true;
                                break;
                            }
                        }
                    }
                    if (pathLoop)
                    {
                        /* Loop detected.  The current search path ends here.
                         * Continue to the next map element without updating the parameters (best result so far). */
                        LogPathfinding(
                            &peep, "Search path ends at %d,%d,%d; Steps: %u; Loop", loc.x >> 5, loc.y >> 5, loc.z, numSteps);
                        continue;
                    }

                    /* If the junction search limit is reached, the
                     * current search path ends here. The goal may still
                     * be reachable from here.
                     * If the search result is better than the best so far (in the parameters),
                     * then update the parameters with this search before continuing to the next map element. */
                    if (state.junctionCount <= 0)
                    {
                        if (newScore < *endScore || (newScore == *endScore && numSteps < *endSteps))
                        {
                            // Update the search results
                            *endScore = newScore;
                            *endSteps = numSteps;
                            // Update the end x,y,z
                            *endXYZ = loc;
                            // Update the telemetry
                            *endJunctions = state.maxJunctions; // - _peepPathFindNumJunctions;
                            for (uint8_t junctInd = 0; junctInd < *endJunctions; junctInd++)
                            {
                                uint8_t histIdx = state.maxJunctions - junctInd;
                                junctionList[junctInd] = state.history[histIdx].location;
                                directionList[junctInd] = state.history[histIdx].direction;
                            }
                        }
                        LogPathfinding(
                            &peep, "Search path ends at %d,%d,%d; Steps: %u; NumJunctions < 0; Score: %d", loc.x >> 5,
                            loc.y >> 5, loc.z, numSteps, newScore);
                        continue;
                    }

                    /* This junction was NOT previously visited in the current
                     * search path, so add the junction to the history. */
                    state.history[state.junctionCount].location = loc;
                    // .direction take is added below.

                    state.junctionCount--;
                }
            }

            /* Continue searching down each remaining edge of the path
             * (recursive call). */
            do
            {
                edges &= ~(1 << nextTestEdge);
                uint8_t savedNumJunctions = state.junctionCount;

                uint8_t height = loc.z;
                if (tileElement->asPath()->isSloped() && tileElement->asPath()->getSlopeDirection() == nextTestEdge)
                {
                    height += 2;
                }

                if constexpr (kLogPathfinding)
                {
                    if (searchResult == PathSearchResult::junction)
                    {
                        if (isThinJunction)
                            LogPathfinding(
                                &peep, "Recurse from %d,%d,%d; Steps: %u; edge: %d; Thin-Junction", loc.x >> 5, loc.y >> 5,
                                loc.z, numSteps, nextTestEdge);
                        else
                            LogPathfinding(
                                &peep, "Recurse from %d,%d,%d; Steps: %u; edge: %d; Wide-Junction", loc.x >> 5, loc.y >> 5,
                                loc.z, numSteps, nextTestEdge);
                    }
                    else
                    {
                        LogPathfinding(
                            &peep, "Recurse from %d,%d,%d; Steps: %u; edge: %d; Segment", loc.x >> 5, loc.y >> 5, loc.z,
                            numSteps, nextTestEdge);
                    }
                }

                if (isThinJunction)
                {
                    /* Add the current test_edge to the history. */
                    state.history[state.junctionCount + 1].direction = nextTestEdge;
                }

                PeepPathfindHeuristicSearch(
                    state, { loc.x, loc.y, height }, goal, peep, tileElement, nextInPatrolArea, numSteps, endScore,
                    nextTestEdge, endJunctions, junctionList, directionList, endXYZ, endSteps);
                state.junctionCount = savedNumJunctions;

                LogPathfinding(
                    &peep, "Returned to %d,%d,%d; Steps: %u; edge: %d; Score: %d", loc.x >> 5, loc.y >> 5, loc.z, numSteps,
                    nextTestEdge, *endScore);
            } while ((nextTestEdge = Numerics::bitScanForward(edges)) != -1);

        } while (!(tileElement++)->isLastForTile());

        if (!found)
        {
            /* No map element could be found.
             * Return without updating the parameters (best result so far). */
            LogPathfinding(
                &peep, "Returning from %d,%d,%d; Steps: %u; No relevant map element found", loc.x >> 5, loc.y >> 5, loc.z,
                numSteps);
        }
        else
        {
            LogPathfinding(
                &peep, "Returning from %d,%d,%d; Steps: %u; All map elements checked", loc.x >> 5, loc.y >> 5, loc.z, numSteps);
        }
    }

    /**
     * Returns:
     *   -1   - no direction chosen
     *   0..3 - chosen direction
     *
     *  rct2: 0x0069A5F0
     */
    Direction ChooseDirection(
        const TileCoordsXYZ& loc, const TileCoordsXYZ& goal, Peep& peep, bool ignoreForeignQueues, RideId queueRideIndex)
    {
        PROFILED_FUNCTION();

        PathFindingState state{};

        state.ignoreForeignQueues = ignoreForeignQueues;
        state.queueRideIndex = queueRideIndex;

        // The max number of thin junctions searched - a per-search-path limit.
        state.maxJunctions = PeepPathfindGetMaxNumberJunctions(peep);

        /* The max number of tiles to check - a whole-search limit.
         * Mainly to limit the performance impact of the path finding. */
        int32_t maxTilesChecked = (peep.is<Staff>()) ? 50000 : 15000;

        LogPathfinding(&peep, "Choose direction for goal %d,%d,%d from %d,%d,%d", goal.x, goal.y, goal.z, loc.x, loc.y, loc.z);

        // Get the path element at this location
        TileElement* destTileElement = MapGetFirstElementAt(loc);
        /* Where there are multiple matching map elements placed with zero
         * clearance, save the first one for later use to determine the path
         * slope - this maintains the original behaviour (which only processes
         * the first matching map element found) and is consistent with peep
         * placement (i.e. height) on such paths with differing slopes.
         *
         * I cannot see a legitimate reason for building overlaid paths with
         * differing slopes and do not recall ever seeing this in practise.
         * Normal cases I have seen in practise are overlaid paths with the
         * same slope (flat) in order to place scenery (e.g. benches) in the
         * middle of a wide path that can still be walked through.
         * Anyone attempting to overlay paths with different slopes should
         * EXPECT to experience path finding irregularities due to those paths!
         * In particular common edges at different heights will not work
         * in a useful way. Simply do not do it! :-) */
        TileElement* firstTileElement = nullptr;

        bool found = false;
        uint8_t permittedEdges = 0;
        bool isThin = false;
        do
        {
            if (destTileElement == nullptr)
                break;
            if (destTileElement->baseHeight != loc.z)
                continue;
            if (destTileElement->getType() != TileElementType::path)
                continue;
            found = true;
            if (firstTileElement == nullptr)
            {
                firstTileElement = destTileElement;
            }

            /* Check if this path element is a thin junction.
             * Only 'thin' junctions are remembered in peep.PathfindHistory.
             * NO attempt is made to merge the overlaid path elements and
             * check if the combination is 'thin'!
             * The junction is considered 'thin' simply if any of the
             * overlaid path elements there is a 'thin junction'. */
            isThin = isThin || PathIsThinJunction(destTileElement->asPath(), loc);

            // Collect the permitted edges of ALL matching path elements at this location.
            permittedEdges |= PathGetPermittedEdges(peep.is<Staff>(), destTileElement->asPath());
        } while (!(destTileElement++)->isLastForTile());
        // Peep is not on a path.
        if (!found)
            return kInvalidDirection;

        permittedEdges &= 0xF;
        uint32_t edges = permittedEdges;
        if (isThin && peep.pathfindGoal == goal)
        {
            /* Use of peep.PathfindHistory[]:
             * When walking to a goal, the peep PathfindHistory stores
             * the last 4 thin junctions that the peep walked through.
             * For each of these 4 thin junctions the peep remembers
             * those edges it has not yet taken.
             * If a peep returns to one of the 4 thin junctions that it
             * remembers, it will only choose from the directions that it
             * did not try yet.
             * This forces to the peep pathfinding to try the "next best"
             * direction after trying the "best" direction(s) and finding
             * that the goal could not be reached. */

            /* If the peep remembers walking through this junction
             * previously while heading for its goal, retrieve the
             * directions it has not yet tried. */
            for (auto& pathfindHistory : peep.pathfindHistory)
            {
                if (pathfindHistory == loc)
                {
                    /* Fix broken PathfindHistory[i].direction
                     * which have untried directions that are not
                     * currently possible - could be due to pathing
                     * changes or in earlier code .directions was
                     * initialised to 0xF rather than the permitted
                     * edges. */
                    pathfindHistory.direction &= permittedEdges;

                    edges = pathfindHistory.direction;

                    LogPathfinding(
                        &peep, "Getting untried edges from pf_history for %d,%d,%d:  %s,%s,%s,%s", loc.x, loc.y, loc.z,
                        (edges & 1) ? "0" : "-", (edges & 2) ? "1" : "-", (edges & 4) ? "2" : "-", (edges & 8) ? "3" : "-");

                    if (edges == 0)
                    {
                        /* If peep has tried all edges, reset to
                         * all edges are untried.
                         * This permits the pathfinding to try
                         * again, which is good for getting
                         * unstuck when the player has edited
                         * the paths or the pathfinding itself
                         * has changed (been fixed) since
                         * the game was saved. */
                        pathfindHistory.direction = permittedEdges;
                        edges = pathfindHistory.direction;

                        LogPathfinding(&peep, "All edges tried for %d,%d,%d - resetting to all untried", loc.x, loc.y, loc.z);
                    }
                    break;
                }
            }
        }

        /* If this is a new goal for the peep. Store it and reset the peep's
         * PathfindHistory. */
        if (!DirectionValid(peep.pathfindGoal.direction) || peep.pathfindGoal != goal)
        {
            peep.pathfindGoal = { goal, 0 };

            // Clear pathfinding history
            TileCoordsXYZD nullPos;
            nullPos.setNull();

            std::fill(std::begin(peep.pathfindHistory), std::end(peep.pathfindHistory), nullPos);

            LogPathfinding(&peep, "New goal; clearing pf_history.");
        }

        /* Guest-spread change (personal fork): distance-field routing. For
         * goal-directed guests, a shared per-destination distance field
         * (GuestFieldRouting) can answer this junction outright: a permitted
         * step to a strictly-closer node is guaranteed progress along a
         * shortest route, so the heuristic search below - with its junction
         * budget and its wide-tile path ends - does not need to run. When
         * no field answer applies (the field is not built yet, the guest
         * stands off the routed graph, or they are already on the
         * destination's doorstep), execution falls through to the stock
         * search unchanged. The goal bookkeeping above has already run, so
         * peep.pathfindGoal stays correct for the callers that read it;
         * the field attempt itself writes no peep state, consumes no
         * pathfind history and draws no ScenarioRand. */
        {
            Direction fieldDirection = kInvalidDirection;
            if (GuestFieldTryStep(peep, loc, goal, permittedEdges, fieldDirection))
            {
                return fieldDirection;
            }
        }

        // Peep has tried all edges.
        if (edges == 0)
            return kInvalidDirection;

        int32_t chosenEdge = Numerics::bitScanForward(edges);

        // Peep has multiple edges still to try.
        if (edges & ~(1 << chosenEdge))
        {
            uint8_t bestJunctions = 0;
            TileCoordsXYZ bestJunctionList[16];
            uint8_t bestDirectionList[16];
            TileCoordsXYZ bestXYZ;

            uint16_t bestScore = 0xFFFF;
            uint8_t bestSub = 0xFF;

            LogPathfinding(
                &peep, "Pathfind start for goal %d,%d,%d from %d,%d,%d", goal.x, goal.y, goal.z, loc.x, loc.y, loc.z);

            /* Call the search heuristic on each edge, keeping track of the
             * edge that gives the best (i.e. smallest) value (best_score)
             * or for different edges with equal value, the edge with the
             * least steps (best_sub). */
            int32_t numEdges = std::popcount(edges);
            for (int32_t testEdge = chosenEdge; testEdge != -1; testEdge = Numerics::bitScanForward(edges))
            {
                edges &= ~(1 << testEdge);
                uint8_t height = loc.z;

                if (firstTileElement->asPath()->isSloped() && firstTileElement->asPath()->getSlopeDirection() == testEdge)
                {
                    height += 0x2;
                }

                /* Divide the maxTilesChecked global search limit
                 * between the remaining edges to ensure the search
                 * covers all of the remaining edges. */
                state.countTilesChecked = maxTilesChecked / numEdges;
                state.junctionCount = state.maxJunctions;

                // Initialise _peepPathFindHistory.

                for (auto& entry : state.history)
                {
                    entry.location.setNull();
                    entry.direction = kInvalidDirection;
                }

                /* The pathfinding will only use elements
                 * 1.._peepPathFindMaxJunctions, so the starting point
                 * is placed in element 0 */
                state.history[0].location = loc;
                state.history[0].direction = 0xF;

                uint16_t score = 0xFFFF;
                /* Variable endXYZ contains the end location of the
                 * search path. */
                TileCoordsXYZ endXYZ;
                endXYZ.x = 0;
                endXYZ.y = 0;
                endXYZ.z = 0;

                uint8_t endSteps = 255;

                /* Variable endJunctions is the number of junctions
                 * passed through in the search path.
                 * Variables endJunctionList and endDirectionList
                 * contain the junctions and corresponding directions
                 * of the search path.
                 * In the future these could be used to visualise the
                 * pathfinding on the map. */
                uint8_t endJunctions = 0;
                TileCoordsXYZ endJunctionList[16];
                uint8_t endDirectionList[16] = { 0 };

                bool inPatrolArea = false;
                auto* staff = peep.as<Staff>();
                if (staff != nullptr && staff->isMechanic())
                {
                    /* Mechanics are the only staff type that
                     * pathfind to a destination. Determine if the
                     * mechanic is in their patrol area. */
                    inPatrolArea = staff->isLocationInPatrol(peep.nextLoc);
                }

                LogPathfinding(
                    &peep, "Pathfind searching in direction: %d from %d,%d,%d", testEdge, loc.x >> 5, loc.y >> 5, loc.z);

                PeepPathfindHeuristicSearch(
                    state, { loc.x, loc.y, height }, goal, peep, firstTileElement, inPatrolArea, 0, &score, testEdge,
                    &endJunctions, endJunctionList, endDirectionList, &endXYZ, &endSteps);

                if constexpr (kLogPathfinding)
                {
                    LogPathfinding(
                        &peep, "Pathfind test edge: %d score: %d steps: %d end: %d,%d,%d junctions: %d", testEdge, score,
                        endSteps, endXYZ.x, endXYZ.y, endXYZ.z, endJunctions);
                    for (uint8_t listIdx = 0; listIdx < endJunctions; listIdx++)
                    {
                        LogPathfinding(
                            &peep, "Junction#%d %d,%d,%d Direction %d", listIdx + 1, endJunctionList[listIdx].x,
                            endJunctionList[listIdx].y, endJunctionList[listIdx].z, endDirectionList[listIdx]);
                    }
                }

                if (score < bestScore || (score == bestScore && endSteps < bestSub))
                {
                    chosenEdge = testEdge;
                    bestScore = score;
                    bestSub = endSteps;

                    if constexpr (kLogPathfinding)
                    {
                        bestJunctions = endJunctions;
                        for (uint8_t index = 0; index < endJunctions; index++)
                        {
                            bestJunctionList[index].x = endJunctionList[index].x;
                            bestJunctionList[index].y = endJunctionList[index].y;
                            bestJunctionList[index].z = endJunctionList[index].z;
                            bestDirectionList[index] = endDirectionList[index];
                        }
                        bestXYZ.x = endXYZ.x;
                        bestXYZ.y = endXYZ.y;
                        bestXYZ.z = endXYZ.z;
                    }
                }
            }

            /* Check if the heuristic search failed. e.g. all connected
             * paths are within the search limits and none reaches the
             * goal. */
            if (bestScore == 0xFFFF)
            {
                LogPathfinding(&peep, "Pathfind heuristic search failed.");
                return kInvalidDirection;
            }

            if constexpr (kLogPathfinding)
            {
                LogPathfinding(&peep, "Pathfind best edge %d with score %d steps %d", chosenEdge, bestScore, bestSub);
                for (uint8_t listIdx = 0; listIdx < bestJunctions; listIdx++)
                {
                    LogPathfinding(
                        &peep, "Junction#%d %d,%d,%d Direction %d", listIdx + 1, bestJunctionList[listIdx].x,
                        bestJunctionList[listIdx].y, bestJunctionList[listIdx].z, bestDirectionList[listIdx]);
                }
                LogPathfinding(&peep, "End at %d,%d,%d", bestXYZ.x, bestXYZ.y, bestXYZ.z);
            }
        }

        if (isThin)
        {
            for (std::size_t i = 0; i < peep.pathfindHistory.size(); ++i)
            {
                if (peep.pathfindHistory[i] == loc)
                {
                    /* Peep remembers this junction, so remove the
                     * chosen_edge from those left to try. */
                    peep.pathfindHistory[i].direction &= ~(1 << chosenEdge);
                    /* Also remove the edge through which the peep
                     * entered the junction from those left to try. */
                    peep.pathfindHistory[i].direction &= ~(1 << DirectionReverse(peep.peepDirection));

                    LogPathfinding(
                        &peep, "Updating existing pf_history (in index: %u) for %d,%d,%d without entry edge %d & exit edge %d.",
                        i, loc.x, loc.y, loc.z, DirectionReverse(peep.peepDirection), chosenEdge);

                    return chosenEdge;
                }
            }

            /* Peep does not remember this junction, so forget a junction
             * and remember this junction. */
            int32_t i = peep.pathfindGoal.direction++;
            peep.pathfindGoal.direction &= 3;
            peep.pathfindHistory[i] = { loc, permittedEdges };
            /* Remove the chosen_edge from those left to try. */
            peep.pathfindHistory[i].direction &= ~(1 << chosenEdge);
            /* Also remove the edge through which the peep
             * entered the junction from those left to try. */
            peep.pathfindHistory[i].direction &= ~(1 << DirectionReverse(peep.peepDirection));

            LogPathfinding(
                &peep, "Storing new pf_history (in index: %d) for %d,%d,%d without entry edge %d & exit edge %d.", i, loc.x,
                loc.y, loc.z, DirectionReverse(peep.peepDirection), chosenEdge);
        }

        return chosenEdge;
    }

    /**
     * Gets the nearest park entrance relative to point, by using Manhattan distance.
     * @param x x coordinate of location
     * @param y y coordinate of location
     * @return Index of gParkEntrance (or 0xFF if no park entrances exist).
     */
    static std::optional<CoordsXYZ> GetNearestParkEntrance(const CoordsXY& loc)
    {
        std::optional<CoordsXYZ> chosenEntrance = std::nullopt;
        uint16_t nearestDist = 0xFFFF;
        for (const auto& parkEntrance : getGameState().park.entrances)
        {
            auto dist = abs(parkEntrance.x - loc.x) + abs(parkEntrance.y - loc.y);
            if (dist < nearestDist)
            {
                nearestDist = dist;
                chosenEntrance = parkEntrance;
            }
        }
        return chosenEntrance;
    }

    /**
     *
     *  rct2: 0x006952C0
     */
    int32_t GuestPathFindParkEntranceEntering(Peep& peep, uint8_t edges)
    {
        // Send peeps to the nearest park entrance.
        auto chosenEntrance = GetNearestParkEntrance(peep.nextLoc);

        // If no defined park entrances are found, walk aimlessly.
        if (!chosenEntrance.has_value())
            return GuestPathfindAimless(peep, edges);

        const auto goalPos = TileCoordsXYZ(chosenEntrance.value());
        Direction chosenDirection = ChooseDirection(TileCoordsXYZ{ peep.nextLoc }, goalPos, peep, true, RideId::GetNull());

        if (chosenDirection == kInvalidDirection)
            return GuestPathfindAimless(peep, edges);

        return PeepMoveOneTile(chosenDirection, peep);
    }

    /**
     * Gets the nearest peep spawn relative to point, by using Manhattan distance.
     * @param x x coordinate of location
     * @param y y coordinate of location
     * @return Index of gameState.peepSpawns (or 0xFF if no peep spawns exist).
     */
    static uint8_t GetNearestPeepSpawnIndex(uint16_t x, uint16_t y)
    {
        uint8_t chosenSpawn = 0xFF;
        uint16_t nearestDist = 0xFFFF;
        uint8_t i = 0;
        for (const auto& spawn : getGameState().peepSpawns)
        {
            uint16_t dist = abs(spawn.x - x) + abs(spawn.y - y);
            if (dist < nearestDist)
            {
                nearestDist = dist;
                chosenSpawn = i;
            }
            i++;
        }
        return chosenSpawn;
    }

    /**
     *
     *  rct2: 0x0069536C
     */
    int32_t GuestPathFindPeepSpawn(Peep& peep, uint8_t edges)
    {
        // Send peeps to the nearest spawn point.
        uint8_t chosenSpawn = GetNearestPeepSpawnIndex(peep.nextLoc.x, peep.nextLoc.y);

        // If no defined spawns were found, walk aimlessly.
        if (chosenSpawn == 0xFF)
            return GuestPathfindAimless(peep, edges);

        const auto peepSpawnLoc = getGameState().peepSpawns[chosenSpawn].toTileStart();
        Direction direction = peepSpawnLoc.direction;

        if (peepSpawnLoc.x == peep.nextLoc.x && peepSpawnLoc.y == peep.nextLoc.y)
        {
            return PeepMoveOneTile(direction, peep);
        }

        const auto goalPos = TileCoordsXYZ(peepSpawnLoc);
        direction = ChooseDirection(TileCoordsXYZ{ peep.nextLoc }, goalPos, peep, true, RideId::GetNull());
        if (direction == kInvalidDirection)
            return GuestPathfindAimless(peep, edges);

        return PeepMoveOneTile(direction, peep);
    }

    /**
     *
     *  rct2: 0x00695161
     */
    int32_t GuestPathFindParkEntranceLeaving(Peep& peep, uint8_t edges)
    {
        TileCoordsXYZ entranceGoal{};
        if (peep.peepFlags.has(PeepFlag::parkEntranceChosen))
        {
            entranceGoal = peep.pathfindGoal;
            auto* entranceElement = MapGetParkEntranceElementAt(entranceGoal.toCoordsXYZ(), false);
            // If entrance no longer exists, choose a new one
            if (entranceElement == nullptr)
            {
                peep.peepFlags.unset(PeepFlag::parkEntranceChosen);
            }
        }

        if (!peep.peepFlags.has(PeepFlag::parkEntranceChosen))
        {
            auto chosenEntrance = GetNearestParkEntrance(peep.nextLoc);

            if (!chosenEntrance.has_value())
                return GuestPathfindAimless(peep, edges);

            peep.peepFlags.set(PeepFlag::parkEntranceChosen);
            entranceGoal = TileCoordsXYZ(*chosenEntrance);
        }

        Direction chosenDirection = ChooseDirection(TileCoordsXYZ{ peep.nextLoc }, entranceGoal, peep, true, RideId::GetNull());
        if (chosenDirection == kInvalidDirection)
            return GuestPathfindAimless(peep, edges);

        return PeepMoveOneTile(chosenDirection, peep);
    }

    /**
     *
     *  rct2: 0x006A72C5
     *  param dist is not used.
     *
     * In case where the map element at (x, y) is invalid or there is no entrance
     * or queue leading to it the function will not update its arguments.
     */
    static void GetRideQueueEnd(TileCoordsXYZ& loc)
    {
        TileCoordsXY queueEnd = { 0, 0 };
        TileElement* tileElement = MapGetFirstElementAt(loc);

        if (tileElement == nullptr)
        {
            return;
        }

        bool found = false;
        do
        {
            if (tileElement->getType() != TileElementType::entrance)
                continue;

            if (loc.z != tileElement->baseHeight)
                continue;

            found = true;
            break;
        } while (!(tileElement++)->isLastForTile());

        if (!found)
            return;

        Direction direction = DirectionReverse(tileElement->getDirection());
        TileElement* lastPathElement = nullptr;
        TileElement* firstPathElement = nullptr;

        int16_t baseZ = tileElement->baseHeight;
        TileCoordsXY nextTile = { loc.x, loc.y };

        while (true)
        {
            if (tileElement->getType() == TileElementType::path)
            {
                lastPathElement = tileElement;
                // Update the current queue end
                queueEnd = nextTile;
                // queueEnd.direction = direction;
                if (tileElement->asPath()->isSloped())
                {
                    if (tileElement->asPath()->getSlopeDirection() == direction)
                    {
                        baseZ += 2;
                    }
                }
            }
            nextTile += TileDirectionDelta[direction];

            tileElement = MapGetFirstElementAt(nextTile);
            found = false;
            if (tileElement == nullptr)
                break;
            do
            {
                if (tileElement == firstPathElement)
                    continue;

                if (tileElement->getType() != TileElementType::path)
                    continue;

                if (baseZ == tileElement->baseHeight)
                {
                    if (tileElement->asPath()->isSloped())
                    {
                        if (tileElement->asPath()->getSlopeDirection() != direction)
                        {
                            break;
                        }
                    }
                    found = true;
                    break;
                }

                if (baseZ - 2 == tileElement->baseHeight)
                {
                    if (!tileElement->asPath()->isSloped())
                        break;

                    if (tileElement->asPath()->getSlopeDirection() != DirectionReverse(direction))
                        break;

                    baseZ -= 2;
                    found = true;
                    break;
                }
            } while (!(tileElement++)->isLastForTile());

            if (!found)
                break;

            if (!tileElement->asPath()->isQueue())
                break;

            if (!(tileElement->asPath()->getEdges() & (1 << DirectionReverse(direction))))
                break;

            if (firstPathElement == nullptr)
                firstPathElement = tileElement;

            // More queue to go.
            if (tileElement->asPath()->getEdges() & (1 << (direction)))
                continue;

            direction++;
            direction &= 3;
            // More queue to go.
            if (tileElement->asPath()->getEdges() & (1 << (direction)))
                continue;

            direction = DirectionReverse(direction);
            // More queue to go.
            if (tileElement->asPath()->getEdges() & (1 << (direction)))
                continue;

            break;
        }

        if (loc.z == kMaxTileElementHeight)
            return;

        tileElement = lastPathElement;
        if (tileElement == nullptr)
            return;

        if (!tileElement->asPath()->isQueue())
            return;

        loc.x = queueEnd.x;
        loc.y = queueEnd.y;
        loc.z = tileElement->baseHeight;
    }

    /*
     * If a ride has multiple entrance stations and is set to sync with
     * adjacent stations, cycle through the entrance stations (based on
     * number of rides the peep has been on) so the peep will try the
     * different sections of the ride.
     * In this case, the ride's various entrance stations will typically,
     * though not necessarily, be adjacent to one another and consequently
     * not too far for the peep to walk when cycling between them.
     * Note: the same choice of station must made while the peep navigates
     * to the station. Consequently a truly random station selection here is not
     * appropriate.
     */
    static StationIndex GuestPathfindingSelectRandomStation(
        const Guest& guest, int32_t numEntranceStations, BitSet<Limits::kMaxStationsPerRide>& entranceStations)
    {
        int32_t select = guest.guestNumRides % numEntranceStations;
        while (select > 0)
        {
            for (StationIndex::UnderlyingType i = 0; i < Limits::kMaxStationsPerRide; i++)
            {
                if (entranceStations[i])
                {
                    entranceStations[i] = false;
                    select--;
                    break;
                }
            }
        }
        for (StationIndex::UnderlyingType i = 0; i < Limits::kMaxStationsPerRide; i++)
        {
            if (entranceStations[i])
            {
                return StationIndex::FromUnderlying(i);
            }
        }

        return StationIndex::FromUnderlying(0);
    }
    /**
     *
     *  rct2: 0x00694C35
     */
    static int32_t CalculateNextDestinationDefault(Guest& peep)
    {
        LogPathfinding(&peep, "Starting CalculateNextDestination");

        if (peep.getNextIsSurface())
        {
            return GuestSurfacePathFinding(peep);
        }

        TileCoordsXYZ loc{ peep.nextLoc };

        auto* pathElement = MapGetPathElementAt(loc);
        if (pathElement == nullptr)
        {
            return 1;
        }

        // Because this function is called for guests only, never ignore banners.
        uint32_t edges = PathGetPermittedEdges(false, pathElement);

        if (edges == 0)
        {
            return GuestSurfacePathFinding(peep);
        }

        if (!peep.outsideOfPark && peep.headingForRideOrParkExit())
        {
            /* If this tileElement is adjacent to any non-wide paths,
             * remove all of the edges to wide paths. */
            uint8_t adjustedEdges = edges;
            for (Direction chosenDirection : kAllDirections)
            {
                // If there is no path in that direction try another
                if (!(adjustedEdges & (1 << chosenDirection)))
                    continue;

                /* If there is a wide path in that direction,
                    remove that edge and try another */
                if (FootpathElementNextInDirection(loc, pathElement, chosenDirection) == PathSearchResult::wide)
                {
                    adjustedEdges &= ~(1 << chosenDirection);
                }
            }
            if (adjustedEdges != 0)
                edges = adjustedEdges;
        }

        int32_t direction = DirectionReverse(peep.peepDirection);
        // Check if in a dead end (i.e. only edge is where the peep came from)
        if (!(edges & ~(1 << direction)))
        {
            // In a dead end.  Check if peep is lost, etc.
            if (_captureMoveDirection)
            {
                _capturedDeadEnd = true;
            }
            else
            {
                peep.checkIfLost();
                peep.checkCantFindRide();
                peep.checkCantFindExit();
            }
        }
        else
        {
            /* Not a dead end. Remove edge peep came from so peep will
             * continue on rather than going back where it came from */
            edges &= ~(1 << direction);
        }

        direction = Numerics::bitScanForward(edges);
        // IF only one edge to choose from
        if ((edges & ~(1 << direction)) == 0)
        {
            LogPathfinding(&peep, "Completed CalculateNextDestination - taking only direction available: %d.", direction);

            if (_captureMoveDirection)
                _capturedForced = true;
            return PeepMoveOneTile(direction, peep);
        }

        // Peep still has multiple edges to choose from.

        // Peep is outside the park.
        // Loc694F19:
        if (peep.outsideOfPark)
        {
            LogPathfinding(&peep, "Completed CalculateNextDestination - peep is outside the park.");

            switch (peep.state)
            {
                case PeepState::enteringPark:
                    return GuestPathFindParkEntranceEntering(peep, edges);
                case PeepState::leavingPark:
                    return GuestPathFindPeepSpawn(peep, edges);
                default:
                    return GuestPathfindAimless(peep, edges);
            }
        }

        /* Peep is inside the park.
         * If the peep does not have food, randomly cull the useless directions
         * (dead ends, ride exits, wide paths) from the edges.
         * In principle, peeps with food are not paying as much attention to
         * where they are going and are consequently more like to walk up
         * dead end paths, paths to ride exits, etc. */
        if (!peep.hasFoodOrDrink() && (ScenarioRand() & 0xFFFF) >= 2184)
        {
            uint8_t adjustedEdges = edges;
            for (Direction chosenDirection : kAllDirections)
            {
                // If there is no path in that direction try another
                if (!(adjustedEdges & (1 << chosenDirection)))
                    continue;

                RideId rideIndex = RideId::GetNull();
                auto pathSearchResult = FootpathElementDestinationInDirection(loc, pathElement, chosenDirection, &rideIndex);
                switch (pathSearchResult)
                {
                    case PathSearchResult::deadEnd:
                    case PathSearchResult::rideExit:
                    case PathSearchResult::wide:
                        adjustedEdges &= ~(1 << chosenDirection);
                        break;
                    default:
                        break;
                }
            }
            if (adjustedEdges != 0)
                edges = adjustedEdges;
        }

        /* If there are still multiple directions to choose from,
         * peeps with maps will randomly read the map: probability of doing so
         * is much higher when heading for a ride or the park exit. */
        if (peep.hasItem(ShopItem::map))
        {
            // If at least 2 directions consult map
            if (std::popcount(edges) >= 2)
            {
                uint16_t probability = 1638;
                if (peep.headingForRideOrParkExit())
                {
                    probability = 9362;
                }
                if ((ScenarioRand() & 0xFFFF) < probability)
                {
                    peep.readMap();
                }
            }
        }

        if (peep.peepFlags.has(PeepFlag::leavingPark))
        {
            LogPathfinding(&peep, "Completed CalculateNextDestination - peep is leaving the park.");

            return GuestPathFindParkEntranceLeaving(peep, edges);
        }

        if (peep.guestHeadingToRideId.IsNull())
        {
            LogPathfinding(&peep, "Completed CalculateNextDestination - peep is aimless.");

            return GuestPathfindAimless(peep, edges);
        }

        // Peep is heading for a ride.
        RideId rideIndex = peep.guestHeadingToRideId;
        auto ride = GetRide(rideIndex);
        if (ride == nullptr || ride->status != RideStatus::open)
        {
            LogPathfinding(&peep, "Completed CalculateNextDestination - peep is heading to closed ride == aimless.");

            return GuestPathfindAimless(peep, edges);
        }

        /* Find the ride's closest entrance station to the peep.
         * At the same time, count how many entrance stations there are and
         * which stations are entrance stations. */
        auto bestScore = std::numeric_limits<int32_t>::max();
        StationIndex closestStationNum = StationIndex::FromUnderlying(0);

        int32_t numEntranceStations = 0;
        BitSet<Limits::kMaxStationsPerRide> entranceStations = {};

        for (const auto& station : ride->getStations())
        {
            // Skip if stationNum has no entrance (so presumably an exit only station)
            if (station.entrance.isNull())
                continue;

            const auto stationIndex = ride->getStationIndex(&station);

            numEntranceStations++;
            entranceStations[stationIndex.ToUnderlying()] = true;

            TileCoordsXYZD entranceLocation = station.entrance;
            auto score = CalculateHeuristicPathingScore(entranceLocation, TileCoordsXYZ{ peep.nextLoc });
            if (score < bestScore)
            {
                bestScore = score;
                closestStationNum = stationIndex;
            }
        }

        // Ride has no stations with an entrance, so head to station 0.
        if (numEntranceStations == 0)
            closestStationNum = StationIndex::FromUnderlying(0);

        if (numEntranceStations > 1 && (ride->departFlags & RIDE_DEPART_SYNCHRONISE_WITH_ADJACENT_STATIONS))
        {
            closestStationNum = GuestPathfindingSelectRandomStation(peep, numEntranceStations, entranceStations);
        }

        if (numEntranceStations == 0)
        {
            // closestStationNum is always 0 here.
            const auto& closestStation = ride->getStation(closestStationNum);
            auto entranceXY = TileCoordsXY(closestStation.start);
            loc.x = entranceXY.x;
            loc.y = entranceXY.y;
            loc.z = closestStation.height;
        }
        else
        {
            TileCoordsXYZD entranceXYZD = ride->getStation(closestStationNum).entrance;
            loc.x = entranceXYZD.x;
            loc.y = entranceXYZD.y;
            loc.z = entranceXYZD.z;
        }

        GetRideQueueEnd(loc);

        direction = ChooseDirection(TileCoordsXYZ{ peep.nextLoc }, loc, peep, true, rideIndex);

        if (direction == kInvalidDirection)
        {
            /* Heuristic search failed for all directions.
             * Reset the PathfindGoal - this means that the PathfindHistory
             * will be reset in the next call to ChooseDirection().
             * This lets the heuristic search "try again" in case the player has
             * edited the path layout or the mechanic was already stuck in the
             * save game (e.g. with a worse version of the pathfinding). */
            peep.resetPathfindGoal();

            LogPathfinding(&peep, "Completed CalculateNextDestination - failed to choose a direction == aimless.");

            return GuestPathfindAimless(peep, edges);
        }

        LogPathfinding(&peep, "Completed CalculateNextDestination - direction chosen: %d.", direction);

        return PeepMoveOneTile(direction, peep);
    }

#pragma region Wide path lanes
    /*
     * Wide path lanes (personal fork; first ported from FAshe27/openrct2-claude fix-lost-guests @ 3756da2c9).
     *
     * Where paths sit side by side, FootpathUpdatePathWideFlags sets the wide flag on some of their tiles, and the
     * guest pathfinding avoids wide tiles: CalculateNextDestinationDefault strips edges into them for goal-directed
     * guests, and the heuristic search stops at them. On a 2-wide path that leaves a single lane carrying everyone. On
     * a straight stretch, the wide-edge strip plus the removal of the edge the guest came in by usually leaves one edge,
     * and the single-edge short-circuit then moves the guest without calling ChooseDirection - so the distance fields
     * and their corridor scoring are never consulted there.
     *
     * On paths more than one tile wide (IsOnWidePath):
     *  - Guests with a destination that has a distance field take each step straight from the field (TryFieldLaneStep):
     *    the neighbouring or diagonal flat tile that gets closer, including turning around, preferring their lane. With
     *    no flat move, the field's own step (GuestFieldTryStep); on the destination's doorstep, or with no field, the
     *    default code's decision. They never get a move that leads away from the destination.
     *  - Aimless guests go where the default code's own random choice takes them; walking straight on, they drift
     *    diagonally towards their lane (see the end of CalculateNextDestinationInLanes).
     *  - Each guest's lane comes from GuestSpreadSignedLane - the same per-guest lane PeepMoveOneTile walks them at
     *    within a tile - scored across the path's width by the same centre-distance rule as the field's corridor
     *    scoring (CorridorCandidateScores). The path's width is measured on the rows just behind and ahead too
     *    (GetCorridorLaneRange), so a crossing path doesn't count as width. Lane changes are diagonal moves across a
     *    fully connected flat 2x2 block (IsLaneBlock).
     *
     * Determinism (Franklin's invariant: ScenarioRand draws per decision identical to stock):
     *  - The guest's own tile is evaluated first, in capture mode (PeepMoveOneTile records the direction instead of
     *    moving): this consumes exactly the draws stock CalculateNextDestination makes before its final PeepMoveOneTile.
     *    It's the only evaluation; no other tile is asked about.
     *  - Every decision ends in exactly one move, PeepMoveOneTile or PeepMoveDiagonally, each drawing once - the same
     *    as stock's final PeepMoveOneTile. The default code's decision is followed from the capture, not re-run.
     *  - Field reads and lane choice make no draws.
     * Dead-end checks (checkIfLost etc.) are not run during capture; they run when the guest follows their own tile's
     * decision at a dead end, exactly as in stock.
     *
     * Setting: guests_use_wide_path_lanes in config.ini ([general]), default true. Off = stock behaviour exactly.
     */

    // The widest path (in tiles, either side of the main lane) that guests spread out across.
    static constexpr int32_t kMaxLaneOffset = 6;

    // Must match the lane block in PeepMoveOneTile.
    static constexpr int32_t kWideLaneFurnitureLaneMax = 5;

    struct CapturedDecision
    {
        Direction direction = kInvalidDirection;
        // The guest's only way on was back where they came from.
        bool deadEnd = false;
        // There was only one way to go (no choice was made).
        bool forced = false;
    };

    // Puts PeepMoveOneTile into capture mode and the guest at another tile for its lifetime, restoring all of it (and
    // any capture already in progress) afterwards, the same way HypotheticalQuery does for the generator.
    class CaptureScope
    {
    public:
        CaptureScope(Guest& guest, const TileCoordsXYZ& loc)
            : _guest(guest)
            , _nextLoc(guest.nextLoc)
            , _wasCapturing(_captureMoveDirection)
            , _direction(_capturedMoveDirection)
            , _deadEnd(_capturedDeadEnd)
            , _forced(_capturedForced)
        {
            _guest.nextLoc = loc.toCoordsXYZ();
            _captureMoveDirection = true;
            _capturedMoveDirection = kInvalidDirection;
            _capturedDeadEnd = false;
            _capturedForced = false;
        }

        ~CaptureScope()
        {
            _guest.nextLoc = _nextLoc;
            _captureMoveDirection = _wasCapturing;
            _capturedMoveDirection = _direction;
            _capturedDeadEnd = _deadEnd;
            _capturedForced = _forced;
        }

        CaptureScope(const CaptureScope&) = delete;
        CaptureScope& operator=(const CaptureScope&) = delete;

    private:
        Guest& _guest;
        CoordsXYZ _nextLoc;
        bool _wasCapturing;
        Direction _direction;
        bool _deadEnd;
        bool _forced;
    };

    // Returns the direction CalculateNextDestinationDefault chooses with the guest at loc, without moving them.
    static std::optional<CapturedDecision> CaptureDecisionAt(Guest& peep, const TileCoordsXYZ& loc)
    {
        CaptureScope capture(peep, loc);
        const auto result = CalculateNextDestinationDefault(peep);
        if (result != 0 || _capturedMoveDirection == kInvalidDirection)
            return std::nullopt;
        return CapturedDecision{ _capturedMoveDirection, _capturedDeadEnd, _capturedForced };
    }

    // Returns the flat, non-queue path element at the given tile and height, if guests can walk onto it.
    static PathElement* GetLanePathAt(const TileCoordsXY& loc, int32_t baseHeight)
    {
        const auto coords = loc.toCoordsXY();
        if (!MapIsLocationValid(coords))
            return nullptr;

        auto* pathElement = MapGetPathElementAt(TileCoordsXYZ{ loc, baseHeight });
        if (pathElement == nullptr || pathElement->isSloped() || pathElement->isQueue())
            return nullptr;
        if (!MapIsLocationOwned({ coords, pathElement->getBaseZ() }))
            return nullptr;

        return pathElement;
    }

    static bool LanePathsConnected(const PathElement& from, const PathElement& to, Direction direction)
    {
        return (PathGetPermittedEdges(false, &from) & (1 << direction))
            && (PathGetPermittedEdges(false, &to) & (1 << DirectionReverse(direction)));
    }

    /**
     * Checks that loc, the tile ahead of it, the tile beside it and the tile diagonally ahead of it are all flat paths
     * at the same height that are connected on all four sides, so a guest can walk diagonally across the block.
     */
    static bool IsLaneBlock(const TileCoordsXY& loc, const PathElement& current, Direction forward, Direction side)
    {
        const auto z = current.baseHeight;
        const auto* ahead = GetLanePathAt(loc + TileDirectionDelta[forward], z);
        const auto* beside = GetLanePathAt(loc + TileDirectionDelta[side], z);
        const auto* diagonal = GetLanePathAt(loc + TileDirectionDelta[forward] + TileDirectionDelta[side], z);
        if (ahead == nullptr || beside == nullptr || diagonal == nullptr)
            return false;

        return LanePathsConnected(current, *ahead, forward) && LanePathsConnected(current, *beside, side)
            && LanePathsConnected(*ahead, *diagonal, side) && LanePathsConnected(*beside, *diagonal, forward);
    }

    static TileCoordsXY OffsetTile(const TileCoordsXY& loc, Direction direction, int32_t amount)
    {
        return { loc.x + TileDirectionDelta[direction].x * amount, loc.y + TileDirectionDelta[direction].y * amount };
    }

    /**
     * Returns how far the path extends either side of loc (in tiles towards right, and towards the left as a negative
     * number), following lane paths that are connected to each other.
     */
    static std::pair<int32_t, int32_t> GetLaneRange(const TileCoordsXY& loc, int32_t baseHeight, Direction right)
    {
        std::pair<int32_t, int32_t> range{ 0, 0 };
        const auto* origin = GetLanePathAt(loc, baseHeight);
        if (origin == nullptr)
            return range;

        for (const int32_t sign : { 1, -1 })
        {
            const Direction direction = sign > 0 ? right : DirectionReverse(right);
            const auto* previous = origin;
            for (int32_t i = 1; i <= kMaxLaneOffset; i++)
            {
                const auto* next = GetLanePathAt(OffsetTile(loc, direction, i), baseHeight);
                if (next == nullptr || !LanePathsConnected(*previous, *next, direction))
                    break;
                (sign > 0 ? range.second : range.first) = sign * i;
                previous = next;
            }
        }
        return range;
    }

    /**
     * Like GetLaneRange, but only counts lanes that also exist on the rows just behind and ahead of loc (where those rows
     * are paths). Where another path crosses, the crossing path makes a row look much wider than the path the guest is
     * walking along, so those extra tiles are left out.
     */
    static std::pair<int32_t, int32_t> GetCorridorLaneRange(
        const TileCoordsXY& loc, int32_t baseHeight, Direction forward, Direction right)
    {
        auto range = GetLaneRange(loc, baseHeight, right);
        for (const int32_t rowOffset : { -2, -1, 1, 2 })
        {
            const auto row = OffsetTile(loc, forward, rowOffset);
            if (GetLanePathAt(row, baseHeight) == nullptr)
                continue;
            const auto rowRange = GetLaneRange(row, baseHeight, right);
            range.first = std::max(range.first, rowRange.first);
            range.second = std::min(range.second, rowRange.second);
        }
        return range;
    }

    /**
     * The lane (offset from the main lane, towards right) the guest wants to walk in, out of the lanes from laneLeft to
     * laneRight. Uses the same rule as CorridorCandidateScores in GuestFieldRouting.cpp: the guest's GuestSpreadSignedLane
     * picks the point width * (16 + lane) across the path (in 32nds of a tile, from the side GuestSpreadSignedLane calls
     * negative), and the column whose centre (32 * column + 16) is closest to it wins. So the tile lane and the position
     * within it are the same preference at two scales. On an exact tie (the point on a column boundary), the field's
     * scoring falls back to its per-guest hash order over step directions, which has no counterpart here; the column on
     * the negative side wins instead.
     */
    static int32_t GetPreferredLane(const Guest& peep, Direction forward, Direction right, int32_t laneLeft, int32_t laneRight)
    {
        const int32_t width = laneRight - laneLeft + 1;
        const int32_t lane = GuestSpreadSignedLane(peep.peepId, forward);
        const int32_t target = width * (16 + lane);
        int32_t column = 0;
        int32_t bestDistance = std::numeric_limits<int32_t>::max();
        for (int32_t i = 0; i < width; i++)
        {
            const int32_t distance = std::abs(32 * i + 16 - target);
            if (distance < bestDistance)
            {
                bestDistance = distance;
                column = i;
            }
        }
        const Direction plusSide = (forward == 0 || forward == 2) ? 1 : 2;
        return right == plusSide ? laneLeft + column : laneRight - column;
    }

    /**
     * Same as PeepMoveOneTile, but moves the peep to the tile diagonally ahead (one tile forward and one tile to the
     * side). Only valid when IsLaneBlock() is true for those directions. Draws once, like PeepMoveOneTile, and places
     * the guest at their usual lane within the tile (GuestSpreadSignedLane, pulled in next to path additions).
     */
    static int32_t PeepMoveDiagonally(Direction forward, Direction side, Peep& peep)
    {
        auto newTile = CoordsXY{ CoordsXY{ peep.nextLoc } + CoordsDirectionDelta[forward] + CoordsDirectionDelta[side] }
                           .toTileCentre();

        peep.peepDirection = forward;

        const int8_t offset = (ScenarioRand() & 7) - 3;
        int32_t lane = GuestSpreadSignedLane(peep.peepId, forward);
        if (lane > kWideLaneFurnitureLaneMax || lane < -kWideLaneFurnitureLaneMax)
        {
            const Direction plusSide = (forward == 0 || forward == 2) ? 1 : 2;
            const Direction laneSide = (lane > 0) ? plusSide : DirectionReverse(plusSide);
            if (GuestSpreadPathAdditionOnEdge(newTile, peep, forward, laneSide))
                lane = std::clamp(lane, -kWideLaneFurnitureLaneMax, kWideLaneFurnitureLaneMax);
        }
        if (forward == 0 || forward == 2)
        {
            newTile.x += offset;
            newTile.y += lane;
        }
        else
        {
            newTile.x += lane;
            newTile.y += offset;
        }
        peep.setDestination(newTile, 2);

        return 0;
    }

    // Runs the checks that the regular pathfinding does when a guest reaches a dead end.
    static void CheckGuestAtDeadEnd(Guest& peep)
    {
        peep.checkIfLost();
        peep.checkCantFindRide();
        peep.checkCantFindExit();
    }

    /**
     * The destination a goal-directed guest is walking to, worked out the same way CalculateNextDestinationDefault does
     * (the ride's nearest entrance station or queue end, or the park entrance the guest leaves by), without changing
     * anything. Returns nothing for guests without a destination.
     */
    static std::optional<TileCoordsXYZ> GetGuestGoal(const Guest& peep)
    {
        if (peep.peepFlags.has(PeepFlag::leavingPark))
        {
            if (peep.peepFlags.has(PeepFlag::parkEntranceChosen)
                && MapGetParkEntranceElementAt(TileCoordsXYZ{ peep.pathfindGoal }.toCoordsXYZ(), false) != nullptr)
                return TileCoordsXYZ{ peep.pathfindGoal };
            const auto entrance = GetNearestParkEntrance(peep.nextLoc);
            if (!entrance.has_value())
                return std::nullopt;
            return TileCoordsXYZ(*entrance);
        }

        if (peep.guestHeadingToRideId.IsNull())
            return std::nullopt;
        auto* ride = GetRide(peep.guestHeadingToRideId);
        if (ride == nullptr || ride->status != RideStatus::open)
            return std::nullopt;

        // Same station choice as CalculateNextDestinationDefault.
        auto bestScore = std::numeric_limits<int32_t>::max();
        StationIndex closestStationNum = StationIndex::FromUnderlying(0);
        int32_t numEntranceStations = 0;
        BitSet<Limits::kMaxStationsPerRide> entranceStations = {};
        for (const auto& station : ride->getStations())
        {
            if (station.entrance.isNull())
                continue;
            const auto stationIndex = ride->getStationIndex(&station);
            numEntranceStations++;
            entranceStations[stationIndex.ToUnderlying()] = true;
            auto score = CalculateHeuristicPathingScore(station.entrance, TileCoordsXYZ{ peep.nextLoc });
            if (score < bestScore)
            {
                bestScore = score;
                closestStationNum = stationIndex;
            }
        }
        if (numEntranceStations > 1 && (ride->departFlags & RIDE_DEPART_SYNCHRONISE_WITH_ADJACENT_STATIONS))
            closestStationNum = GuestPathfindingSelectRandomStation(peep, numEntranceStations, entranceStations);

        TileCoordsXYZ goal;
        if (numEntranceStations == 0)
        {
            // Ride has no stations with an entrance, so head to station 0's start (closestStationNum is 0 here).
            const auto& closestStation = ride->getStation(closestStationNum);
            const auto start = TileCoordsXY(closestStation.start);
            goal = { start.x, start.y, closestStation.height };
        }
        else
        {
            const TileCoordsXYZD entrance = ride->getStation(closestStationNum).entrance;
            goal = { entrance.x, entrance.y, entrance.z };
        }
        GetRideQueueEnd(goal);
        return goal;
    }

    /**
     * For guests with a destination that has a distance field: picks the next step straight from the field, among moves
     * to a neighbouring flat path tile or diagonally across a 2x2 block (IsLaneBlock), including turning around. Moves
     * must get closer to the destination, except a diagonal move onwards towards the guest's preferred lane, which may
     * stay as close (see below). Among them the guest takes the one that keeps or brings them nearest their preferred
     * lane, then the one that gets closest, then a straight move over a diagonal one. Returns false (and the lane code below
     * decides) if the field isn't available, the guest is on the destination's doorstep, or no flat move gets closer.
     */
    enum class FieldLaneStep
    {
        moved,    // The guest has been moved.
        noField,  // No destination, or no field for it here: the lane code decides.
        handOver, // On the destination's doorstep, or the field has no step: the default code decides.
    };

    static FieldLaneStep TryFieldLaneStep(Guest& peep, const TileCoordsXYZ& loc, const PathElement& current, int32_t& result)
    {
        const auto goal = GetGuestGoal(peep);
        if (!goal.has_value())
            return FieldLaneStep::noField;
        // A destination without a field here (not built yet, or the guest is off its graph): the default code decides.
        // The lane code below follows decisions that, around wide paths, can flip back and forth between tiles; with a
        // destination that can trap the guest (seen in Franklin's park). Aimless guests choose randomly, so they can't.
        uint32_t distanceHere = 0;
        if (!GuestFieldDistance(*goal, loc, distanceHere))
            return FieldLaneStep::handOver;
        if (distanceHere == 0)
            return FieldLaneStep::handOver;

        const auto tile = TileCoordsXY{ loc };
        bool found = false;
        Direction bestForward = kInvalidDirection;
        std::optional<Direction> bestSide;
        int32_t bestLanePenalty = 0;
        uint32_t bestDistance = 0;

        for (const Direction forward : kAllDirections)
        {
            const auto* ahead = GetLanePathAt(tile + TileDirectionDelta[forward], loc.z);
            if (ahead == nullptr || !LanePathsConnected(current, *ahead, forward))
                continue;

            // The guest's preferred lane for walking this way, as an offset from this tile.
            const Direction right = (forward + 1) & 3;
            const auto [laneLeft, laneRight] = GetCorridorLaneRange(tile, loc.z, forward, right);
            const int32_t target = GetPreferredLane(peep, forward, right, laneLeft, laneRight);

            for (const int32_t sideOffset : { 0, 1, -1 })
            {
                std::optional<Direction> side;
                auto landing = tile + TileDirectionDelta[forward];
                if (sideOffset != 0)
                {
                    side = sideOffset > 0 ? right : DirectionReverse(right);
                    // A diagonal move onwards in the guest's heading is judged as that, not as a sideways shift while
                    // going some other way: "south-west" for a guest heading west is a lane change, not a turn.
                    if (*side == peep.peepDirection)
                        continue;
                    if (!IsLaneBlock(tile, current, forward, *side))
                        continue;
                    landing = landing + TileDirectionDelta[*side];
                }

                uint32_t distance = 0;
                if (!GuestFieldDistance(*goal, { landing, loc.z }, distance) || distance > distanceHere)
                    continue;

                // A move that gets no closer is only allowed diagonally onwards, towards the guest's preferred lane:
                // that's how guests reach lanes further from the destination than the one they're in (e.g. the far
                // side of a path whose destination is on one side). Each such move brings the guest a lane nearer their
                // preferred lane, so they can't keep making them.
                const int32_t lanePenalty = std::abs(target - sideOffset);
                if (distance == distanceHere
                    && (!side.has_value() || forward != peep.peepDirection || lanePenalty >= std::abs(target)))
                    continue;
                const bool better = !found || lanePenalty < bestLanePenalty
                    || (lanePenalty == bestLanePenalty
                        && (distance < bestDistance
                            || (distance == bestDistance && !side.has_value() && bestSide.has_value())));
                if (better)
                {
                    found = true;
                    bestForward = forward;
                    bestSide = side;
                    bestLanePenalty = lanePenalty;
                    bestDistance = distance;
                }
            }
        }

        if (!found)
        {
            // No flat move gets closer (e.g. the way on is a slope, a queue or a narrower path): take the field's own
            // step, which handles every kind of path. Never fall back to the lane code here: it can step the guest
            // away from the destination, and the field would then step them back, over and over.
            Direction fieldDirection = kInvalidDirection;
            if (!GuestFieldTryStep(
                    peep, loc, *goal, static_cast<uint8_t>(PathGetPermittedEdges(false, &current)), fieldDirection))
                return FieldLaneStep::handOver;
            result = PeepMoveOneTile(fieldDirection, peep);
            return FieldLaneStep::moved;
        }
        result = bestSide.has_value() ? PeepMoveDiagonally(bestForward, *bestSide, peep) : PeepMoveOneTile(bestForward, peep);
        return FieldLaneStep::moved;
    }

    static int32_t CalculateNextDestinationInLanes(Guest& peep, const TileCoordsXYZ& loc, PathElement& current)
    {
        const auto tile = TileCoordsXY{ loc };

        // The decision for the guest's own tile, using exactly the draws stock would.
        const auto own = CaptureDecisionAt(peep, loc);
        if (!own.has_value())
            return 1;
        const auto followOwnDecision = [&]() {
            if (own->deadEnd)
                CheckGuestAtDeadEnd(peep);
            return PeepMoveOneTile(own->direction, peep);
        };

        // Guests heading somewhere with a distance field follow the field directly across the whole path.
        int32_t fieldResult = 0;
        switch (TryFieldLaneStep(peep, loc, current, fieldResult))
        {
            case FieldLaneStep::moved:
                return fieldResult;
            case FieldLaneStep::handOver:
                return followOwnDecision();
            case FieldLaneStep::noField:
                break;
        }

        // Aimless guests (no destination). Their direction is the default code's own random choice, as in stock;
        // the lane code only decides which tile across the path they take while walking straight on:
        //  - A sideways step onto another lane of the same path is an artefact of the default code avoiding wide tiles,
        //    not a choice of where to go, so it's treated as walking straight on.
        //  - Walking straight on, the guest drifts diagonally towards their preferred lane where the path allows.
        //  - Anything else (turning into a side path, a shop, a junction branch, turning back at a dead end) is followed
        //    exactly.
        // (An earlier version took decisions from other tiles across the path instead; around junctions and shop fronts
        // that produced sideways steps and U-turns that kept guests circling near Franklin's park entrance.)
        const Direction heading = peep.peepDirection;
        const Direction right = (heading + 1) & 3;
        const auto* ahead = GetLanePathAt(tile + TileDirectionDelta[heading], loc.z);
        const bool canGoOn = ahead != nullptr && LanePathsConnected(current, *ahead, heading);
        if (!canGoOn)
            return followOwnDecision();

        const auto [laneLeft, laneRight] = GetCorridorLaneRange(tile, loc.z, heading, right);
        Direction direction = own->direction;
        if (direction != heading && direction != DirectionReverse(heading))
        {
            const int32_t sideOffset = direction == right ? 1 : -1;
            const auto* beside = GetLanePathAt(tile + TileDirectionDelta[direction], loc.z);
            const bool withinPath = sideOffset >= laneLeft && sideOffset <= laneRight && beside != nullptr
                && LanePathsConnected(current, *beside, direction);
            if (withinPath)
                direction = heading;
        }
        if (direction != heading)
            return followOwnDecision();

        const int32_t target = GetPreferredLane(peep, heading, right, laneLeft, laneRight);
        if (target != 0)
        {
            const Direction side = target > 0 ? right : DirectionReverse(right);
            if (IsLaneBlock(tile, current, heading, side))
                return PeepMoveDiagonally(heading, side, peep);
        }
        return PeepMoveOneTile(heading, peep);
    }

    // True if the guest's tile is part of a path more than one tile wide across their heading.
    static bool IsOnWidePath(const Peep& peep, const TileCoordsXYZ& loc, const PathElement& current)
    {
        if (current.isWide())
            return true;
        const auto tile = TileCoordsXY{ loc };
        for (const Direction side :
             { static_cast<Direction>((peep.peepDirection + 1) & 3), static_cast<Direction>((peep.peepDirection + 3) & 3) })
        {
            const auto* beside = GetLanePathAt(tile + TileDirectionDelta[side], loc.z);
            if (beside != nullptr && LanePathsConnected(current, *beside, side))
                return true;
        }
        return false;
    }

    int32_t CalculateNextDestination(Guest& peep)
    {
        if (!Config::Get().general.guestsUseWidePathLanes || peep.outsideOfPark || peep.state != PeepState::walking
            || peep.getNextIsSurface())
        {
            return CalculateNextDestinationDefault(peep);
        }

        const TileCoordsXYZ loc{ peep.nextLoc };
        auto* current = GetLanePathAt(loc, loc.z);
        if (current == nullptr || !IsOnWidePath(peep, loc, *current))
            return CalculateNextDestinationDefault(peep);

        return CalculateNextDestinationInLanes(peep, loc, *current);
    }
#pragma endregion

} // namespace OpenRCT2::PathFinding
