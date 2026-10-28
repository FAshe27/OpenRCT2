/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "GuestFieldRouting.h"

#include "../GameState.h"
#include "../entity/Guest.h"
#include "../ride/RideData.h"
#include "../world/Footpath.h"
#include "../world/Map.h"
#include "../world/tile_element/BannerElement.h"
#include "../world/tile_element/EntranceElement.h"
#include "../world/tile_element/PathElement.h"
#include "../world/tile_element/TileElement.h"
#include "../world/tile_element/TrackElement.h"

#include <deque>
#include <unordered_map>
#include <vector>

namespace OpenRCT2::PathFinding
{
    namespace
    {
        // At most this many destination fields are kept at once. Fields are
        // shared by every guest heading to the same destination, and both
        // eviction and staleness are safe (a missing field just means the
        // stock search runs, and a needed field rebuilds lazily), so the cap
        // only bounds memory.
        constexpr size_t kMaxGuestFields = 32;

        // Path-network edits can dirty every field at once (e.g. a mass
        // footpath relay). Rebuilds are spread over ticks so the first
        // consultations after an edit cannot hitch the game; guests whose
        // field is still pending use the stock search meanwhile.
        constexpr uint32_t kMaxFieldBuildsPerTick = 2;

        // Safety valve: a field build that would exceed this many nodes is
        // abandoned and the destination stays on the stock search. Only
        // reachable on absurd maps (the technical map size is 1001x1001);
        // a fully paved 256x256 park with stacked paths stays far below it.
        constexpr size_t kMaxFieldNodes = 1 << 19;

        struct FieldData
        {
            TileCoordsXYZ goal{};
            // The invalidation generation this field was built against.
            uint32_t generation = 0;
            // Monotonic access counter for LRU eviction.
            uint64_t lastUsed = 0;
            // Packed node -> exact step distance to the goal's source set.
            // Nodes absent from the map are unreached (unreachable).
            std::unordered_map<uint32_t, uint32_t> distance;
        };

        std::vector<FieldData> _fields;
        uint32_t _fieldsGeneration = 1;
        uint64_t _fieldsUseCounter = 0;
        uint32_t _fieldBuildTick = 0xFFFFFFFF;
        uint32_t _fieldBuildsThisTick = 0;

        // Node identity: a path element is identified by its tile and its
        // base height, so stacked path elements on one tile (different z)
        // are distinct nodes. Tile coordinates are below 1024 and base
        // heights below 256 on every real map, so the packing is lossless.
        uint32_t PackNode(const TileCoordsXYZ& loc)
        {
            return static_cast<uint32_t>(loc.x) | (static_cast<uint32_t>(loc.y) << 10) | (static_cast<uint32_t>(loc.z) << 20);
        }

        bool NodePackable(const TileCoordsXYZ& loc)
        {
            return loc.x >= 0 && loc.x < 1024 && loc.y >= 0 && loc.y < 1024 && loc.z >= 0 && loc.z < 4096;
        }

        // The banner helpers below mirror the statics of the same name in
        // GuestPathfinding.cpp exactly: a 'no entry' banner on a tile clears
        // the matching edges of the path element on that tile, and guests
        // never ignore banners.
        const TileElement* GetBannerOnPath(const TileElement* pathElement)
        {
            if (pathElement->isLastForTile())
                return nullptr;

            const TileElement* bannerElement = pathElement + 1;
            do
            {
                if (bannerElement->getType() == TileElementType::path)
                    return nullptr;
                if (bannerElement->getType() == TileElementType::banner)
                    return bannerElement;
                if (bannerElement->isLastForTile())
                    return nullptr;

            } while (bannerElement++ != nullptr);

            return nullptr;
        }

        int32_t BannerClearPathEdges(const PathElement* pathElement, int32_t edges)
        {
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

        int32_t FieldGetPermittedEdges(const PathElement* pathElement)
        {
            return BannerClearPathEdges(pathElement, pathElement->getEdgesAndCorners()) & 0x0F;
        }

        // The height a walker leaving the node (from, fromElement) in
        // direction d arrives at on the next tile: standing on a slope that
        // rises in direction d puts them one level (2 z units) up. This is
        // the same adjustment FootpathElementNextInDirection performs.
        int32_t StepArrivalZ(const TileCoordsXYZ& from, const PathElement* fromElement, Direction d)
        {
            int32_t arrivalZ = from.z;
            if (fromElement->isSloped() && fromElement->getSlopeDirection() == d)
            {
                arrivalZ += 2;
            }
            return arrivalZ;
        }

        // One movement-graph step: can a walker standing on the node
        // (from, fromElement) step in direction d, and if so, onto which
        // node? Mirrors FootpathElementNextInDirection: the destination
        // tile's first path element (in element order) that accepts the
        // walker's arrival height via FootpathIsZAndDirectionValid is the
        // node stepped onto. Queue elements are reported like any other
        // node here; graph membership (non-queue only) is the caller's
        // concern, because the ride-queue tail needs exactly this test.
        bool FieldStepTo(const TileCoordsXYZ& from, const PathElement* fromElement, Direction d, TileCoordsXYZ& outNext)
        {
            const int32_t arrivalZ = StepArrivalZ(from, fromElement, d);
            TileCoordsXYZ destLoc{ from.x + TileDirectionDelta[d].x, from.y + TileDirectionDelta[d].y, arrivalZ };

            TileElement* tileElement = MapGetFirstElementAt(destLoc.toCoordsXY());
            if (tileElement == nullptr)
            {
                return false;
            }
            do
            {
                if (tileElement->isGhost())
                    continue;
                if (tileElement->getType() != TileElementType::path)
                    continue;

                const auto* pathElement = tileElement->asPath();
                if (!FootpathIsZAndDirectionValid(*pathElement, arrivalZ, d))
                    continue;

                outNext = TileCoordsXYZ{ destLoc.x, destLoc.y, tileElement->baseHeight };
                return true;
            } while (!(tileElement++)->isLastForTile());

            return false;
        }

        // Collects the source nodes (distance 0) for a step-target tile that
        // is not itself a graph node: the entrance tile of a ride or the
        // park, the tile of a shop, or a ride's queue tail. A graph node v
        // is a source if a walker standing on v can step onto the target:
        // for path-element targets (queue tails) the step must land exactly
        // on the target node; for element targets (entrances, shops) the
        // step's arrival height must equal the goal height, which is the
        // test the stock search applies to those elements.
        // facingOnly >= 0 restricts the approach to one direction: entrances
        // are only ever entered against their facing (the stock search
        // recognises an entrance only when the step direction equals the
        // entrance's direction), so an entrance's only source is the node
        // its facing points from.
        void CollectStepSources(
            const TileCoordsXYZ& goal, bool targetIsPathNode, int32_t facingOnly, std::vector<TileCoordsXYZ>& sources)
        {
            for (Direction d : kAllDirections)
            {
                if (facingOnly >= 0 && d != facingOnly)
                    continue;

                const TileCoordsXYZ neighbourTile{ goal.x - TileDirectionDelta[d].x, goal.y - TileDirectionDelta[d].y, 0 };
                if (neighbourTile.x < 0 || neighbourTile.y < 0)
                    continue;

                TileElement* tileElement = MapGetFirstElementAt(neighbourTile.toCoordsXY());
                if (tileElement == nullptr)
                {
                    continue;
                }
                do
                {
                    if (tileElement->isGhost())
                        continue;
                    if (tileElement->getType() != TileElementType::path)
                        continue;

                    const auto* pathElement = tileElement->asPath();
                    // Queue tiles are never graph nodes (queues are their own
                    // system); a guest routes to the tail's doorstep instead.
                    if (pathElement->isQueue())
                        continue;

                    const TileCoordsXYZ node{ neighbourTile.x, neighbourTile.y, tileElement->baseHeight };
                    if (!NodePackable(node))
                        continue;

                    // The step onto the target must be one this node's own
                    // edges permit (one-way connections and 'no entry'
                    // banners included).
                    if (!(FieldGetPermittedEdges(pathElement) & (1 << d)))
                        continue;

                    if (targetIsPathNode)
                    {
                        TileCoordsXYZ stepped;
                        if (FieldStepTo(node, pathElement, d, stepped) && stepped == goal)
                        {
                            sources.push_back(node);
                        }
                    }
                    else if (StepArrivalZ(node, pathElement, d) == goal.z)
                    {
                        sources.push_back(node);
                    }
                } while (!(tileElement++)->isLastForTile());
            }
        }

        // Resolves the source set for a goal tile, following what the stock
        // code treats as "arrived" for that goal (the goal tile itself is
        // the one CalculateNextDestination hands to ChooseDirection):
        //  - park / ride entrance tile: the node the entrance faces onto.
        //  - shop tile (a track element of a shop or facility ride): every
        //    node that can step up to it, from any direction.
        //  - queue tail (a queue path element): every ordinary node that
        //    can step onto it.
        //  - any other path tile (e.g. a peep spawn): the node itself.
        // Returns false when the goal tile holds nothing a field can root
        // at; the caller then leaves the guest on the stock search.
        bool ResolveFieldSources(const TileCoordsXYZ& goal, std::vector<TileCoordsXYZ>& sources)
        {
            TileElement* tileElement = MapGetFirstElementAt(goal.toCoordsXY());
            if (tileElement != nullptr)
            {
                do
                {
                    if (tileElement->isGhost())
                        continue;
                    if (tileElement->baseHeight != goal.z)
                        continue;

                    if (tileElement->getType() == TileElementType::entrance)
                    {
                        const auto entranceType = tileElement->asEntrance()->getEntranceType();
                        if (entranceType == EntranceType::parkEntrance || entranceType == EntranceType::rideEntrance)
                        {
                            CollectStepSources(goal, false, tileElement->getDirection(), sources);
                            return true;
                        }
                    }
                    else if (tileElement->getType() == TileElementType::track)
                    {
                        auto ride = GetRide(tileElement->asTrack()->getRideIndex());
                        if (ride != nullptr && ride->getRideTypeDescriptor().flags.has(RtdFlag::isShopOrFacility))
                        {
                            CollectStepSources(goal, false, -1, sources);
                            return true;
                        }
                    }
                } while (!(tileElement++)->isLastForTile());
            }

            const PathElement* pathElement = MapGetPathElementAt(goal);
            if (pathElement == nullptr)
            {
                return false;
            }
            if (pathElement->isQueue())
            {
                CollectStepSources(goal, true, -1, sources);
            }
            else
            {
                sources.push_back(goal);
            }
            return true;
        }

        // Reverse breadth-first search from the source set over the
        // movement graph. Neighbours are enumerated in fixed direction
        // order and first visit wins; with unit step costs the recorded
        // distance of a node is its exact shortest step count to the goal,
        // independent of visit order, so the field is a pure function of
        // the path network. Wide path tiles are ordinary nodes; queue
        // tiles are excluded. No ScenarioRand is drawn anywhere here.
        bool BuildField(FieldData& field)
        {
            std::vector<TileCoordsXYZ> sources;
            if (!ResolveFieldSources(field.goal, sources))
            {
                return false;
            }

            std::deque<TileCoordsXYZ> queue;
            for (const auto& source : sources)
            {
                if (NodePackable(source) && field.distance.emplace(PackNode(source), 0u).second)
                {
                    queue.push_back(source);
                }
            }

            while (!queue.empty())
            {
                const TileCoordsXYZ node = queue.front();
                queue.pop_front();
                const uint32_t nodeDistance = field.distance[PackNode(node)];

                // Predecessors: nodes on the tile behind each direction
                // whose own step in that direction lands exactly here.
                for (Direction d : kAllDirections)
                {
                    const TileCoordsXYZ predTile{ node.x - TileDirectionDelta[d].x, node.y - TileDirectionDelta[d].y, 0 };
                    if (predTile.x < 0 || predTile.y < 0)
                    {
                        continue;
                    }

                    TileElement* tileElement = MapGetFirstElementAt(predTile.toCoordsXY());
                    if (tileElement == nullptr)
                    {
                        continue;
                    }
                    do
                    {
                        if (tileElement->isGhost())
                            continue;
                        if (tileElement->getType() != TileElementType::path)
                            continue;

                        const auto* pathElement = tileElement->asPath();
                        if (pathElement->isQueue())
                            continue;

                        const TileCoordsXYZ pred{ predTile.x, predTile.y, tileElement->baseHeight };
                        if (!NodePackable(pred) || field.distance.contains(PackNode(pred)))
                        {
                            continue;
                        }

                        // The step from the predecessor into this node must
                        // be one the predecessor's own edges permit (one-way
                        // connections and 'no entry' banners included);
                        // without this the graph would promise steps the
                        // walker cannot take.
                        if (!(FieldGetPermittedEdges(pathElement) & (1 << d)))
                        {
                            continue;
                        }

                        TileCoordsXYZ stepped;
                        if (!FieldStepTo(pred, pathElement, d, stepped) || stepped != node)
                        {
                            continue;
                        }

                        field.distance.emplace(PackNode(pred), nodeDistance + 1);
                        queue.push_back(pred);
                        if (field.distance.size() > kMaxFieldNodes)
                        {
                            return false;
                        }
                    } while (!(tileElement++)->isLastForTile());
                }
            }
            return true;
        }

        // Returns the field for goal, building or rebuilding it if needed
        // and the per-tick build budget allows; nullptr when the caller
        // must fall back to the stock search for this consultation.
        const FieldData* GetFieldForGoal(const TileCoordsXYZ& goal)
        {
            FieldData* slot = nullptr;
            for (auto& field : _fields)
            {
                if (field.goal == goal)
                {
                    slot = &field;
                    break;
                }
            }
            if (slot != nullptr && slot->generation == _fieldsGeneration)
            {
                slot->lastUsed = ++_fieldsUseCounter;
                return slot;
            }

            const uint32_t tick = getGameState().currentTicks;
            if (tick != _fieldBuildTick)
            {
                _fieldBuildTick = tick;
                _fieldBuildsThisTick = 0;
            }
            if (_fieldBuildsThisTick >= kMaxFieldBuildsPerTick)
            {
                return nullptr;
            }
            _fieldBuildsThisTick++;

            if (slot == nullptr)
            {
                if (_fields.size() >= kMaxGuestFields)
                {
                    // Evict the least recently used field. Eviction is safe:
                    // its guests fall back to the stock search until the
                    // field is requested again and rebuilds.
                    auto lru = _fields.begin();
                    for (auto it = _fields.begin(); it != _fields.end(); ++it)
                    {
                        if (it->lastUsed < lru->lastUsed)
                        {
                            lru = it;
                        }
                    }
                    _fields.erase(lru);
                }
                _fields.push_back(FieldData{});
                slot = &_fields.back();
                slot->goal = goal;
            }

            slot->distance.clear();
            if (!BuildField(*slot))
            {
                // Unbuildable (no rootable goal, or the node cap tripped):
                // leave an empty field stamped with the current generation,
                // so the destination stays on the stock search - without a
                // rebuild attempt on every consultation - until the next
                // path-network edit.
                slot->distance.clear();
            }
            slot->generation = _fieldsGeneration;
            slot->lastUsed = ++_fieldsUseCounter;
            return slot;
        }

        // Deterministic per-guest preference over step directions, in the
        // same peepId-hash style as the lane spread in PeepMoveOneTile: a
        // guest's favourite downhill direction is stable, unrelated guests
        // prefer unrelated directions, and no ScenarioRand is drawn. This
        // is what keeps a two-wide path flowing on both columns: both are
        // equally downhill, and different guests prefer different ones.
        uint32_t FieldPreferenceRank(uint32_t peepId, Direction direction)
        {
            uint32_t hash = peepId * 2654435761u;
            hash ^= (static_cast<uint32_t>(direction) + 1u) * 2246822519u;
            hash ^= hash >> 16;
            return hash;
        }

    } // namespace

    void InvalidateGuestFields()
    {
        _fieldsGeneration++;
    }

    void ClearGuestFields()
    {
        _fields.clear();
        _fieldsGeneration++;
    }

    bool GuestFieldTryStep(
        Peep& peep, const TileCoordsXYZ& loc, const TileCoordsXYZ& goal, uint8_t permittedEdges, Direction& outDirection)
    {
        // Guests only - ChooseDirection is shared with staff pathfinding,
        // and staff routing is untouched by design.
        auto* guest = peep.as<Guest>();
        if (guest == nullptr)
        {
            return false;
        }

        // Gate to exactly the goal-directed guest consultations the stock
        // code routes through ChooseDirection: entering the park (goal =
        // park entrance), leaving to a peep spawn (goal = spawn tile),
        // leaving the park (goal = chosen park entrance), and heading to a
        // ride - which in this code also covers shops, stalls and toilets,
        // since those are rides whose goal tile is the shop tile. Aimless
        // guests never reach ChooseDirection, and neither do queuing or
        // on-ride guests.
        if (guest->outsideOfPark)
        {
            if (peep.state != PeepState::enteringPark && peep.state != PeepState::leavingPark)
            {
                return false;
            }
        }
        else if (!guest->peepFlags.has(PeepFlag::leavingPark) && guest->guestHeadingToRideId.IsNull())
        {
            return false;
        }

        if (!NodePackable(loc))
        {
            return false;
        }

        const FieldData* field = GetFieldForGoal(goal);
        if (field == nullptr)
        {
            return false;
        }

        const auto it = field->distance.find(PackNode(loc));
        if (it == field->distance.end())
        {
            // The guest's node is unreached: either there is no route (which
            // the guest now knows before walking into a dead end) or the
            // guest is off the graph (e.g. standing on a queue tile). Either
            // way the stock search decides, exactly as today.
            return false;
        }
        const uint32_t distanceHere = it->second;
        if (distanceHere == 0)
        {
            // On the destination's doorstep node. The final step onto the
            // entrance element, the queue tail or the spawn tile belongs to
            // the stock code (its interaction handlers key on that step), so
            // the field hands over here.
            return false;
        }

        const PathElement* pathElement = MapGetPathElementAt(loc);
        if (pathElement == nullptr)
        {
            return false;
        }

        bool found = false;
        uint32_t bestRank = 0xFFFFFFFF;
        Direction bestDirection = 0;
        for (Direction d : kAllDirections)
        {
            if (!(permittedEdges & (1 << d)))
            {
                continue;
            }

            TileCoordsXYZ next;
            if (!FieldStepTo(loc, pathElement, d, next))
            {
                continue;
            }
            // Queue nodes are never in the field, so a step onto a queue
            // tile simply finds no distance and is skipped here; the stock
            // code makes that step itself from the tail's doorstep node
            // (see distanceHere == 0 above).
            const auto nextIt = field->distance.find(PackNode(next));
            if (nextIt == field->distance.end() || nextIt->second >= distanceHere)
            {
                continue;
            }

            const uint32_t rank = FieldPreferenceRank(peep.peepId, d);
            if (!found || rank < bestRank)
            {
                found = true;
                bestRank = rank;
                bestDirection = d;
            }
        }
        if (!found)
        {
            return false;
        }

        outDirection = bestDirection;
        return true;
    }

} // namespace OpenRCT2::PathFinding
