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
#include "GuestPathfinding.h"

#include <cstdlib>
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

        // --- Corridor lane scoring ---------------------------------------
        //
        // On a straight two- or three-wide corridor the field alone does
        // not spread guests across the columns: guests enter from a feeder
        // path in one column, and where a sideways step is downhill at all
        // (the destination's cone gives the far column a lateral
        // gradient), the fixed hash order picks it or not by luck, so the
        // entry column stays dominant for the whole stretch. The scoring
        // below treats the corridor as ONE road: when a guest has more
        // than one downhill step, the candidates are scored by how close
        // the column each step leads to sits to the guest's own lane -
        // the very lane PeepMoveOneTile will walk them at, via
        // GuestSpreadSignedLane - drawn across the full corridor width. A
        // guest whose lane falls in the far column takes the sideways step
        // as soon as the corridor widens; a guest whose lane is in the
        // entry column goes straight; and because the column and the
        // within-tile lane are the same preference at two scales, the two
        // systems never fight.
        //
        // Hard limits, by design:
        //  - Only strictly-downhill candidates are ever scored. The
        //    candidate set is exactly the one the hash order chooses from
        //    without this scoring; no equal or uphill step is admitted,
        //    so the strict-decrease arrival guarantee is untouched.
        //  - The corridor frame must prove itself: a contiguous row of
        //    path tiles perpendicular to a travel axis, two or three
        //    tiles wide, closed at both ends (no movement-graph step
        //    leads out of the row sideways), every tile connected along
        //    the axis and none sloping sideways. Junctions, plazas,
        //    bends and anything wider fail the proof and keep the plain
        //    hash order.
        //  - Both travel axes are tried; if both frame the row (a small
        //    square block can), the choice is ambiguous and the plain
        //    hash order is kept.
        // Everything here is a pure function of the peepId, the guest's
        // position and heading, and the path network: no state is kept
        // and no ScenarioRand is drawn.

        // One downhill step candidate. A candidate set never exceeds the
        // four compass directions.
        struct FieldStepCandidate
        {
            Direction direction = 0;
            TileCoordsXYZ next{};
        };

        constexpr size_t kMaxFieldStepCandidates = 4;

        // Tests whether a cross-section tile belongs to a straight
        // corridor row for the given travel axis: it must stay connected
        // along the axis (either sense - a row near the corridor's end
        // may only connect backwards) and must not slope sideways out of
        // the row. Slopes ALONG the axis are fine: a corridor climbing a
        // hill is flat across its width on every row.
        bool CorridorTileFitsRow(
            const PathElement* pathElement, Direction axisFwd, Direction axisRev, Direction plusDir, Direction minusDir)
        {
            if (pathElement->isSloped())
            {
                const Direction slopeDir = pathElement->getSlopeDirection();
                if (slopeDir == plusDir || slopeDir == minusDir)
                {
                    return false;
                }
            }
            return (FieldGetPermittedEdges(pathElement) & ((1 << axisFwd) | (1 << axisRev))) != 0;
        }

        // Extends a cross-section from loc in one lateral direction.
        // Returns the number of corridor tiles found on that side (0 when
        // the row simply ends there, which the closure test then
        // verifies), or -1 when a neighbouring path tile disqualifies the
        // row. The end tile and its element are returned for the closure
        // test; they stay at loc / the loc element when the count is 0.
        int32_t CorridorRowExtend(
            const TileCoordsXYZ& loc, const PathElement* locElement, Direction dir, Direction axisFwd, Direction axisRev,
            Direction plusDir, Direction minusDir, TileCoordsXYZ& outEnd, const PathElement*& outEndElement)
        {
            int32_t count = 0;
            TileCoordsXYZ tile = loc;
            outEnd = loc;
            outEndElement = locElement;
            while (count < 3)
            {
                const TileCoordsXYZ nextTile{ tile.x + TileDirectionDelta[dir].x, tile.y + TileDirectionDelta[dir].y, loc.z };
                const PathElement* element = MapGetPathElementAt(nextTile);
                if (element == nullptr || element->isQueue())
                {
                    break;
                }
                if (!CorridorTileFitsRow(element, axisFwd, axisRev, plusDir, minusDir))
                {
                    return -1;
                }
                tile = nextTile;
                count++;
                outEnd = tile;
                outEndElement = element;
            }
            return count;
        }

        // A cross-section end is closed when no movement-graph step leads
        // out of the row there: the step test fails, or lands somewhere
        // that is not a graph node (a queue tile, or no path element at
        // all). A step landing on another path element - even a stacked
        // one at another height - means the row leaks into a junction or
        // a crossing, so the row is not a simple corridor cross-section.
        bool CorridorRowEndClosed(const TileCoordsXYZ& end, const PathElement* endElement, Direction outwardDir)
        {
            TileCoordsXYZ stepped;
            if (!FieldStepTo(end, endElement, outwardDir, stepped))
            {
                return true;
            }
            const PathElement* steppedElement = MapGetPathElementAt(stepped);
            return steppedElement == nullptr || steppedElement->isQueue();
        }

        // Attempts to frame the row through loc as a corridor cross-
        // section perpendicular to one travel axis. On success returns
        // true and fills outColumns (the column index each candidate
        // leads to, parallel to candidates), outWidth and outSense (the
        // travel direction along the axis the candidates imply). Any
        // doubt returns false and the caller keeps the plain hash order.
        bool CorridorFrameForAxis(
            const Peep& peep, const TileCoordsXYZ& loc, const PathElement* pathElement, uint8_t permittedEdges,
            bool travelAlongX, const FieldStepCandidate* candidates, size_t candidateCount, int32_t* outColumns,
            int32_t& outWidth, Direction& outSense)
        {
            const Direction axisFwd = travelAlongX ? 0 : 1;
            const Direction axisRev = DirectionReverse(axisFwd);
            // The lateral direction a positive lane leans towards, as in
            // PeepMoveOneTile: direction 1 on X-travel, direction 2 on
            // Y-travel. Column indices increase in this direction.
            const Direction plusDir = travelAlongX ? 1 : 2;
            const Direction minusDir = DirectionReverse(plusDir);

            // The guest's own tile must belong to such a row: connected
            // along the axis, and not sloping sideways itself.
            if ((permittedEdges & ((1 << axisFwd) | (1 << axisRev))) == 0)
            {
                return false;
            }
            if (pathElement->isSloped())
            {
                const Direction slopeDir = pathElement->getSlopeDirection();
                if (slopeDir == plusDir || slopeDir == minusDir)
                {
                    return false;
                }
            }

            TileCoordsXYZ minusEnd;
            const PathElement* minusEndElement = nullptr;
            const int32_t minusCount = CorridorRowExtend(
                loc, pathElement, minusDir, axisFwd, axisRev, plusDir, minusDir, minusEnd, minusEndElement);
            if (minusCount < 0)
            {
                return false;
            }
            TileCoordsXYZ plusEnd;
            const PathElement* plusEndElement = nullptr;
            const int32_t plusCount = CorridorRowExtend(
                loc, pathElement, plusDir, axisFwd, axisRev, plusDir, minusDir, plusEnd, plusEndElement);
            if (plusCount < 0)
            {
                return false;
            }

            const int32_t width = minusCount + 1 + plusCount;
            if (width < 2 || width > 3)
            {
                return false;
            }
            if (!CorridorRowEndClosed(minusEnd, minusEndElement, minusDir)
                || !CorridorRowEndClosed(plusEnd, plusEndElement, plusDir))
            {
                return false;
            }

            const int32_t columnHere = minusCount;

            // The travel sense along the axis: when the candidates offer
            // exactly one axis sense, that is the sense the guest is
            // travelling (this also keeps the frame correct on the
            // consultation right after a sideways step, when the guest's
            // heading still points along the old lateral step). When both
            // senses are on offer (a field ridge), the guest's current
            // heading decides; anything else means this frame does not
            // describe the choice at hand.
            bool hasFwd = false;
            bool hasRev = false;
            for (size_t i = 0; i < candidateCount; i++)
            {
                if (candidates[i].direction == axisFwd)
                {
                    hasFwd = true;
                }
                else if (candidates[i].direction == axisRev)
                {
                    hasRev = true;
                }
            }
            if (!hasFwd && !hasRev)
            {
                return false;
            }
            Direction sense;
            if (hasFwd != hasRev)
            {
                sense = hasFwd ? axisFwd : axisRev;
            }
            else if (peep.peepDirection == axisFwd || peep.peepDirection == axisRev)
            {
                sense = peep.peepDirection;
            }
            else
            {
                return false;
            }

            // Every candidate must have a column in this frame: axis
            // steps keep the guest's column; lateral steps move one
            // column and must land exactly on the framed row tile, not on
            // some other element stacked on the neighbouring tile.
            for (size_t i = 0; i < candidateCount; i++)
            {
                const Direction d = candidates[i].direction;
                if (d == plusDir)
                {
                    const TileCoordsXYZ expected{ loc.x + TileDirectionDelta[plusDir].x, loc.y + TileDirectionDelta[plusDir].y,
                                                  loc.z };
                    if (plusCount == 0 || candidates[i].next != expected)
                    {
                        return false;
                    }
                    outColumns[i] = columnHere + 1;
                }
                else if (d == minusDir)
                {
                    const TileCoordsXYZ expected{ loc.x + TileDirectionDelta[minusDir].x,
                                                  loc.y + TileDirectionDelta[minusDir].y, loc.z };
                    if (minusCount == 0 || candidates[i].next != expected)
                    {
                        return false;
                    }
                    outColumns[i] = columnHere - 1;
                }
                else
                {
                    outColumns[i] = columnHere;
                }
            }

            outWidth = width;
            outSense = sense;
            return true;
        }

        // Scores the downhill candidates by corridor-lane fit and
        // returns true when a corridor frame applies. outScores is
        // parallel to candidates; lower is a better fit. Returns false -
        // and the caller keeps the plain hash order - when neither travel
        // axis frames the row, or when both do (ambiguous).
        bool CorridorCandidateScores(
            const Peep& peep, const TileCoordsXYZ& loc, const PathElement* pathElement, uint8_t permittedEdges,
            const FieldStepCandidate* candidates, size_t candidateCount, int32_t* outScores)
        {
            int32_t columnsX[kMaxFieldStepCandidates];
            int32_t columnsY[kMaxFieldStepCandidates];
            int32_t widthX = 0;
            int32_t widthY = 0;
            Direction senseX = 0;
            Direction senseY = 0;
            const bool framedX = CorridorFrameForAxis(
                peep, loc, pathElement, permittedEdges, true, candidates, candidateCount, columnsX, widthX, senseX);
            const bool framedY = CorridorFrameForAxis(
                peep, loc, pathElement, permittedEdges, false, candidates, candidateCount, columnsY, widthY, senseY);
            if (framedX == framedY)
            {
                return false;
            }
            const int32_t* columns = framedX ? columnsX : columnsY;
            const int32_t width = framedX ? widthX : widthY;
            const Direction sense = framedX ? senseX : senseY;

            // The guest's lane fraction - (16 + lane) of one 32-unit
            // tile - drawn across the whole corridor width picks a point
            // in the corridor; each candidate is scored by the distance
            // from that point to the centre of the column the step leads
            // to, so the columns fill in proportion to the lane
            // distribution. (Scoring by the guest's settled foot position
            // within the column instead would count the lane twice - once
            // in the target, once in the position - and collapse
            // three-wide corridors into their middle column.)
            const int32_t lane = GuestSpreadSignedLane(peep.peepId, sense);
            const int32_t target = width * (16 + lane);
            for (size_t i = 0; i < candidateCount; i++)
            {
                const int32_t diff = 32 * columns[i] + 16 - target;
                outScores[i] = diff < 0 ? -diff : diff;
            }
            return true;
        }

        // The field distance recorded for a node, when the field covers it.
        bool FieldDistanceAt(const FieldData& field, const TileCoordsXYZ& node, uint32_t& outDistance)
        {
            const auto it = field.distance.find(PackNode(node));
            if (it == field.distance.end())
            {
                return false;
            }
            outDistance = it->second;
            return true;
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

        // The downhill candidates, in fixed direction order: the steps
        // whose target node is strictly closer to the goal. This set is
        // exactly the one the plain hash order has always chosen from.
        FieldStepCandidate candidates[kMaxFieldStepCandidates];
        size_t candidateCount = 0;
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

            candidates[candidateCount] = FieldStepCandidate{ d, next };
            candidateCount++;
        }
        if (candidateCount == 0)
        {
            return false;
        }

        // With several downhill steps, corridor-lane scoring gets first
        // say where a corridor frame applies (see the corridor block
        // above); the per-guest hash order breaks the remaining ties and
        // decides everywhere else, exactly as before.
        int32_t corridorScores[kMaxFieldStepCandidates] = {};
        const bool useCorridorScores = candidateCount >= 2
            && CorridorCandidateScores(peep, loc, pathElement, permittedEdges, candidates, candidateCount, corridorScores);

        size_t best = 0;
        uint32_t bestRank = FieldPreferenceRank(peep.peepId, candidates[0].direction);
        for (size_t i = 1; i < candidateCount; i++)
        {
            const uint32_t rank = FieldPreferenceRank(peep.peepId, candidates[i].direction);
            const bool better = useCorridorScores
                ? (corridorScores[i] < corridorScores[best] || (corridorScores[i] == corridorScores[best] && rank < bestRank))
                : (rank < bestRank);
            if (better)
            {
                best = i;
                bestRank = rank;
            }
        }

        outDirection = candidates[best].direction;
        return true;
    }

    bool GuestFieldDistance(const TileCoordsXYZ& goal, const TileCoordsXYZ& node, uint32_t& outDistance)
    {
        if (!NodePackable(node))
        {
            return false;
        }
        const FieldData* field = GetFieldForGoal(goal);
        if (field == nullptr)
        {
            return false;
        }
        return FieldDistanceAt(*field, node, outDistance);
    }

} // namespace OpenRCT2::PathFinding
