/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#pragma once

// Pure, game-independent grid logic for v2 plaza walking (personal fork).
//
// Everything in this header operates on plain integer grids so that the exact
// code the game runs can also be compiled into a standalone test harness
// (see the big-build plaza workstream harness). No game headers, no globals,
// no RNG: every function is a deterministic function of its inputs.
//
// Conventions:
//  - Tiles are addressed by (x, y) tile coordinates; grids are row-major
//    with index y * width + x.
//  - Directions match the game: 0 = -x, 1 = +y, 2 = +x, 3 = -y
//    (TileDirectionDelta in world/Map.cpp).
//  - World coordinates are the game's: 32 units per tile on x/y.
//  - A "flat path z" grid holds, per tile, the base height (in the game's
//    path-element baseHeight units) of the tile's flat, non-queue,
//    non-ghost path element, or kNoFlatPath (-1) when there is none, or
//    kAmbiguousFlatPath (-2) when flat path elements exist at more than one
//    height (overlaid paths at different heights). Ambiguous tiles are
//    never plaza tiles: which height a guest is on cannot be told from the
//    2D grid alone, so the safe answer is to not steer there.

#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <queue>
#include <vector>

namespace OpenRCT2::PlazaGrid
{
    inline constexpr int32_t kNoFlatPath = -1;
    inline constexpr int32_t kAmbiguousFlatPath = -2;
    inline constexpr int32_t kTileSize = 32;
    inline constexpr int32_t kWalkStep = 2;      // Peep::updateWalkingAction step size
    inline constexpr int32_t kWalkTolerance = 2; // destination tolerance used for steering

    struct Tile
    {
        int32_t x{};
        int32_t y{};

        constexpr bool operator==(const Tile& other) const
        {
            return x == other.x && y == other.y;
        }
        constexpr bool operator!=(const Tile& other) const
        {
            return !(*this == other);
        }
    };

    struct WorldPoint
    {
        int32_t x{};
        int32_t y{};

        constexpr bool operator==(const WorldPoint& other) const
        {
            return x == other.x && y == other.y;
        }
    };

    inline constexpr Tile TileInDirection(Tile tile, int32_t direction)
    {
        // Mirrors TileDirectionDelta[0..3].
        switch (direction & 3)
        {
            case 0:
                return { tile.x - 1, tile.y };
            case 1:
                return { tile.x, tile.y + 1 };
            case 2:
                return { tile.x + 1, tile.y };
            default:
                return { tile.x, tile.y - 1 };
        }
    }

    inline constexpr int32_t ReverseDirection(int32_t direction)
    {
        return direction ^ 2;
    }

    inline constexpr int32_t GridIndex(int32_t width, Tile tile)
    {
        return tile.y * width + tile.x;
    }

    inline constexpr bool InBounds(int32_t width, int32_t height, Tile tile)
    {
        return tile.x >= 0 && tile.y >= 0 && tile.x < width && tile.y < height;
    }

    inline constexpr WorldPoint TileCentre(Tile tile)
    {
        return { tile.x * kTileSize + kTileSize / 2, tile.y * kTileSize + kTileSize / 2 };
    }

    inline constexpr int32_t FloorDiv(int32_t value, int32_t divisor)
    {
        // divisor > 0; matches the game's floor2-style tile lookup for
        // negative coordinates as well.
        int32_t q = value / divisor;
        if ((value % divisor != 0) && ((value < 0) != (divisor < 0)))
            q--;
        return q;
    }

    inline constexpr Tile TileOfWorld(WorldPoint point)
    {
        return { FloorDiv(point.x, kTileSize), FloorDiv(point.y, kTileSize) };
    }

    // A tile is plaza-eligible iff it has a flat path (z >= 0) and belongs to
    // at least one 2x2 block of flat-path tiles all at the same z. This is
    // the design's section 2.1 rule, computed exactly.
    inline std::vector<char> ComputeEligible(const std::vector<int32_t>& flatPathZ, int32_t width, int32_t height)
    {
        std::vector<char> eligible(static_cast<size_t>(width) * static_cast<size_t>(height), 0);
        if (width < 2 || height < 2)
            return eligible;

        for (int32_t y = 0; y + 1 < height; y++)
        {
            for (int32_t x = 0; x + 1 < width; x++)
            {
                const int32_t z = flatPathZ[GridIndex(width, { x, y })];
                if (z < 0)
                    continue;
                if (flatPathZ[GridIndex(width, { x + 1, y })] != z || flatPathZ[GridIndex(width, { x, y + 1 })] != z
                    || flatPathZ[GridIndex(width, { x + 1, y + 1 })] != z)
                    continue;
                eligible[GridIndex(width, { x, y })] = 1;
                eligible[GridIndex(width, { x + 1, y })] = 1;
                eligible[GridIndex(width, { x, y + 1 })] = 1;
                eligible[GridIndex(width, { x + 1, y + 1 })] = 1;
            }
        }
        return eligible;
    }

    struct Components
    {
        std::vector<int32_t> id; // 0 = none, 1..count otherwise
        int32_t count{};
        std::vector<int32_t> sizes; // sizes[id - 1]
    };

    // Connected components (4-connected) over eligible tiles, seeded in
    // deterministic scan order (y, then x). Ids are 1-based here; the game
    // cache stores them directly (0 = none) in its uint16_t grid.
    inline Components LabelComponents(const std::vector<char>& eligible, int32_t width, int32_t height)
    {
        Components result;
        result.id.assign(static_cast<size_t>(width) * static_cast<size_t>(height), 0);
        std::vector<Tile> stack;
        for (int32_t y = 0; y < height; y++)
        {
            for (int32_t x = 0; x < width; x++)
            {
                const Tile seed{ x, y };
                const size_t seedIdx = static_cast<size_t>(GridIndex(width, seed));
                if (!eligible[seedIdx] || result.id[seedIdx] != 0)
                    continue;

                const int32_t newId = result.count + 1;
                int32_t size = 0;
                stack.clear();
                stack.push_back(seed);
                result.id[seedIdx] = newId;
                while (!stack.empty())
                {
                    const Tile cur = stack.back();
                    stack.pop_back();
                    size++;
                    for (int32_t dir = 0; dir < 4; dir++)
                    {
                        const Tile next = TileInDirection(cur, dir);
                        if (!InBounds(width, height, next))
                            continue;
                        const size_t idx = static_cast<size_t>(GridIndex(width, next));
                        if (!eligible[idx] || result.id[idx] != 0)
                            continue;
                        result.id[idx] = newId;
                        stack.push_back(next);
                    }
                }
                result.count = newId;
                result.sizes.push_back(size);
            }
        }
        return result;
    }

    struct Door
    {
        Tile inside{};
        Tile landing{};
        int32_t direction{}; // inside -> landing
    };

    // Door extraction (design section 2.2). For every region tile and every
    // direction: the crossing is a door iff the inside tile's permitted
    // edges include the direction AND the caller-supplied landingOk grid
    // says the landing tile is a stock-legal place to step onto (a path
    // element at the region's height whose edges include the reverse
    // direction, tested the way FootpathIsZAndDirectionValid tests it, or
    // a park entrance element). landingOk is indexed [tile * 4 + direction]
    // for the (inside, direction) pair. Landings that are themselves region
    // tiles (of any region) are not doors.
    inline std::vector<Door> ExtractDoors(
        const std::vector<int32_t>& componentId, int32_t width, int32_t height, const std::vector<int32_t>& permittedEdges,
        const std::vector<char>& landingOk)
    {
        std::vector<Door> doors;
        for (int32_t y = 0; y < height; y++)
        {
            for (int32_t x = 0; x < width; x++)
            {
                const Tile inside{ x, y };
                const size_t idx = static_cast<size_t>(GridIndex(width, inside));
                if (componentId[idx] == 0)
                    continue;
                for (int32_t dir = 0; dir < 4; dir++)
                {
                    if (!(permittedEdges[idx] & (1 << dir)))
                        continue;
                    const Tile landing = TileInDirection(inside, dir);
                    if (!InBounds(width, height, landing))
                        continue;
                    if (componentId[static_cast<size_t>(GridIndex(width, landing))] != 0)
                        continue;
                    if (!landingOk[idx * 4 + static_cast<size_t>(dir)])
                        continue;
                    doors.push_back({ inside, landing, dir });
                }
            }
        }
        return doors;
    }

    // Breadth-first shortest path over tiles of one component, 4-connected.
    // Returns the tile sequence including start and goal, or an empty
    // vector when goal is unreachable / not in the component.
    inline std::vector<Tile> BfsPath(
        const std::vector<int32_t>& componentId, int32_t width, int32_t height, int32_t regionId, Tile start, Tile goal)
    {
        std::vector<Tile> path;
        if (!InBounds(width, height, start) || !InBounds(width, height, goal))
            return path;
        if (componentId[static_cast<size_t>(GridIndex(width, start))] != regionId
            || componentId[static_cast<size_t>(GridIndex(width, goal))] != regionId)
            return path;
        if (start == goal)
        {
            path.push_back(start);
            return path;
        }

        const size_t total = static_cast<size_t>(width) * static_cast<size_t>(height);
        std::vector<int32_t> parent(total, -1);
        std::queue<Tile> queue;
        queue.push(start);
        parent[static_cast<size_t>(GridIndex(width, start))] = GridIndex(width, start);
        while (!queue.empty())
        {
            const Tile cur = queue.front();
            queue.pop();
            for (int32_t dir = 0; dir < 4; dir++)
            {
                const Tile next = TileInDirection(cur, dir);
                if (!InBounds(width, height, next))
                    continue;
                const size_t idx = static_cast<size_t>(GridIndex(width, next));
                if (componentId[idx] != regionId || parent[idx] != -1)
                    continue;
                parent[idx] = GridIndex(width, cur);
                if (next == goal)
                {
                    queue = std::queue<Tile>();
                    break;
                }
                queue.push(next);
            }
        }

        const size_t goalIdx = static_cast<size_t>(GridIndex(width, goal));
        if (parent[goalIdx] == -1)
            return path;

        int32_t cur = static_cast<int32_t>(goalIdx);
        const int32_t startIdx = GridIndex(width, start);
        while (cur != startIdx)
        {
            path.push_back({ cur % width, cur / width });
            cur = parent[static_cast<size_t>(cur)];
        }
        path.push_back(start);
        std::reverse(path.begin(), path.end());
        return path;
    }

    // Simulates the game's walker (Peep::updateWalkingAction) stepping from
    // `from` toward `to`: each tick it moves kWalkStep units along the
    // dominant axis (larger remaining delta; ties go to the x axis), and it
    // stops when the Manhattan distance is <= tolerance. Returns every tile
    // the walker occupies after each step, in order, with consecutive
    // duplicates removed. This is the ground truth for containment: a
    // steering segment is only safe if every tile returned here is a tile
    // the peep may legally be on (a region tile, or the final landing).
    inline std::vector<Tile> SimulateStaircase(WorldPoint from, WorldPoint to, int32_t tolerance)
    {
        std::vector<Tile> tiles;
        WorldPoint pos = from;
        // Upper bound on steps: Manhattan distance / step, plus slack.
        int32_t guard = (std::abs(to.x - from.x) + std::abs(to.y - from.y)) / kWalkStep + 4;
        while (guard-- > 0)
        {
            const int32_t dx = pos.x - to.x;
            const int32_t dy = pos.y - to.y;
            const int32_t xDelta = std::abs(dx);
            const int32_t yDelta = std::abs(dy);
            if (xDelta + yDelta <= tolerance)
                break;

            if (xDelta < yDelta)
            {
                pos.y += (dy >= 0) ? -kWalkStep : kWalkStep;
            }
            else
            {
                pos.x += (dx >= 0) ? -kWalkStep : kWalkStep;
            }

            const Tile tile = TileOfWorld(pos);
            if (tiles.empty() || tiles.back() != tile)
                tiles.push_back(tile);
        }
        return tiles;
    }

    // True iff walking the staircase from `from` to `to` never leaves the
    // set of allowed tiles: tiles of the given region, plus optionally one
    // extra tile (the final landing tile; pass {-1,-1} for none). The
    // starting tile itself must be allowed by the caller (it is the tile
    // the peep is standing on; only tiles entered after steps are checked,
    // and the first step may still be inside it).
    inline bool StaircaseStaysIn(
        const std::vector<int32_t>& componentId, int32_t width, int32_t height, int32_t regionId, WorldPoint from,
        WorldPoint to, Tile extraAllowed, int32_t tolerance)
    {
        const std::vector<Tile> tiles = SimulateStaircase(from, to, tolerance);
        for (const Tile& tile : tiles)
        {
            if (tile == extraAllowed)
                continue;
            if (!InBounds(width, height, tile))
                return false;
            if (componentId[static_cast<size_t>(GridIndex(width, tile))] != regionId)
                return false;
        }
        return true;
    }

    // String-pulling over a BFS tile path. Greedy: from the current anchor
    // point, take the farthest path tile whose centre can be reached by a
    // staircase that stays inside the region, make it the next waypoint and
    // the new anchor, and repeat until the goal tile is a waypoint. The
    // result always ends with the goal tile and is never longer (in count)
    // than the BFS path. Adjacent BFS steps are always pullable: their
    // staircase stays within the union of the two tiles (monotone in both
    // axes between neighbouring centres), and both tiles are in the region.
    inline std::vector<Tile> StringPull(
        const std::vector<int32_t>& componentId, int32_t width, int32_t height, int32_t regionId,
        const std::vector<Tile>& bfsPath, WorldPoint anchor)
    {
        std::vector<Tile> pulled;
        if (bfsPath.empty())
            return pulled;
        if (bfsPath.size() == 1)
        {
            pulled.push_back(bfsPath.front());
            return pulled;
        }

        size_t current = 0; // index into bfsPath of the tile the anchor belongs to / last waypoint
        WorldPoint anchorPoint = anchor;
        while (current + 1 < bfsPath.size())
        {
            size_t best = current + 1;
            for (size_t probe = bfsPath.size() - 1; probe > current + 1; probe--)
            {
                if (StaircaseStaysIn(
                        componentId, width, height, regionId, anchorPoint, TileCentre(bfsPath[probe]), { -1, -1 },
                        kWalkTolerance))
                {
                    best = probe;
                    break;
                }
            }
            pulled.push_back(bfsPath[best]);
            anchorPoint = TileCentre(bfsPath[best]);
            current = best;
        }
        return pulled;
    }

    // Heuristic score identical in shape to the game's
    // CalculateHeuristicPathingScore (tile units): used only to order doors
    // before the (authoritative) stock heuristic search scores them.
    inline int32_t HeuristicScoreTiles(Tile a, Tile b, int32_t zA, int32_t zB)
    {
        int32_t xDelta = std::abs(a.x - b.x) * 32;
        int32_t yDelta = std::abs(a.y - b.y) * 32;
        const int32_t zDelta = std::abs(zA - zB) * 2;
        if (xDelta < yDelta)
            xDelta >>= 4;
        else
            yDelta >>= 4;
        return xDelta + yDelta + zDelta;
    }
} // namespace OpenRCT2::PlazaGrid
