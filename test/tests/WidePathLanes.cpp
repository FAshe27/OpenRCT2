/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

// Wide path lanes (personal fork): tests for the lane code in GuestPathfinding.cpp ("Wide path lanes" region).
//
// Each test builds a layout of wide paths in the pathfinding test park, leading to the entrance of the "PathWithGap"
// ride at (1, 11), and walks guests through it one at a time. These tests can't show crowd effects: guests are stepped
// individually.

#include "TestData.h"

#include <algorithm>
#include <gtest/gtest.h>
#include <memory>
#include <openrct2/Context.h>
#include <openrct2/Game.h>
#include <openrct2/GameState.h>
#include <openrct2/OpenRCT2.h>
#include <openrct2/config/Config.h>
#include <openrct2/core/String.hpp>
#include <openrct2/entity/Guest.h>
#include <openrct2/peep/GuestPathfinding.h>
#include <openrct2/ride/RideManager.hpp>
#include <openrct2/scenario/Scenario.h>
#include <openrct2/world/Footpath.h>
#include <openrct2/world/Map.h>
#include <openrct2/world/TileElementsView.h>
#include <openrct2/world/tile_element/PathElement.h>
#include <openrct2/world/tile_element/WallElement.h>
#include <set>
#include <string>
#include <utility>
#include <vector>

using namespace OpenRCT2;

namespace
{
    constexpr int32_t kPathZ = 14;

    // The main test path: two lanes, rows y = 9 and y = 10, from x = 1 to x = 10. The ride entrance is at (1, 11),
    // reached from (1, 10).
    constexpr int32_t kStripFirstX = 1;
    constexpr int32_t kStripLastX = 10;
    constexpr TileCoordsXY kGoalTile{ 1, 10 };

    // The ride guests head for, and the path tile in front of its entrance. Most layouts use "PathWithGap", whose
    // entrance is at the end of the main test path.
    std::string gRideName = "PathWithGap";
    TileCoordsXY gGoalTile = kGoalTile;

    struct Rect
    {
        int32_t firstX, lastX, firstY, lastY;
    };

    struct WalkResult
    {
        int32_t arrived{};
        int32_t gaveUp{};
        int32_t countedAsLost{};
        int32_t tilesWalked{};
        std::vector<int32_t> tilesPerRow; // Rows 8, 9 and 10, along the strip (2 <= x <= 9).
        int32_t drawMismatches{};
        int32_t decisionsCompared{};
    };
} // namespace

class WidePathLanesTestBase : public testing::Test
{
protected:
    static void LoadPark(const std::vector<Rect>& rects)
    {
        gOpenRCT2Headless = true;
        gOpenRCT2NoGraphics = true;
        _context = CreateContext();
        ASSERT_TRUE(_context->Initialise());

        GetContext()->LoadParkFromFile(TestData::GetParkPath("pathfinding-tests.sv6"));
        GameLoadInit();

        // Distance fields are built at most two per game tick, and the tests don't run the game: give each layout a
        // tick of its own.
        static uint32_t layoutTick = 0;
        getGameState().currentTicks = 1000000 + (++layoutTick);

        _tiles.clear();
        for (const auto& rect : rects)
            for (int32_t x = rect.firstX; x <= rect.lastX; x++)
                for (int32_t y = rect.firstY; y <= rect.lastY; y++)
                    _tiles.insert({ x, y });
        BuildPaths();
    }

    static void UnloadPark()
    {
        gRideName = "PathWithGap";
        gGoalTile = kGoalTile;
        Config::Get().general.guestsUseWidePathLanes = true;
        _context = nullptr;
    }

    void SetUp() override
    {
        ScenarioRandSeed(0x12345678, 0x87654321);
    }

    // Adds path tiles on every tile in _tiles, each connected to its neighbours in _tiles.
    static void BuildPaths()
    {
        const auto* templatePath = MapGetPathElementAt({ 1, 10, kPathZ });
        ASSERT_NE(templatePath, nullptr);

        for (const auto& [x, y] : _tiles)
        {
            if (MapGetPathElementAt({ x, y, kPathZ }) != nullptr)
                continue;
            auto* pathElement = TileElementInsert<PathElement>(TileCoordsXYZ{ x, y, kPathZ }.toCoordsXYZ(), 0b1111);
            ASSERT_NE(pathElement, nullptr);
            pathElement->setClearanceZ(templatePath->getClearanceZ());
            if (templatePath->hasLegacyPathEntry())
            {
                pathElement->setLegacyPathEntryIndex(templatePath->getLegacyPathEntryIndex());
            }
            else
            {
                pathElement->setSurfaceEntryIndex(templatePath->getSurfaceEntryIndex());
                pathElement->setRailingsEntryIndex(templatePath->getRailingsEntryIndex());
            }
            pathElement->setRideIndex(RideId::GetNull());
            pathElement->setAdditionStatus(255);
        }
        for (const auto& [x, y] : _tiles)
        {
            auto* pathElement = MapGetPathElementAt({ x, y, kPathZ });
            uint8_t edges = pathElement->getEdges();
            for (Direction d : kAllDirections)
            {
                const auto neighbour = TileCoordsXY{ x, y } + TileDirectionDelta[d];
                if (_tiles.count({ neighbour.x, neighbour.y }))
                    edges |= (1 << d);
            }
            pathElement->setEdges(edges);
        }

        // The wide flags are normally updated a few tiles at a time as the game runs.
        for (int32_t i = 0; i < 1000; i++)
            MapUpdatePathWideFlags();
    }

    static Ride* FindRideByName(const char* name)
    {
        for (auto& ride : RideManager(getGameState()))
        {
            if (String::startsWith(ride.getName(), u8string{ name }, true))
                return &ride;
        }
        return nullptr;
    }

    // Number of ScenarioRand draws that take the generator from state `from` to state `to`.
    static int32_t CountDraws(const random_engine_t::state_type& from, const random_engine_t::state_type& to)
    {
        random_engine_t engine;
        Random::RCT2::Seed seed{ from.s0, from.s1 };
        engine.seed(seed);
        for (int32_t draws = 0; draws < 100000; draws++)
        {
            if (engine.state().s0 == to.s0 && engine.state().s1 == to.s1)
                return draws;
            engine();
        }
        return -1;
    }

    /**
     * If the guest is about to make a routing decision (they have reached their destination, so the next
     * performNextAction will call CalculateNextDestination), makes that decision with the lanes off and with them on,
     * from the same state, and compares their ScenarioRand draw counts. Restores the guest and the generator afterwards,
     * so the walk carries on unchanged. Returns false if the draw counts differ.
     */
    static bool CompareDecisionDraws(Guest& guest, bool& madeDecision)
    {
        madeDecision = false;
        const auto destination = guest.getDestination();
        const int32_t distance = std::abs(guest.x - destination.x) + std::abs(guest.y - destination.y);
        if (guest.state != PeepState::walking || !guest.isActionWalking() || distance > guest.destinationTolerance)
            return true;
        madeDecision = true;

        const Guest before = guest;
        const auto randBefore = ScenarioRandState();
        const bool lanes = Config::Get().general.guestsUseWidePathLanes;

        int32_t draws[2]{};
        for (const bool enabled : { false, true })
        {
            Config::Get().general.guestsUseWidePathLanes = enabled;
            PathFinding::CalculateNextDestination(guest);
            draws[enabled ? 1 : 0] = CountDraws(randBefore, ScenarioRandState());

            // Restore: move back first, so the entity's spatial index stays consistent, then everything else.
            guest.moveTo(before.getLocation());
            guest = before;
            ScenarioRandSeed(randBefore.s0, randBefore.s1);
        }
        Config::Get().general.guestsUseWidePathLanes = lanes;

        if (draws[0] != draws[1])
        {
            const auto tile = TileCoordsXY{ TileCoordsXYZ{ before.nextLoc } };
            std::printf(
                "Draw mismatch for guest %u at %d,%d heading %d: %d with the lanes off, %d with them on\n", before.peepId,
                tile.x, tile.y, before.peepDirection, draws[0], draws[1]);
        }
        return draws[0] == draws[1];
    }

    /**
     * Walks guests from start (heading in the given direction) until they reach the ride, give up, or run out of
     * steps. heading = true makes them head for the ride; otherwise they walk around aimlessly for maxSteps.
     * With compareDraws, every routing decision is also made with the lanes off and on, comparing their draws (see
     * CompareDecisionDraws).
     */
    static WalkResult WalkGuests(
        const TileCoordsXYZ& start, Direction startDirection, int32_t numGuests, bool lanes, bool headForRide,
        bool compareDraws = false, int32_t maxSteps = 8000)
    {
        WalkResult result{};
        result.tilesPerRow.resize(3);
        auto* ride = FindRideByName(gRideName.c_str());
        EXPECT_NE(ride, nullptr);
        if (ride == nullptr)
            return result;

        // Create all guests up front, so they all have different ids.
        std::vector<Guest*> guests;
        for (int32_t i = 0; i < numGuests; i++)
            guests.push_back(Guest::generate(start.toCoordsXYZ().toTileCentre()));

        for (auto* guest : guests)
        {
            guest->outsideOfPark = false;
            guest->state = PeepState::walking;
            guest->guestHeadingToRideId = headForRide ? ride->id : RideId::GetNull();
            guest->guestIsLostCountdown = 200;
            guest->peepDirection = startDirection;
            guest->nextLoc = start.toCoordsXYZ();
            guest->setNextFlags(0, false, false);
            guest->setDestination(CoordsXY{ guest->getLocation() } + CoordsDirectionDelta[startDirection], 2);

            Config::Get().general.guestsUseWidePathLanes = lanes;
            TileCoordsXY lastTile{ start };
            for (int32_t step = 0; step < maxSteps; step++)
            {
                if (compareDraws)
                {
                    bool madeDecision = false;
                    if (!CompareDecisionDraws(*guest, madeDecision))
                        result.drawMismatches++;
                    if (madeDecision)
                        result.decisionsCompared++;
                }
                guest->performNextAction();

                EXPECT_NE(MapGetFootpathElement(guest->nextLoc), nullptr) << "Guest left the path";
                const auto tile = TileCoordsXY{ TileCoordsXYZ{ guest->nextLoc } };
                if (tile != lastTile)
                {
                    lastTile = tile;
                    result.tilesWalked++;
                    if (getenv("LANE_TRACE"))
                        std::printf("%d,%d ", tile.x, tile.y);
                    if (tile.x > kStripFirstX && tile.x < kStripLastX && tile.y >= 8 && tile.y <= 10)
                        result.tilesPerRow[tile.y - 8]++;
                }
                if (headForRide && tile == gGoalTile)
                {
                    result.arrived++;
                    break;
                }
                if (headForRide && guest->guestHeadingToRideId.IsNull())
                {
                    result.gaveUp++;
                    break;
                }
            }
            if (getenv("LANE_TRACE"))
                std::printf("\n");
            if (guest->guestIsLostCountdown < 200)
                result.countedAsLost++;
        }
        for (auto* guest : guests)
            PeepEntityRemove(guest);
        return result;
    }

    static void Print(const char* name, const WalkResult& result)
    {
        std::printf(
            "%s: arrived %d, gave up %d, counted as lost %d, tiles walked %d, rows 8/9/10 = %d/%d/%d", name, result.arrived,
            result.gaveUp, result.countedAsLost, result.tilesWalked, result.tilesPerRow[0], result.tilesPerRow[1],
            result.tilesPerRow[2]);
        if (result.decisionsCompared > 0)
            std::printf(", decisions compared %d, draw mismatches %d", result.decisionsCompared, result.drawMismatches);
        std::printf("\n");
    }

    static std::set<std::pair<int32_t, int32_t>> _tiles;

private:
    static std::shared_ptr<IContext> _context;
};

std::set<std::pair<int32_t, int32_t>> WidePathLanesTestBase::_tiles;
std::shared_ptr<IContext> WidePathLanesTestBase::_context;

// --- Two lanes, with a side path off the far lane ----------------------------------------------------------------

// The strip, plus a side path off its north lane at x = 4, ending in a T junction at y = 6.
class WidePathLanesTest : public WidePathLanesTestBase
{
public:
    static void SetUpTestSuite()
    {
        LoadPark({ { kStripFirstX, kStripLastX, 9, 10 }, { 4, 4, 6, 8 }, { 3, 5, 6, 6 } });
    }

    static void TearDownTestSuite()
    {
        UnloadPark();
    }
};

TEST_F(WidePathLanesTest, GuestsUseOneLaneWhenDisabled)
{
    const auto result = WalkGuests({ kStripLastX, 10, kPathZ }, 0, 20, false, true);
    Print("Two lanes, disabled", result);
    EXPECT_EQ(result.arrived, 20);
    const auto total = result.tilesPerRow[1] + result.tilesPerRow[2];
    EXPECT_GT(std::max(result.tilesPerRow[1], result.tilesPerRow[2]) * 10, total * 8);
}

TEST_F(WidePathLanesTest, GuestsSpreadAcrossBothLanes)
{
    const auto result = WalkGuests({ kStripLastX, 10, kPathZ }, 0, 20, true, true);
    Print("Two lanes, enabled", result);
    EXPECT_EQ(result.arrived, 20);
    EXPECT_EQ(result.countedAsLost, 0);
    const auto total = result.tilesPerRow[1] + result.tilesPerRow[2];
    EXPECT_GT(result.tilesPerRow[1] * 10, total * 2) << "Expected guests to use both lanes";
    EXPECT_GT(result.tilesPerRow[2] * 10, total * 2) << "Expected guests to use both lanes";
}

TEST_F(WidePathLanesTest, AimlessGuestsKeepWalking)
{
    const auto result = WalkGuests({ 5, 9, kPathZ }, 0, 20, true, false, false, 3000);
    Print("Two lanes, aimless", result);
    EXPECT_GT(result.tilesWalked, 20 * 50) << "Expected guests to keep walking around rather than getting stuck";
}

TEST_F(WidePathLanesTest, DrawCountsMatchStockAlongTheCorridor)
{
    const auto result = WalkGuests({ kStripLastX, 10, kPathZ }, 0, 20, true, true, true);
    Print("Draws, corridor", result);
    EXPECT_GT(result.decisionsCompared, 150);
    EXPECT_EQ(result.drawMismatches, 0);
}

TEST_F(WidePathLanesTest, DrawCountsMatchStockForAimlessGuestsAndSidePaths)
{
    // Aimless guests in the north lane pass the side path at x = 4, so this covers the outer-side check too.
    const auto result = WalkGuests({ 8, 9, kPathZ }, 0, 20, true, false, true, 3000);
    Print("Draws, aimless and side path", result);
    // Measured: 3,494 decisions before the field-direct step (ce917080), 2,997 after it (ba5ec463), with 0 mismatches
    // both times. Field-directed guests take fewer, longer steps, so fewer decisions fit in the same walk.
    EXPECT_GT(result.decisionsCompared, 2500);
    EXPECT_EQ(result.drawMismatches, 0);
}

// --- Three lanes, whose outer lanes end in dead ends -------------------------------------------------------------

class WidePathThreeLanesTest : public WidePathLanesTestBase
{
public:
    static void SetUpTestSuite()
    {
        LoadPark({ { 1, 7, 8, 10 } });
    }

    static void TearDownTestSuite()
    {
        UnloadPark();
    }
};

TEST_F(WidePathThreeLanesTest, GuestsSpreadAcrossAllLanes)
{
    const auto result = WalkGuests({ 7, 10, kPathZ }, 0, 30, true, true);
    Print("Three lanes", result);
    EXPECT_EQ(result.arrived, 30);
    EXPECT_EQ(result.countedAsLost, 0);
    for (const auto tiles : result.tilesPerRow)
        EXPECT_GT(tiles, 0) << "Expected guests to use every lane";
}

TEST_F(WidePathThreeLanesTest, DrawCountsMatchStockAtDeadEnds)
{
    // Guests starting in the north lane, whose west end is a dead end.
    const auto result = WalkGuests({ 7, 8, kPathZ }, 0, 20, true, true, true);
    Print("Draws, three lanes and dead ends", result);
    EXPECT_GT(result.decisionsCompared, 100);
    EXPECT_EQ(result.drawMismatches, 0);
}

// --- Two two-lane paths crossing ---------------------------------------------------------------------------------

class WidePathCrossingTest : public WidePathLanesTestBase
{
public:
    static void SetUpTestSuite()
    {
        LoadPark({ { kStripFirstX, kStripLastX, 9, 10 }, { 5, 6, 6, 13 } });
    }

    static void TearDownTestSuite()
    {
        UnloadPark();
    }
};

TEST_F(WidePathCrossingTest, GuestsFindTheirWayThroughTheCrossing)
{
    const std::pair<TileCoordsXYZ, Direction> starts[] = {
        { { 10, 9, kPathZ }, 0 }, { { 10, 10, kPathZ }, 0 }, { { 5, 6, kPathZ }, 1 },
        { { 6, 6, kPathZ }, 1 },  { { 5, 13, kPathZ }, 3 },  { { 6, 13, kPathZ }, 3 },
    };
    for (const auto& [start, direction] : starts)
    {
        ScenarioRandSeed(0x12345678, 0x87654321);
        const auto stock = WalkGuests(start, direction, 10, false, true);
        ScenarioRandSeed(0x12345678, 0x87654321);
        const auto result = WalkGuests(start, direction, 10, true, true);
        const auto name = "Crossing from " + std::to_string(start.x) + "," + std::to_string(start.y);
        Print((name + " (disabled)").c_str(), stock);
        Print(name.c_str(), result);
        EXPECT_EQ(result.arrived, 10);
        EXPECT_EQ(result.countedAsLost, 0);
        EXPECT_LE(result.tilesWalked, stock.tilesWalked + 20) << "Expected about the same distance as stock";
    }
}

TEST_F(WidePathCrossingTest, DrawCountsMatchStockThroughTheCrossing)
{
    for (const auto& [start, direction] :
         { std::pair<TileCoordsXYZ, Direction>{ { 10, 9, kPathZ }, 0 }, { { 5, 6, kPathZ }, 1 }, { { 6, 13, kPathZ }, 3 } })
    {
        const auto result = WalkGuests(start, direction, 10, true, true, true);
        Print("Draws, crossing", result);
        EXPECT_GT(result.decisionsCompared, 60);
        EXPECT_EQ(result.drawMismatches, 0);
    }
}

// --- A corner ----------------------------------------------------------------------------------------------------

// The strip, joined at its east end by a two-lane path along x = 9 and x = 10 from y = 6.
class WidePathCornerTest : public WidePathLanesTestBase
{
public:
    static void SetUpTestSuite()
    {
        LoadPark({ { kStripFirstX, kStripLastX, 9, 10 }, { 9, 10, 6, 10 } });
    }

    static void TearDownTestSuite()
    {
        UnloadPark();
    }
};

TEST_F(WidePathCornerTest, GuestsKeepTheirLaneRoundCorners)
{
    for (const int32_t startX : { 9, 10 })
    {
        ScenarioRandSeed(0x12345678, 0x87654321);
        const auto stock = WalkGuests({ startX, 6, kPathZ }, 1, 20, false, true);
        ScenarioRandSeed(0x12345678, 0x87654321);
        const auto result = WalkGuests({ startX, 6, kPathZ }, 1, 20, true, true);
        Print("Corner (disabled)", stock);
        Print("Corner", result);
        EXPECT_EQ(result.arrived, 20);
        EXPECT_EQ(result.countedAsLost, 0);
        EXPECT_LE(result.tilesWalked, stock.tilesWalked + 40) << "Expected about the same distance as stock";
    }
}

TEST_F(WidePathCornerTest, DrawCountsMatchStockRoundCorners)
{
    for (const int32_t startX : { 9, 10 })
    {
        const auto result = WalkGuests({ startX, 6, kPathZ }, 1, 20, true, true, true);
        Print("Draws, corner", result);
        EXPECT_GT(result.decisionsCompared, 200);
        EXPECT_EQ(result.drawMismatches, 0);
    }
}

// --- A ride entrance on the side of a two-lane path --------------------------------------------------------------

// Two lanes, rows y = 9 and y = 10, from x = 7 to x = 15, passing the entrance of "PathWithFences" at (11, 11), which
// joins the south lane at (11, 10) from the side. The fences around the existing path at x = 11 are removed.
class WidePathSideEntranceTest : public WidePathLanesTestBase
{
public:
    static void SetUpTestSuite()
    {
        LoadPark({ { 7, 15, 9, 10 } });
        for (const int32_t y : { 9, 10 })
        {
            const auto loc = TileCoordsXY{ 11, y }.toCoordsXY();
            for (auto* wall : TileElementsView<WallElement>(loc))
                wall->setGhost(true);
        }
        gRideName = "PathWithFences";
        gGoalTile = { 11, 10 };
    }

    static void TearDownTestSuite()
    {
        UnloadPark();
    }
};

TEST_F(WidePathSideEntranceTest, GuestsFindAnEntranceOnTheSideOfThePath)
{
    auto* ride = FindRideByName("PathWithFences");
    ASSERT_NE(ride, nullptr);
    std::printf(
        "PathWithFences status %d, entrance %d,%d dir %d\n", static_cast<int>(ride->status), ride->getStation().entrance.x,
        ride->getStation().entrance.y, ride->getStation().entrance.direction);

    const std::pair<TileCoordsXYZ, Direction> starts[] = {
        { { 15, 9, kPathZ }, 0 },
        { { 15, 10, kPathZ }, 0 },
        { { 7, 9, kPathZ }, 2 },
        { { 7, 10, kPathZ }, 2 },
        // Starting out walking away from the entrance.
        { { 12, 9, kPathZ }, 2 },
        { { 12, 10, kPathZ }, 2 },
        { { 10, 9, kPathZ }, 0 },
        { { 10, 10, kPathZ }, 0 },
    };
    for (const auto& [start, direction] : starts)
    {
        ScenarioRandSeed(0x12345678, 0x87654321);
        const auto stock = WalkGuests(start, direction, 10, false, true);
        ScenarioRandSeed(0x12345678, 0x87654321);
        const auto result = WalkGuests(start, direction, 10, true, true);
        const auto name = "Side entrance from " + std::to_string(start.x) + "," + std::to_string(start.y);
        Print((name + " (disabled)").c_str(), stock);
        Print(name.c_str(), result);
        EXPECT_EQ(result.arrived, 10);
        // Guests that start out walking away from the entrance may reach a dead end and turn around, which stock also
        // counts towards getting lost, so this only requires no worse than stock. Measured at ba5ec463: 0 counted as lost
        // from every start with the lanes on (stock: up to 10 of 10).
        EXPECT_LE(result.countedAsLost, stock.countedAsLost);
        EXPECT_LE(result.tilesWalked, stock.tilesWalked + 20) << "Expected about the same distance as stock";
    }
}

// --- A plaza -----------------------------------------------------------------------------------------------------

// The strip, with a plaza (x = 2..7, y = 6..8) joined along its north side, making one open area of paths from y = 6
// to y = 10 between x = 2 and x = 7.
class WidePathPlazaTest : public WidePathLanesTestBase
{
public:
    static void SetUpTestSuite()
    {
        LoadPark({ { kStripFirstX, kStripLastX, 9, 10 }, { 2, 7, 6, 8 } });
    }

    static void TearDownTestSuite()
    {
        UnloadPark();
    }
};

TEST_F(WidePathPlazaTest, GuestsCrossThePlaza)
{
    const std::pair<TileCoordsXYZ, Direction> starts[] = {
        { { 7, 6, kPathZ }, 0 },
        { { 7, 6, kPathZ }, 1 },
        { { 2, 6, kPathZ }, 1 },
        { { 10, 9, kPathZ }, 0 },
    };
    for (const auto& [start, direction] : starts)
    {
        ScenarioRandSeed(0x12345678, 0x87654321);
        const auto stock = WalkGuests(start, direction, 10, false, true);
        ScenarioRandSeed(0x12345678, 0x87654321);
        const auto result = WalkGuests(start, direction, 10, true, true);
        const auto name = "Plaza from " + std::to_string(start.x) + "," + std::to_string(start.y);
        Print((name + " (disabled)").c_str(), stock);
        Print(name.c_str(), result);
        EXPECT_EQ(result.arrived, 10);
        EXPECT_EQ(result.countedAsLost, 0);
        EXPECT_LE(result.tilesWalked, std::max(stock.tilesWalked, 10 * 12) + 20);
    }
}

TEST_F(WidePathPlazaTest, AimlessGuestsKeepWalking)
{
    const auto result = WalkGuests({ 4, 7, kPathZ }, 0, 20, true, false, false, 3000);
    Print("Plaza, aimless", result);
    EXPECT_GT(result.tilesWalked, 20 * 50) << "Expected guests to keep walking around rather than getting stuck";
}

TEST_F(WidePathPlazaTest, DrawCountsMatchStockOnThePlaza)
{
    const auto heading = WalkGuests({ 7, 6, kPathZ }, 0, 10, true, true, true);
    const auto aimless = WalkGuests({ 4, 7, kPathZ }, 0, 10, true, false, true, 2000);
    Print("Draws, plaza", heading);
    Print("Draws, plaza aimless", aimless);
    EXPECT_GT(heading.decisionsCompared + aimless.decisionsCompared, 500);
    EXPECT_EQ(heading.drawMismatches + aimless.drawMismatches, 0);
}
