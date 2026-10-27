#include "TestData.h"

#include <gtest/gtest.h>
#include <memory>
#include <openrct2/Context.h>
#include <openrct2/Game.h>
#include <openrct2/GameState.h>
#include <openrct2/OpenRCT2.h>
#include <openrct2/actions/GameActionResult.h>
#include <openrct2/actions/GameActionRunner.h>
#include <openrct2/actions/footpath/FootpathPlaceAction.h>
#include <openrct2/actions/ride/RideCreateAction.h>
#include <openrct2/actions/ride/RideEntranceExitPlaceAction.h>
#include <openrct2/core/String.hpp>
#include <openrct2/entity/Guest.h>
#include <openrct2/peep/GuestPathfinding.h>
#include <openrct2/peep/PlazaRegions.h>
#include <openrct2/platform/Platform.h>
#include <openrct2/ride/RideManager.hpp>
#include <openrct2/scenario/Scenario.h>
#include <openrct2/world/Footpath.h>
#include <openrct2/world/Map.h>
#include <openrct2/world/tile_element/PathElement.h>
#include <openrct2/world/tile_element/SurfaceElement.h>
#include <set>
#include <string>

using namespace OpenRCT2;

static std::ostream& operator<<(std::ostream& os, const TileCoordsXYZ& coords)
{
    return os << "(" << coords.x << ", " << coords.y << ", " << coords.z << ")";
}

class PathfindingTestBase : public testing::Test
{
public:
    static void SetUpTestCase()
    {
        gOpenRCT2Headless = true;
        gOpenRCT2NoGraphics = true;
        _context = CreateContext();
        const bool initialised = _context->Initialise();
        ASSERT_TRUE(initialised);

        std::string parkPath = TestData::GetParkPath("pathfinding-tests.sv6");
        GetContext()->LoadParkFromFile(parkPath);
        GameLoadInit(); // NB: calls `setActiveScene`
    }

    void SetUp() override
    {
        // Use a consistent random seed in every test
        ScenarioRandSeed(0x12345678, 0x87654321);
    }

    static void TearDownTestCase()
    {
        _context = nullptr;
    }

protected:
    static Ride* FindRideByName(const char* name)
    {
        auto& gameState = getGameState();
        for (auto& ride : RideManager(gameState))
        {
            auto thisName = ride.getName();
            if (String::startsWith(thisName, u8string{ name }, true))
            {
                return &ride;
            }
        }
        return nullptr;
    }

    static bool FindPath(TileCoordsXYZ* pos, const TileCoordsXYZ& goal, int expectedSteps, RideId targetRideID)
    {
        // Our start position is in tile coordinates, but we need to give the peep spawn
        // position in actual world coords (32 units per tile X/Y, 8 per Z level).
        // Add 16 so the peep spawns in the centre of the tile.
        auto* peep = Guest::generate(pos->toCoordsXYZ().toTileCentre());

        // Peeps that are outside of the park use specialized pathfinding which we don't want to
        // use here
        peep->outsideOfPark = false;

        // An earlier iteration of this code just gave peeps a target position to walk to, but it turns out
        // that with no actual ride to head towards, when a peep reaches a junction they use the 'aimless'
        // pathfinder instead of pursuing their original pathfinding target. So, we always need to give them
        // an actual ride to walk to the entrance of.
        peep->guestHeadingToRideId = targetRideID;

        // Pick the direction the peep should initially move in, given the goal position.
        // This will also store the goal position and initialize pathfinding data for the peep.
        const Direction moveDir = PathFinding::ChooseDirection(*pos, goal, *peep, false, RideId::GetNull());
        if (moveDir == kInvalidDirection)
        {
            // Couldn't determine a direction to move off in
            return false;
        }

        // We have already set up the peep's overall pathfinding goal, but we also have to set their initial
        // 'destination' which is a close position that they will walk towards in a straight line - in this case, one
        // tile away. Stepping the peep will move them towards their destination, and once they reach it, a new
        // destination will be picked, to try and get the peep towards the overall pathfinding goal.
        peep->peepDirection = moveDir;
        auto destination = CoordsDirectionDelta[moveDir] + peep->getLocation();
        peep->setDestination(destination, 2);

        // Repeatedly step the peep, until they reach the target position or until the expected number of steps have
        // elapsed. Each step, check that the tile they are standing on is not marked as forbidden in the test data
        // (red neon ground type).
        int step = 0;
        while (*pos != goal && step < expectedSteps)
        {
            peep->performNextAction();
            ++step;

            *pos = TileCoordsXYZ(peep->getLocation());

            EXPECT_PRED_FORMAT1(AssertIsNotForbiddenPosition, *pos);

            // Check that the peep is still on a footpath. Use next_z instead of pos->z here because pos->z will change
            // when the peep is halfway up a slope, but next_z will not change until they move to the next tile.
            EXPECT_NE(MapGetFootpathElement({ pos->toCoordsXY(), peep->nextLoc.z }), nullptr);
        }

        // Clean up the peep, because we're reusing this loaded context for all tests.
        PeepEntityRemove(peep);

        // Require that the number of steps taken is exactly what we expected. The pathfinder is supposed to be
        // deterministic, and we reset the RNG seed for each test, everything should be entirely repeatable; as
        // such a change in the number of steps taken on one of these paths needs to be reviewed. For the negative
        // tests, we will not have reached the goal but we still expect the loop to have run for the total number
        // of steps requested before giving up.
        EXPECT_EQ(step, expectedSteps);

        return *pos == goal;
    }

    static testing::AssertionResult AssertIsStartPosition(const char*, const TileCoordsXYZ& location)
    {
        const auto expectedSurfaceColour = EnumValue(Drawing::Colour::brightGreen);
        const auto colour = EnumValue(MapGetSurfaceElementAt(location.toCoordsXYZ())->getPrimarySurfaceColour());

        if (colour != expectedSurfaceColour)
            return testing::AssertionFailure()
                << "Start location " << location << " should have surface colour " << expectedSurfaceColour
                << " but actually has style " << colour
                << ". Either the test map is not set up correctly, or you got the coordinates wrong.";

        return testing::AssertionSuccess();
    }

    static testing::AssertionResult AssertIsNotForbiddenPosition(const char*, const TileCoordsXYZ& location)
    {
        const uint32_t forbiddenSurfaceColour = EnumValue(Drawing::Colour::brightRed);
        const auto colour = EnumValue(MapGetSurfaceElementAt(location.toCoordsXYZ())->getPrimarySurfaceColour());

        if (colour == forbiddenSurfaceColour)
            return testing::AssertionFailure()
                << "Path traversed location " << location << ", but it is marked as a forbidden location (surface colour "
                << forbiddenSurfaceColour << "). Either the map is set up incorrectly, or the pathfinder went the wrong way.";

        return testing::AssertionSuccess();
    }

private:
    static std::shared_ptr<IContext> _context;
};

std::shared_ptr<IContext> PathfindingTestBase::_context;

struct SimplePathfindingScenario
{
    const char* name;
    TileCoordsXYZ start;
    uint32_t steps;

    SimplePathfindingScenario(const char* _name, const TileCoordsXYZ& _start, int _steps)
        : name(_name)
        , start(_start)
        , steps(_steps)
    {
    }

    static std::string ToName(const testing::TestParamInfo<SimplePathfindingScenario>& param_info)
    {
        return param_info.param.name;
    }
};

class SimplePathfindingTest : public PathfindingTestBase, public testing::WithParamInterface<SimplePathfindingScenario>
{
};

TEST_P(SimplePathfindingTest, CanFindPathFromStartToGoal)
{
    const SimplePathfindingScenario& scenario = GetParam();

    ASSERT_PRED_FORMAT1(AssertIsStartPosition, scenario.start);
    TileCoordsXYZ pos = scenario.start;

    auto ride = FindRideByName(scenario.name);
    ASSERT_NE(ride, nullptr);

    auto entrancePos = ride->getStation().entrance;
    TileCoordsXYZ goal = TileCoordsXYZ(
        entrancePos.x - TileDirectionDelta[entrancePos.direction].x,
        entrancePos.y - TileDirectionDelta[entrancePos.direction].y, entrancePos.z);

    const auto succeeded = FindPath(&pos, goal, scenario.steps, ride->id) ? testing::AssertionSuccess()
                                                                          : testing::AssertionFailure()
            << "Failed to find path from " << scenario.start << " to " << goal << " in " << scenario.steps << " steps; reached "
            << pos << " before giving up.";

    EXPECT_TRUE(succeeded);
}

INSTANTIATE_TEST_SUITE_P(
    ForScenario, SimplePathfindingTest,
    ::testing::Values(
        SimplePathfindingScenario("StraightFlat", { 19, 15, 14 }, 24), SimplePathfindingScenario("SBend", { 15, 12, 14 }, 87),
        SimplePathfindingScenario("UBend", { 17, 9, 14 }, 87), SimplePathfindingScenario("CBend", { 14, 5, 14 }, 164),
        SimplePathfindingScenario("TwoEqualRoutes", { 9, 13, 14 }, 89),
        SimplePathfindingScenario("TwoUnequalRoutes", { 3, 13, 14 }, 89),
        SimplePathfindingScenario("StraightUpBridge", { 12, 15, 14 }, 24),
        SimplePathfindingScenario("StraightUpSlope", { 14, 15, 14 }, 24),
        SimplePathfindingScenario("SelfCrossingPath", { 6, 5, 14 }, 211)),
    SimplePathfindingScenario::ToName);

class ImpossiblePathfindingTest : public PathfindingTestBase, public testing::WithParamInterface<SimplePathfindingScenario>
{
};

TEST_P(ImpossiblePathfindingTest, CannotFindPathFromStartToGoal)
{
    const SimplePathfindingScenario& scenario = GetParam();
    TileCoordsXYZ pos = scenario.start;
    ASSERT_PRED_FORMAT1(AssertIsStartPosition, scenario.start);

    auto ride = FindRideByName(scenario.name);
    ASSERT_NE(ride, nullptr);

    auto entrancePos = ride->getStation().entrance;
    TileCoordsXYZ goal = TileCoordsXYZ(
        entrancePos.x + TileDirectionDelta[entrancePos.direction].x,
        entrancePos.y + TileDirectionDelta[entrancePos.direction].y, entrancePos.z);

    EXPECT_FALSE(FindPath(&pos, goal, 10000, ride->id));
}

INSTANTIATE_TEST_SUITE_P(
    ForScenario, ImpossiblePathfindingTest,
    ::testing::Values(
        SimplePathfindingScenario("PathWithGap", { 1, 6, 14 }, 10000),
        SimplePathfindingScenario("PathWithFences", { 11, 6, 14 }, 10000),
        SimplePathfindingScenario("PathWithCliff", { 7, 17, 14 }, 10000)),
    SimplePathfindingScenario::ToName);

// ---------------------------------------------------------------------------
// V2 plaza walking tests (personal fork).
//
// PlazaRegionCacheTest runs against the EXISTING pathfinding-tests.sv6
// fixture and needs no new park data: it only checks cache invariants
// (builds, deterministic across a rebuild, self-consistent lookups).
//
// PlazaCrossingTest was originally specified against a hand-extended
// .sv6 fixture. That is unnecessary - and the prescribed coordinates
// (x=25..34) do not even exist: pathfinding-tests.sv6 is only 22x22
// tiles. Instead the scenario is CONSTRUCTED IN CODE on top of
// BigMapTest.sv6 (the big flat ratings park that RideRatings loads in
// this same test binary). BigMapTest has a verified-empty flat area at
// z=14 around x=151..162, y=62..69 (surface-only tiles, no water,
// park-owned, nothing within 25+ tiles in any direction). In
// SetUpTestCase the fixture builds the scenario through the ordinary
// game actions:
//   * an 8x6 plaza of flat footpath at x=151..158, y=62..67, z=14 -
//     every tile belongs to a 2x2 block of such tiles, so the plaza is
//     a plaza region by the V2 definition;
//   * a two-tile stub at (159,64) and (159,65), east of the plaza. The
//     stub shares 2x2 blocks with the plaza's east edge, so the goal
//     tile (159,64) is part of the same region (goal-in-region
//     steering, design M3), and the region's only way out is the ride
//     entrance itself;
//   * a ride named "PlazaCrossing" (RideCreateAction, copying type /
//     subtype / entrance style / inspection interval from an existing
//     BigMapTest ride so every object is guaranteed loaded) whose
//     entrance element is placed at (160,64) with entrance direction 2
//     (guests enter walking east), so the tile in front of the entrance
//     - the goal, computed below exactly like SimplePathfindingTest
//     computes it - is the stub tile (159,64).
// The start tile (151,64), on the plaza's west edge, is painted bright
// green and the ring of non-path tiles around plaza + stub bright red,
// following this file's start/forbidden conventions.
// Unlike SimplePathfindingTest this test does NOT demand an exact step
// count: plaza steering replaces many one-tile decisions with a few
// long strides, and the count legitimately moves when steering tuning
// constants change. What is asserted: the guest stays on footpath
// tiles, never touches a forbidden tile, crosses the plaza's INTERIOR
// (not just its boundary ring), and arrives at the goal within a
// generous step budget. On failure the message carries the step count,
// the end position and the full tile route the guest took.
// ---------------------------------------------------------------------------

class PlazaRegionCacheTest : public PathfindingTestBase
{
};

TEST_F(PlazaRegionCacheTest, CacheIsDeterministicAcrossRebuilds)
{
    ASSERT_TRUE(Plaza::EnsurePlazaRegions());

    struct Snapshot
    {
        uint16_t id;
        int32_t baseZ;
        size_t tiles;
        size_t doors;
    };
    std::vector<Snapshot> first;
    for (const auto& region : Plaza::AllRegions())
    {
        first.push_back({ region.id, region.baseZ, region.tiles.size(), region.doors.size() });

        // Every tile the cache claims must look up as that region at the
        // region's height, and as no region at a different height.
        for (const auto& tile : region.tiles)
        {
            EXPECT_EQ(Plaza::GetRegionIdAt({ tile.x, tile.y, region.baseZ }), region.id);
            EXPECT_EQ(Plaza::GetRegionIdAt({ tile.x, tile.y, region.baseZ + 2 }), 0);
        }
    }

    Plaza::InvalidatePlazaRegions();
    ASSERT_TRUE(Plaza::PlazaRegionsDirty());
    ASSERT_TRUE(Plaza::EnsurePlazaRegions());

    const auto& regions = Plaza::AllRegions();
    ASSERT_EQ(regions.size(), first.size());
    for (size_t i = 0; i < first.size(); i++)
    {
        EXPECT_EQ(regions[i].id, first[i].id);
        EXPECT_EQ(regions[i].baseZ, first[i].baseZ);
        EXPECT_EQ(regions[i].tiles.size(), first[i].tiles);
        EXPECT_EQ(regions[i].doors.size(), first[i].doors);
    }
}

class PlazaCrossingTest : public PathfindingTestBase
{
public:
    static void SetUpTestCase()
    {
        // NB: PathfindingTestBase::_context is private, so this fixture
        // keeps its own context for the BigMapTest park.
        gOpenRCT2Headless = true;
        gOpenRCT2NoGraphics = true;
        _plazaContext = CreateContext();
        const bool initialised = _plazaContext->Initialise();
        ASSERT_TRUE(initialised);

        std::string parkPath = TestData::GetParkPath("BigMapTest.sv6");
        GetContext()->LoadParkFromFile(parkPath);
        GameLoadInit(); // NB: calls `setActiveScene`

        BuildPlazaScenario();
    }

    static void TearDownTestCase()
    {
        _plazaContext = nullptr;
    }

protected:
    // Scenario geometry (tile coordinates; z in path baseHeight units).
    static constexpr int32_t kPlazaX = 151;
    static constexpr int32_t kPlazaY = 62;
    static constexpr int32_t kPlazaZ = 14;
    static constexpr int32_t kPlazaWidth = 8;                                            // plaza tiles x: 151..158
    static constexpr int32_t kPlazaHeight = 6;                                           // plaza tiles y: 62..67
    static constexpr TileCoordsXYZ kStart{ kPlazaX, kPlazaY + 2, kPlazaZ };              // (151, 64)
    static constexpr TileCoordsXYZ kGoal{ kPlazaX + kPlazaWidth, kPlazaY + 2, kPlazaZ }; // stub tile (159, 64)
    static constexpr int32_t kEntranceX = kPlazaX + kPlazaWidth + 1;                     // 160
    static constexpr int32_t kEntranceY = kPlazaY + 2;                                   // 64
    static constexpr Direction kEntranceDirection = 2;                                   // guests enter walking east (+x)

    static bool IsScenarioPathTile(const TileCoordsXYZ& tile)
    {
        if (tile.z != kPlazaZ)
        {
            return false;
        }
        const bool inPlaza = tile.x >= kPlazaX && tile.x < kPlazaX + kPlazaWidth && tile.y >= kPlazaY
            && tile.y < kPlazaY + kPlazaHeight;
        const bool inStub = tile.x == kGoal.x && (tile.y == kGoal.y || tile.y == kGoal.y + 1);
        return inPlaza || inStub;
    }

    static bool IsPlazaInterior(const TileCoordsXYZ& tile)
    {
        // Plaza tiles inset by one tile from the plaza boundary ring. The
        // stub and the entrance tile are never interior.
        return tile.z == kPlazaZ && tile.x > kPlazaX && tile.x < kPlazaX + kPlazaWidth - 1 && tile.y > kPlazaY
            && tile.y < kPlazaY + kPlazaHeight - 1;
    }

    static bool FindDonorPathIndices(ObjectEntryIndex* outSurface, ObjectEntryIndex* outRailings)
    {
        const auto mapSize = getGameState().mapSize;
        for (int32_t y = 0; y < mapSize.y; y++)
        {
            for (int32_t x = 0; x < mapSize.x; x++)
            {
                TileElement* element = MapGetFirstElementAt(TileCoordsXY{ x, y });
                if (element == nullptr)
                {
                    continue;
                }
                do
                {
                    if (element->getType() == TileElementType::path)
                    {
                        auto* path = element->asPath();
                        if (!path->isQueue())
                        {
                            *outSurface = path->getSurfaceEntryIndex();
                            *outRailings = path->getRailingsEntryIndex();
                            return true;
                        }
                    }
                } while (!(element++)->isLastForTile());
            }
        }
        return false;
    }

    static void PlacePath(const TileCoordsXYZ& tile, ObjectEntryIndex pathSurface, ObjectEntryIndex pathRailings)
    {
        GameActions::FootpathPlaceAction action(
            tile.toCoordsXYZ(), FootpathSlope{ FootpathSlopeType::flat, kInvalidDirection }, pathSurface, pathRailings);
        const auto result = GameActions::Execute(&action, getGameState());
        ASSERT_EQ(result.error, GameActions::Status::ok)
            << "FootpathPlaceAction failed at " << tile << " with status " << static_cast<int32_t>(result.error);
    }

    static void BuildPlazaScenario()
    {
        auto& gameState = getGameState();

        // Construction costs money; make sure the park can pay for it.
        gameState.park.cash = 10000000;

        // Make sure every tile we build on is park-owned (BigMapTest's
        // already are; a safeguard no-op, not a workaround).
        for (int32_t x = kPlazaX; x <= kEntranceX; x++)
        {
            for (int32_t y = kPlazaY; y < kPlazaY + kPlazaHeight; y++)
            {
                auto* surface = MapGetSurfaceElementAt(TileCoordsXYZ{ x, y, kPlazaZ }.toCoordsXYZ());
                ASSERT_NE(surface, nullptr) << "No surface element at tile " << x << ", " << y;
                if (!surface->hasOwnership(OwnershipFlag::landOwned))
                {
                    surface->setOwnership(OwnershipFlag::landOwned);
                }
            }
        }

        // Borrow the footpath surface/railings object indices from an
        // existing non-queue path so the objects are guaranteed loaded.
        ObjectEntryIndex pathSurface = kObjectEntryIndexNull;
        ObjectEntryIndex pathRailings = kObjectEntryIndexNull;
        ASSERT_TRUE(FindDonorPathIndices(&pathSurface, &pathRailings))
            << "BigMapTest.sv6 contains no non-queue footpath to copy path object indices from";

        // Borrow ride type/subtype/entrance style/inspection interval
        // from an existing ride for the same reason.
        Ride* donor = nullptr;
        for (auto& ride : RideManager(gameState))
        {
            donor = &ride;
            break;
        }
        ASSERT_NE(donor, nullptr) << "BigMapTest.sv6 contains no rides";
        const auto donorType = donor->type;
        const auto donorSubtype = donor->subtype;
        const auto donorEntranceStyle = donor->entranceStyle;
        const auto donorInspection = donor->inspectionInterval;

        // Remember the existing ride ids so the new ride can be found
        // afterwards (RideCreateAction does not hand it back directly).
        std::set<uint32_t> existingRideIds;
        for (auto& ride : RideManager(gameState))
        {
            existingRideIds.insert(ride.id.ToUnderlying());
        }

        GameActions::RideCreateAction createAction(donorType, donorSubtype, 0, 0, donorEntranceStyle, donorInspection);
        auto result = GameActions::Execute(&createAction, gameState);
        ASSERT_EQ(result.error, GameActions::Status::ok)
            << "RideCreateAction failed with status " << static_cast<int32_t>(result.error);

        Ride* ride = nullptr;
        for (auto& candidate : RideManager(gameState))
        {
            if (existingRideIds.count(candidate.id.ToUnderlying()) == 0)
            {
                ride = &candidate;
                break;
            }
        }
        ASSERT_NE(ride, nullptr) << "RideCreateAction reported success but no new ride appeared";
        ride->customName = "PlazaCrossing";

        // Lay the plaza and the stub. FootpathPlaceAction connects the
        // edges between neighbouring path tiles (and, below, the ride
        // entrance) exactly as in-game placement does.
        for (int32_t x = kPlazaX; x < kPlazaX + kPlazaWidth; x++)
        {
            for (int32_t y = kPlazaY; y < kPlazaY + kPlazaHeight; y++)
            {
                PlacePath(TileCoordsXYZ{ x, y, kPlazaZ }, pathSurface, pathRailings);
            }
        }
        PlacePath(kGoal, pathSurface, pathRailings);
        PlacePath(TileCoordsXYZ{ kGoal.x, kGoal.y + 1, kPlazaZ }, pathSurface, pathRailings);

        // Place the ride entrance east of the stub. The action derives
        // its height from the station record, so set the station base
        // height first (there is no track; the station record is only a
        // data anchor here). The action also fills in station.entrance
        // and connects the stub's east edge to the entrance.
        auto& station = ride->getStation();
        station.setBaseZ(kStart.toCoordsXYZ().z);
        GameActions::RideEntranceExitPlaceAction entranceAction(
            TileCoordsXY{ kEntranceX, kEntranceY }.toCoordsXY(), kEntranceDirection, ride->id, StationIndex::FromUnderlying(0),
            false);
        result = GameActions::Execute(&entranceAction, gameState);
        ASSERT_EQ(result.error, GameActions::Status::ok)
            << "RideEntranceExitPlaceAction failed with status " << static_cast<int32_t>(result.error);

        const auto entrancePos = ride->getStation().entrance;
        ASSERT_EQ(entrancePos.x, kEntranceX);
        ASSERT_EQ(entrancePos.y, kEntranceY);
        ASSERT_EQ(entrancePos.z, kPlazaZ);
        ASSERT_EQ(entrancePos.direction, kEntranceDirection);

        ride->status = RideStatus::open;

        // Paint this file's start/forbidden conventions.
        MapGetSurfaceElementAt(kStart.toCoordsXYZ())->setPrimarySurfaceColour(Drawing::Colour::brightGreen);
        for (int32_t x = kPlazaX - 1; x <= kEntranceX + 1; x++)
        {
            for (int32_t y = kPlazaY - 1; y <= kPlazaY + kPlazaHeight; y++)
            {
                const TileCoordsXYZ tile{ x, y, kPlazaZ };
                if (IsScenarioPathTile(tile))
                {
                    continue;
                }
                MapGetSurfaceElementAt(tile.toCoordsXYZ())->setPrimarySurfaceColour(Drawing::Colour::brightRed);
            }
        }

        // Structural sanity: plaza + stub must read back as ONE plaza
        // region containing both the start and the goal tile.
        ASSERT_TRUE(Plaza::EnsurePlazaRegions());
        const auto startRegion = Plaza::GetRegionIdAt(kStart);
        ASSERT_NE(startRegion, 0) << "Start tile " << kStart << " is not in any plaza region";
        ASSERT_EQ(Plaza::GetRegionIdAt(kGoal), startRegion)
            << "Goal tile " << kGoal << " is not in the start tile's plaza region";
        const auto* region = Plaza::FindRegion(startRegion);
        ASSERT_NE(region, nullptr);
        EXPECT_GE(region->tiles.size(), static_cast<size_t>(kPlazaWidth * kPlazaHeight))
            << "Plaza region has only " << region->tiles.size() << " tiles";
    }

private:
    static std::shared_ptr<IContext> _plazaContext;
};

std::shared_ptr<IContext> PlazaCrossingTest::_plazaContext;

TEST_F(PlazaCrossingTest, GuestCrossesPlazaToRideEntrance)
{
    ASSERT_PRED_FORMAT1(AssertIsStartPosition, kStart);

    auto* ride = FindRideByName("PlazaCrossing");
    ASSERT_NE(ride, nullptr) << "No ride named PlazaCrossing - the programmatic fixture setup must have failed.";

    auto entrancePos = ride->getStation().entrance;
    const TileCoordsXYZ goal = TileCoordsXYZ(
        entrancePos.x - TileDirectionDelta[entrancePos.direction].x,
        entrancePos.y - TileDirectionDelta[entrancePos.direction].y, entrancePos.z);
    ASSERT_EQ(goal, kGoal) << "Goal computed from the ride entrance is " << goal << ", expected the stub tile " << kGoal;

    TileCoordsXYZ pos = kStart;
    auto* peep = Guest::generate(pos.toCoordsXYZ().toTileCentre());
    ASSERT_NE(peep, nullptr);
    peep->outsideOfPark = false;
    peep->guestHeadingToRideId = ride->id;

    const Direction moveDir = PathFinding::ChooseDirection(pos, goal, *peep, false, RideId::GetNull());
    ASSERT_NE(moveDir, kInvalidDirection);

    peep->peepDirection = moveDir;
    auto destination = CoordsDirectionDelta[moveDir] + peep->getLocation();
    peep->setDestination(destination, 2);

    constexpr int32_t kPlazaCrossingMaxSteps = 400;
    int32_t step = 0;
    int32_t interiorTilesVisited = 0;
    std::string route;
    TileCoordsXYZ lastTile{ -1, -1, -1 };
    while (pos != goal && step < kPlazaCrossingMaxSteps)
    {
        peep->performNextAction();
        ++step;

        pos = TileCoordsXYZ(peep->getLocation());
        EXPECT_PRED_FORMAT1(AssertIsNotForbiddenPosition, pos);
        EXPECT_NE(MapGetFootpathElement({ pos.toCoordsXY(), peep->nextLoc.z }), nullptr)
            << "Guest left the footpath at " << pos << " after " << step << " steps";
        if (pos != lastTile)
        {
            lastTile = pos;
            if (!route.empty())
            {
                route += " -> ";
            }
            route += "(" + std::to_string(pos.x) + "," + std::to_string(pos.y) + ")";
            if (IsPlazaInterior(pos))
            {
                ++interiorTilesVisited;
            }
        }
        if (::testing::Test::HasFailure())
            break;
    }

    PeepEntityRemove(peep);
    EXPECT_EQ(pos, goal) << "Guest did not reach the PlazaCrossing goal in " << kPlazaCrossingMaxSteps << " steps; ended at "
                         << pos << " after " << step << " steps. Route: " << route;
    EXPECT_GE(interiorTilesVisited, 4) << "Guest arrived but did not cross the plaza interior (only " << interiorTilesVisited
                                       << " interior tiles visited). Route: " << route;
}
