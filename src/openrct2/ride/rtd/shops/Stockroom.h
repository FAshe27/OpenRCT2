/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#pragma once

#include "../../RideData.h"
#include "../../RideStringIds.h"
#include "../../ShopItem.h"

// clang-format off
namespace OpenRCT2
{
// Restock: the stockroom is the break room's sibling - the toilets
// facility with the guest interaction removed, used as a supply
// building instead of a rest area. The same four absences make guest
// non-interaction structural rather than filtered: no specialType (left
// at the default, none), no guestsShouldGoInsideFacility flag, no shop
// item sold and no food/drink classification - so no guest goal, thought
// or interaction path can ever target it. Workers reach it through the
// worker transit machinery on a restock run, never through guest logic.
// The cloned object (fashe.ride.stockroom1) binds to this type by the
// Name below and borrows the toilets' sprites as placeholder art.
constexpr RideTypeDescriptor kStockroomRTD =
{
    .Category = RideCategory::shop,
    .StartTrackPiece = TrackElemType::flatTrack1x1A,
    .TrackPaintFunctions = TrackDrawerDescriptor({
        .trackStyle = TrackStyle::facility,
        .enabledTrackGroups = {},
        .extraTrackGroups = {},
    }),
    .InvertedTrackPaintFunctions = {},
    .flags = RtdFlags(RtdFlag::hasSinglePieceStation, RtdFlag::cannotHaveGaps, RtdFlag::noTestMode,
                     RtdFlag::noVehicles, RtdFlag::isShopOrFacility, RtdFlag::noWallsAroundTrack,
                     RtdFlag::isFlatRide, RtdFlag::describeAsInside,
                     RtdFlag::listVehiclesSeparately),
    .rideModes = { RideMode::shopStall },
    .DefaultMode = RideMode::shopStall,
    .OperatingSettings = { 4, 4 },
    .Naming = { STR_RIDE_NAME_STOCKROOM, STR_RIDE_DESCRIPTION_STOCKROOM },
    .NameConvention = { RideComponentType::car, RideComponentType::building, RideComponentType::station },
    .availableBreakdowns = {},
    .Heights = { 12, kDefaultToiletHeight, 0, 0, },
    .MaxMass = 255,
    .LiftData = { Audio::SoundId::null, 5, 5 },
    .RatingsMultipliers = { 0, 0, 0 },
    .UpkeepCosts = { 50, 1, 0, 0, 0, 0 },
    .BuildCosts = { 225.00_GBP, 0.00_GBP, 1, },
    .DefaultPrices = { 0, 0 },
    .DefaultMusic = kMusicObjectGentle,
    .PhotoItem = ShopItem::photo,
    .BonusValue = 5,
    .ColourPresets = kDefaultFlatRideColourPreset,
    .ColourPreview = { 0, 0 },
    .ColourKey = RideColourKey::toilets,
    .Name = "stock_room",
    .RatingsData =
    {
        RatingsCalculationType::stall,
        { 1, 1, 1 },
        1,
        kDynamicRideShelterRating,
        false,
        { { RatingsModifierType::noModifier, 0, 0, 0, 0 } }
    },
    .UpdateRotating = UpdateRotatingDefault,
    .LightFXAddLightsMagicVehicle = nullptr,
    .StartRideMusic = RideAudio::DefaultStartRideMusicChannel,
    .DesignCreateMode = TrackDesignCreateMode::standard,
    .MusicUpdateFunction = DefaultMusicUpdate,
    .Classification = RideClassification::kioskOrFacility,
};
} // namespace OpenRCT2
// clang-format on
