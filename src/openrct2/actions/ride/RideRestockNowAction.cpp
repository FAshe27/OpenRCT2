/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "RideRestockNowAction.h"

#include "../../Diagnostic.h"
#include "../../localisation/StringIds.h"
#include "../../ride/RideStaffing.h"

namespace OpenRCT2::GameActions
{
    RideRestockNowAction::RideRestockNowAction(RideId rideIndex)
        : _rideIndex(rideIndex)
    {
    }

    void RideRestockNowAction::AcceptParameters(GameActionParameterVisitor& visitor)
    {
        visitor.Visit("rideId", _rideIndex);
    }

    uint16_t RideRestockNowAction::GetActionFlags() const
    {
        return GameAction::GetActionFlags() | Flags::AllowWhilePaused;
    }

    void RideRestockNowAction::Serialise(DataSerialiser& stream)
    {
        GameAction::Serialise(stream);

        stream << DS_TAG(_rideIndex);
    }

    Result RideRestockNowAction::Query(GameState_t& gameState, Park::ParkData& park) const
    {
        auto* ride = GetRide(_rideIndex);
        if (ride == nullptr || !RideStaffing::CanRestockNow(*ride))
        {
            LOG_ERROR("Ride %u cannot dispatch a restock run now", _rideIndex);
            return Result(Status::invalidParameters, STR_ERR_INVALID_PARAMETER, STR_ERR_VALUE_OUT_OF_RANGE);
        }

        return Result();
    }

    Result RideRestockNowAction::Execute(GameState_t& gameState, Park::ParkData& park) const
    {
        auto* ride = GetRide(_rideIndex);
        if (ride == nullptr)
        {
            LOG_ERROR("Ride not found for rideIndex %u", _rideIndex);
            return Result(Status::invalidParameters, STR_ERR_INVALID_PARAMETER, STR_ERR_VALUE_OUT_OF_RANGE);
        }

        // The dispatch re-checks every condition itself; a run that no
        // longer applies (stock filled, crew stepped away) simply does
        // not start.
        RideStaffing::RequestRestock(*ride);

        return Result();
    }
} // namespace OpenRCT2::GameActions
