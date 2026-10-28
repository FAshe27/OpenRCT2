/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "StaffSetAssignmentAction.h"

#include "../../Diagnostic.h"
#include "../../entity/EntityList.h"
#include "../../entity/Staff.h"
#include "../../localisation/StringIds.h"
#include "../../ride/RideStaffing.h"

namespace OpenRCT2::GameActions
{
    StaffSetAssignmentAction::StaffSetAssignmentAction(EntityId spriteIndex, RideId rideIndex)
        : _spriteIndex(spriteIndex)
        , _rideIndex(rideIndex)
    {
    }

    void StaffSetAssignmentAction::AcceptParameters(GameActionParameterVisitor& visitor)
    {
        visitor.Visit("id", _spriteIndex);
        visitor.Visit("rideId", _rideIndex);
    }

    uint16_t StaffSetAssignmentAction::GetActionFlags() const
    {
        return GameAction::GetActionFlags() | Flags::AllowWhilePaused;
    }

    void StaffSetAssignmentAction::Serialise(DataSerialiser& stream)
    {
        GameAction::Serialise(stream);

        stream << DS_TAG(_spriteIndex) << DS_TAG(_rideIndex);
    }

    Result StaffSetAssignmentAction::Query(GameState_t& gameState, Park::ParkData& park) const
    {
        if (_spriteIndex.ToUnderlying() >= kMaxEntities || _spriteIndex.IsNull())
        {
            LOG_ERROR("Invalid sprite index %u", _spriteIndex);
            return Result(Status::invalidParameters, STR_ERR_INVALID_PARAMETER, STR_ERR_VALUE_OUT_OF_RANGE);
        }

        auto* staff = gameState.entities.tryGetEntity<Staff>(_spriteIndex);
        if (staff == nullptr || !staff->isWorker())
        {
            LOG_ERROR("Staff assignment can't be changed for staff of type %u", _spriteIndex);
            return Result(Status::invalidParameters, STR_ERR_INVALID_PARAMETER, STR_ERR_ACTION_INVALID_FOR_THAT_STAFF_TYPE);
        }

        if (!_rideIndex.IsNull())
        {
            auto* ride = GetRide(_rideIndex);
            if (ride == nullptr || RideStaffing::GetRequiredOperators(*ride) <= 0)
            {
                LOG_ERROR("Ride %u does not require operators", _rideIndex);
                return Result(Status::invalidParameters, STR_ERR_INVALID_PARAMETER, STR_ERR_VALUE_OUT_OF_RANGE);
            }
        }

        return Result();
    }

    Result StaffSetAssignmentAction::Execute(GameState_t& gameState, Park::ParkData& park) const
    {
        auto* staff = gameState.entities.tryGetEntity<Staff>(_spriteIndex);
        if (staff == nullptr)
        {
            LOG_ERROR("Staff entity not found for spriteIndex %u", _spriteIndex);
            return Result(Status::invalidParameters, STR_ERR_INVALID_PARAMETER, STR_ERR_STAFF_NOT_FOUND);
        }

        staff->setWorkerAssignment(_rideIndex);

        auto res = Result();
        res.position = staff->getLocation();
        return res;
    }
} // namespace OpenRCT2::GameActions
