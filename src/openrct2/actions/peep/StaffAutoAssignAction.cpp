/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "StaffAutoAssignAction.h"

#include "../../ride/RideStaffing.h"

namespace OpenRCT2::GameActions
{
    void StaffAutoAssignAction::AcceptParameters(GameActionParameterVisitor&)
    {
    }

    uint16_t StaffAutoAssignAction::GetActionFlags() const
    {
        return GameAction::GetActionFlags() | Flags::AllowWhilePaused;
    }

    void StaffAutoAssignAction::Serialise(DataSerialiser& stream)
    {
        GameAction::Serialise(stream);
    }

    Result StaffAutoAssignAction::Query(GameState_t& gameState, Park::ParkData& park) const
    {
        return Result();
    }

    Result StaffAutoAssignAction::Execute(GameState_t& gameState, Park::ParkData& park) const
    {
        RideStaffing::AutoAssignAll();
        return Result();
    }
} // namespace OpenRCT2::GameActions
