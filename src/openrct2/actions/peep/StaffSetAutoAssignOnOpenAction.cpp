/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "StaffSetAutoAssignOnOpenAction.h"

#include "../../world/ParkData.h"

namespace OpenRCT2::GameActions
{
    StaffSetAutoAssignOnOpenAction::StaffSetAutoAssignOnOpenAction(bool enabled)
        : _enabled(enabled)
    {
    }

    void StaffSetAutoAssignOnOpenAction::AcceptParameters(GameActionParameterVisitor& visitor)
    {
        visitor.Visit("enabled", _enabled);
    }

    uint16_t StaffSetAutoAssignOnOpenAction::GetActionFlags() const
    {
        return GameAction::GetActionFlags() | Flags::AllowWhilePaused;
    }

    void StaffSetAutoAssignOnOpenAction::Serialise(DataSerialiser& stream)
    {
        GameAction::Serialise(stream);

        stream << DS_TAG(_enabled);
    }

    Result StaffSetAutoAssignOnOpenAction::Query(GameState_t& gameState, Park::ParkData& park) const
    {
        return Result();
    }

    Result StaffSetAutoAssignOnOpenAction::Execute(GameState_t& gameState, Park::ParkData& park) const
    {
        // Stored inverted (bit set = disabled) so parks saved before the
        // toggle existed read as enabled, the gate behaviour they shipped
        // with. The bit rides the already-streamed ParkFlag u64, so this
        // needs no ParkFile version change.
        if (_enabled)
            park.flags.unset(ParkFlag::workersAutoAssignDisabled);
        else
            park.flags.set(ParkFlag::workersAutoAssignDisabled);
        return Result();
    }
} // namespace OpenRCT2::GameActions
