/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "RideSetStatusAction.h"

#include "../../Diagnostic.h"
#include "../../localisation/Formatter.h"
#include "../../localisation/StringIds.h"
#include "../../management/Finance.h"
#include "../../ride/Ride.h"
#include "../../ride/RideStaffing.h"
#include "../../ui/WindowManager.h"
#include "../../world/Map.h"
#include "../ResultWithMessage.h"

namespace OpenRCT2::GameActions
{
    static StringId _StatusErrorTitles[] = {
        STR_CANT_CLOSE,
        STR_CANT_OPEN,
        STR_CANT_TEST,
        STR_CANT_SIMULATE,
    };

    RideSetStatusAction::RideSetStatusAction(RideId rideIndex, RideStatus status)
        : _rideIndex(rideIndex)
        , _status(status)
    {
    }

    void RideSetStatusAction::AcceptParameters(GameActionParameterVisitor& visitor)
    {
        visitor.Visit("ride", _rideIndex);
        visitor.Visit("status", _status);
    }

    uint16_t RideSetStatusAction::GetActionFlags() const
    {
        return GameAction::GetActionFlags() | Flags::AllowWhilePaused;
    }

    void RideSetStatusAction::Serialise(DataSerialiser& stream)
    {
        GameAction::Serialise(stream);

        stream << DS_TAG(_rideIndex) << DS_TAG(_status);
    }

    Result RideSetStatusAction::Query(GameState_t& gameState, Park::ParkData& park) const
    {
        Result res = Result();

        auto ride = GetRide(_rideIndex);
        if (ride == nullptr)
        {
            LOG_ERROR("Ride not found for rideIndex %u", _rideIndex.ToUnderlying());
            res.error = Status::invalidParameters;
            res.errorTitle = STR_RIDE_DESCRIPTION_UNKNOWN;
            res.errorMessage = STR_ERR_RIDE_NOT_FOUND;
            return res;
        }

        if (_status >= RideStatus::count)
        {
            LOG_ERROR("Invalid ride status %u for ride %u", EnumValue(_status), _rideIndex.ToUnderlying());
            res.error = Status::invalidParameters;
            res.errorTitle = STR_RIDE_DESCRIPTION_UNKNOWN;
            res.errorMessage = kStringIdNone;
            return res;
        }

        res.errorTitle = _StatusErrorTitles[EnumValue(_status)];

        Formatter ft(res.errorMessageArgs.data());
        ride->formatNameTo(ft);
        if (_status != ride->status)
        {
            if (_status == RideStatus::simulating && ride->flags.has(RideFlag::brokenDown))
            {
                // Simulating will force clear the track, so make sure player can't cheat around a break down
                res.error = Status::disallowed;
                res.errorMessage = STR_HAS_BROKEN_DOWN_AND_REQUIRES_FIXING;
                return res;
            }

            ResultWithMessage modeSwitchResult = { true };
            switch (_status)
            {
                case RideStatus::open:
                    modeSwitchResult = ride->open(false);
                    break;
                case RideStatus::testing:
                    modeSwitchResult = ride->test(false);
                    break;
                case RideStatus::simulating:
                    modeSwitchResult = ride->simulate(false);
                    break;
                default:
                    break;
            }

            if (!modeSwitchResult.Successful)
            {
                res.error = Status::unknown;
                res.errorMessage = modeSwitchResult.Message;
                return res;
            }
        }
        return Result();
    }

    Result RideSetStatusAction::Execute(GameState_t& gameState, Park::ParkData& park) const
    {
        Result res = Result();
        res.expenditure = ExpenditureType::rideRunningCosts;

        auto ride = GetRide(_rideIndex);
        if (ride == nullptr)
        {
            LOG_ERROR("Ride not found for rideIndex %u", _rideIndex.ToUnderlying());
            res.error = Status::invalidParameters;
            res.errorTitle = STR_RIDE_DESCRIPTION_UNKNOWN;
            res.errorMessage = STR_ERR_RIDE_NOT_FOUND;
            return res;
        }

        res.errorTitle = _StatusErrorTitles[EnumValue(_status)];

        Formatter ft(res.errorMessageArgs.data());
        ft.Increment(6);
        ride->formatNameTo(ft);
        if (!ride->overallView.isNull())
        {
            auto location = ride->overallView.toTileCentre();
            res.position = { location, TileElementHeight(location) };
        }

        auto* windowMgr = Ui::GetWindowManager();

        switch (_status)
        {
            case RideStatus::closed:
                if (ride->status == _status || ride->status == RideStatus::simulating)
                {
                    if (!ride->flags.has(RideFlag::brokenDown))
                    {
                        ride->flags.unset(RideFlag::crashed);
                        RideClearForConstruction(*ride);
                        ride->removePeeps();
                    }
                }

                ride->cancelMorningTestRun();
                ride->cancelPostBreakdownTestRun();
                // Closing cancels any pending open request (Workers gate).
                ride->pendingOpenStatus = RideStatus::closed;
                ride->status = RideStatus::closed;
                ride->flags.unset(RideFlag::passStationNoStopping);
                ride->raceWinner = EntityId::GetNull();
                ride->windowInvalidateFlags.set(RideInvalidateFlag::main, RideInvalidateFlag::list);
                windowMgr->InvalidateByNumber(WindowClass::ride, _rideIndex.ToUnderlying());
                break;
            case RideStatus::simulating:
            {
                ride->flags.unset(RideFlag::crashed);
                RideClearForConstruction(*ride);
                ride->removePeeps();

                const auto modeSwitchResult = ride->simulate(true);
                if (!modeSwitchResult.Successful)
                {
                    res.error = Status::unknown;
                    res.errorMessage = modeSwitchResult.Message;
                    return res;
                }

                ride->cancelMorningTestRun();
                ride->cancelPostBreakdownTestRun();
                ride->pendingOpenStatus = RideStatus::closed;
                ride->status = _status;
                ride->flags.unset(RideFlag::passStationNoStopping);
                ride->raceWinner = EntityId::GetNull();
                ride->currentIssues = 0;
                ride->lastIssueTime = 0;
                ride->getMeasurement();
                ride->windowInvalidateFlags.set(RideInvalidateFlag::main, RideInvalidateFlag::list);
                windowMgr->InvalidateByNumber(WindowClass::ride, _rideIndex.ToUnderlying());
                break;
            }
            case RideStatus::testing:
            case RideStatus::open:
            {
                if (ride->status == _status)
                {
                    return res;
                }

                // Workers opening gate: opening (or testing) a closed ride
                // while staffing is engaged waits for the minimum crew to be
                // at post. The request is remembered (runtime-only), the crew
                // is summoned at once, and Ride::updatePendingOpen promotes
                // the ride the moment the crew completes. With the full crew
                // already at post the gate is transparent. Transitions from
                // simulating, and open-to-testing switches, are not gated.
                if (ride->status == RideStatus::closed && RideStaffing::IsEngaged())
                {
                    const int32_t required = RideStaffing::GetRequiredOperators(*ride);
                    if (required > 0 && RideStaffing::CountOperatorsAtPost(*ride) < required)
                    {
                        ride->pendingOpenStatus = _status;
                        // The trains materialise now, while the request
                        // waits; dispatch stays gated on promotion.
                        ride->createVehiclesForPendingOpen();
                        RideStaffing::SummonWorkers(*ride);
                        ride->windowInvalidateFlags.set(RideInvalidateFlag::main, RideInvalidateFlag::list);
                        windowMgr->InvalidateByNumber(WindowClass::ride, _rideIndex.ToUnderlying());
                        return res;
                    }
                }

                if (ride->status == RideStatus::simulating)
                {
                    RideClearForConstruction(*ride);
                    ride->removePeeps();
                }

                // Fix #3183: Make sure we close the construction window so the ride finishes any editing code before opening
                //            otherwise vehicles get added to the ride incorrectly (such as to a ghost station)
                WindowBase* constructionWindow = windowMgr->FindByNumber(
                    WindowClass::rideConstruction, _rideIndex.ToUnderlying());
                if (constructionWindow != nullptr)
                {
                    windowMgr->Close(*constructionWindow);
                }

                if (_status == RideStatus::testing)
                {
                    const auto modeSwitchResult = ride->test(true);
                    if (!modeSwitchResult.Successful)
                    {
                        res.error = Status::unknown;
                        res.errorMessage = modeSwitchResult.Message;
                        return res;
                    }
                    // A manual test is the player's own: never let a stale
                    // morning-test arm flip it open afterwards.
                    ride->cancelMorningTestRun();
                    ride->cancelPostBreakdownTestRun();
                }
                else
                {
                    const auto modeSwitchResult = ride->open(true);
                    if (!modeSwitchResult.Successful)
                    {
                        res.error = Status::unknown;
                        res.errorMessage = modeSwitchResult.Message;
                        return res;
                    }
                }

                ride->raceWinner = EntityId::GetNull();
                // Morning test run (W2): on the first opening of an in-game
                // day, an eligible coaster stays in the stock testing state
                // for one empty circuit; Ride::updateMorningTestRun flips it
                // to open when every train is back. Manual re-open while a
                // test is armed lands here too: tryArm refuses (already
                // armed / already tested today), so the ride opens at once.
                ride->status = (_status == RideStatus::open && ride->tryArmMorningTestRun()) ? RideStatus::testing : _status;
                ride->currentIssues = 0;
                ride->lastIssueTime = 0;
                ride->getMeasurement();
                ride->windowInvalidateFlags.set(RideInvalidateFlag::main, RideInvalidateFlag::list);
                windowMgr->InvalidateByNumber(WindowClass::ride, _rideIndex.ToUnderlying());
                break;
            }
            default:
                Guard::Assert(false, "Invalid ride status %u", _status);
                break;
        }
        auto windowManager = Ui::GetWindowManager();
        windowManager->BroadcastIntent(Intent(INTENT_ACTION_REFRESH_CAMPAIGN_RIDE_LIST));

        return res;
    }
} // namespace OpenRCT2::GameActions
