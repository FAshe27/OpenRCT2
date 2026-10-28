/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "Paint.Staff.h"

#include "../../drawing/LightFX.h"
#include "../../entity/Staff.h"
#include "../../profiling/Profiling.h"
#include "../Paint.h"
#include "Paint.Peep.h"

using namespace OpenRCT2;

namespace LightFx = OpenRCT2::Drawing::LightFx;

void PaintStaff(PaintSession& session, const Staff& staff, int32_t orientation)
{
    PROFILED_FUNCTION();

    // Workers: an operator at post on any ride class but a coaster works
    // out of sight and is not drawn (a stall operator behind the counter;
    // flat-ride and tracked non-coaster crews likewise) - they would
    // otherwise stand frozen at the post. Only coaster crews stand
    // visibly in the station. Which classes hide is RideStaffing's call
    // (AreOperatorsVisibleAtPost); visibility is derived from state at
    // paint time and never stored: the peep stays fully simulated, and
    // leaving the post for any reason draws them again. Skipping the
    // paint also removes them from in-world picking, which reads the
    // same paint pass.
    if (staff.isHiddenAtPost())
    {
        return;
    }

    if (session.rt.zoom_level > ZoomLevel{ 2 })
    {
        return;
    }

    PaintStaffLightingEffects(staff);

    Direction direction = (orientation >> 3);
    auto baseImageData = PaintPeepGetBaseImageAndOffset(staff, direction);
    auto imageId = ImageId(baseImageData.baseImageId, staff.tShirtColour);

    PaintAddImageAsParent(session, imageId, kPaintPeepOffset(staff.z), kPaintPeepBoundBox(staff.z));
}

void PaintStaffLightingEffects(const Staff& staff)
{
    if (!LightFx::IsAvailable())
        return;

    auto loc = staff.getLocation();
    switch (staff.orientation)
    {
        case 0:
            loc.x -= 10;
            break;
        case 8:
            loc.y += 10;
            break;
        case 16:
            loc.x += 10;
            break;
        case 24:
            loc.y -= 10;
            break;
        default:
            return;
    }

    LightFx::Add3DLight(staff, 0, loc, LightFx::LightType::spot1);
}
