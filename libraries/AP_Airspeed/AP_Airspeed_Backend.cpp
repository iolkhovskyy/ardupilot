/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
  backend driver class for airspeed
 */

#include "AP_Airspeed_config.h"

#if AP_AIRSPEED_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include "AP_Airspeed.h"
#include "AP_Airspeed_Backend.h"

extern const AP_HAL::HAL &hal;

AP_Airspeed_Backend::AP_Airspeed_Backend(AP_Airspeed &_frontend, uint8_t _instance) :
    frontend(_frontend),
    instance(_instance),
    _last_sample_filter_pa(0),
    _last_sample_filter_ms(0),
    _pressure_collapse_count(0)
{
}

AP_Airspeed_Backend::~AP_Airspeed_Backend(void)
{
}
 

int8_t AP_Airspeed_Backend::get_pin(void) const
{
#ifndef HAL_BUILD_AP_PERIPH
    return frontend.param[instance].pin;
#else
    return 0;
#endif
}

float AP_Airspeed_Backend::get_psi_range(void) const
{
    return frontend.param[instance].psi_range;
}

uint8_t AP_Airspeed_Backend::get_bus(void) const
{
    return frontend.param[instance].bus;
}

bool AP_Airspeed_Backend::bus_is_configured(void) const
{
    return frontend.param[instance].bus.configured();
}

void AP_Airspeed_Backend::set_bus_id(uint32_t id)
{
    frontend.param[instance].bus_id.set_and_save(int32_t(id));
}

bool AP_Airspeed_Backend::pressure_sample_ok(float pressure_pa)
{
    const uint32_t now_ms = AP_HAL::millis();

    // Below about 8 m/s (30 Pa at the default ratio) a zero sample is a
    // normal stop. Above that, dynamic pressure cannot fall to a quarter
    // of its previous value in one sample period.
    constexpr float MIN_PRESSURE_PA = 30.0f;
    constexpr float COLLAPSE_RATIO = 0.25f;
    constexpr uint32_t RECENT_MS = 200;
    constexpr uint8_t CONFIRM_COUNT = 3;

    const bool recent = _last_sample_filter_ms != 0 && (now_ms - _last_sample_filter_ms) < RECENT_MS;
    const bool collapse = recent &&
                          fabsf(_last_sample_filter_pa) > MIN_PRESSURE_PA &&
                          fabsf(pressure_pa) < fabsf(_last_sample_filter_pa) * COLLAPSE_RATIO;
    if (collapse && ++_pressure_collapse_count < CONFIRM_COUNT) {
        return false;
    }

    _pressure_collapse_count = 0;
    _last_sample_filter_pa = pressure_pa;
    _last_sample_filter_ms = now_ms;
    return true;
}

#endif  // AP_AIRSPEED_ENABLED
