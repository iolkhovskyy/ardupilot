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

#include "AP_Airspeed_config.h"

#if AP_AIRSPEED_ENABLED

#ifndef HAL_BUILD_AP_PERIPH

#include "AP_Airspeed_Filter.h"

#include <AP_Math/AP_Math.h>
#include <string.h>

void AP_Airspeed_Filter::push(float pressure_pa)
{
    window[window_head] = pressure_pa;
    window_head = (window_head + 1) % WINDOW_SIZE;
    if (window_count < WINDOW_SIZE) {
        window_count++;
    }
}

float AP_Airspeed_Filter::pressure_to_airspeed(float pressure_pa, float ratio)
{
    return sqrtf(MAX(pressure_pa, 0.0f) * MAX(ratio, 0.0f));
}

bool AP_Airspeed_Filter::iqr_reject(float pressure_pa) const
{
    if (window_count < IQR_MIN_SAMPLES) {
        return false;
    }

    float sorted[WINDOW_SIZE];
    memcpy(sorted, window, window_count * sizeof(float));

    // insertion sort — window is small (20)
    for (uint8_t i = 1; i < window_count; i++) {
        const float key = sorted[i];
        int8_t j = i - 1;
        while (j >= 0 && sorted[j] > key) {
            sorted[j + 1] = sorted[j];
            j--;
        }
        sorted[j + 1] = key;
    }

    const uint8_t q1_idx = window_count / 4;
    const uint8_t q2_idx = window_count / 2;
    const uint8_t q3_idx = (3 * window_count) / 4;

    const float q1 = sorted[q1_idx];
    const float q2 = sorted[q2_idx];
    const float q3 = sorted[q3_idx];
    const float iqr = MAX(q3 - q1, MIN_IQR_PA);
    const float lo = q2 - IQR_MULTIPLIER * iqr;
    const float hi = q2 + IQR_MULTIPLIER * iqr;

    return (pressure_pa < lo) || (pressure_pa > hi);
}

bool AP_Airspeed_Filter::roc_reject(float pressure_pa, float ratio, float dt_s) const
{
    if (!have_accepted || dt_s <= 0.0f) {
        return false;
    }

    const float aspd_new = pressure_to_airspeed(pressure_pa, ratio);
    const float aspd_last = pressure_to_airspeed(last_accepted_pa, ratio);
    const float accel = fabsf(aspd_new - aspd_last) / dt_s;
    return accel > MAX_ACCEL_MSS;
}

float AP_Airspeed_Filter::apply(float pressure_pa, float ratio, uint32_t now_ms)
{
    float dt_s = 0.0f;
    if (have_accepted) {
        dt_s = (now_ms - last_accepted_ms) * 0.001f;
    }

    const bool reject = iqr_reject(pressure_pa) || roc_reject(pressure_pa, ratio, dt_s);

    if (reject && have_accepted) {
        if (holds < MAX_HOLDS) {
            holds++;
            return last_accepted_pa;
        }
        // too many consecutive rejects — treat as a real step and resync
    }

    holds = 0;
    push(pressure_pa);
    last_accepted_pa = pressure_pa;
    last_accepted_ms = now_ms;
    have_accepted = true;
    return pressure_pa;
}

#endif // HAL_BUILD_AP_PERIPH

#endif // AP_AIRSPEED_ENABLED
