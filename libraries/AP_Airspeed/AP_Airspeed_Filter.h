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
#pragma once

/*
  Adaptive rolling IQR + rate-of-change spike filter for differential
  airspeed pressure. Intended for the 10 Hz AP_Airspeed::read() path.
 */

#include <stdint.h>

class AP_Airspeed_Filter
{
public:
    AP_Airspeed_Filter() {}

    // Filter corrected differential pressure (Pa).
    // ratio is ARSPD_RATIO used for the airspeed rate-of-change gate.
    // now_ms is the sample timestamp; pass AP_HAL::millis() in production.
    float apply(float pressure_pa, float ratio, uint32_t now_ms);

    // Clear consecutive-hold state (e.g. after sensor becomes healthy again).
    void reset_holds()
    {
        holds = 0;
    }

private:
    static const uint8_t WINDOW_SIZE = 20;
    static const uint8_t IQR_MIN_SAMPLES = 10;
    static const uint8_t MAX_HOLDS = 3;
    static constexpr float IQR_MULTIPLIER = 2.0f;
    static constexpr float MAX_ACCEL_MSS = 15.0f;
    static constexpr float MIN_IQR_PA = 5.0f;

    float window[WINDOW_SIZE] {};
    uint8_t window_count = 0;
    uint8_t window_head = 0;
    float last_accepted_pa = 0.0f;
    uint32_t last_accepted_ms = 0;
    bool have_accepted = false;
    uint8_t holds = 0;


    void push(float pressure_pa);
    bool iqr_reject(float pressure_pa) const;
    bool roc_reject(float pressure_pa, float ratio, float dt_s) const;
    static float pressure_to_airspeed(float pressure_pa, float ratio);
};