/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    10|   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  driver for DLVR differential airspeed sensor
  https://www.allsensors.com/products/DLVR-L01D
 */

#include "AP_Airspeed_DLVR.h"

#if AP_AIRSPEED_DLVR_ENABLED

#include <AP_Math/AP_Math.h>
#include <AP_Logger/AP_Logger.h>

extern const AP_HAL::HAL &hal;

#define DLVR_I2C_ADDR 0x28

#ifdef DLVR_DEBUGGING
 # define Debug(fmt, args ...)  do {hal.console->printf("%s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args); hal.scheduler->delay(1); } while(0)
#else
 # define Debug(fmt, args ...)
#endif


AP_Airspeed_DLVR::AP_Airspeed_DLVR(AP_Airspeed &_frontend, uint8_t _instance, const float _range_inH2O) :
    AP_Airspeed_Backend(_frontend, _instance),
    range_inH2O(_range_inH2O),
    fail_active(false),
    fail_start_ms(0),
    fail_pulls(0),
    fail_last_log_ms(0),
    fail_bytes{},
    fail_zero_bytes(0),
    fail_reason(FailReason::I2C)
{}

/*
  probe for a sensor on a given i2c address
 */
AP_Airspeed_Backend *AP_Airspeed_DLVR::probe(AP_Airspeed &_frontend,
                                             uint8_t _instance,
                                             AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev,
                                             const float _range_inH2O)
{
    if (!_dev) {
        return nullptr;
    }
    AP_Airspeed_DLVR *sensor = new AP_Airspeed_DLVR(_frontend, _instance, _range_inH2O);
    if (!sensor) {
        return nullptr;
    }
    sensor->dev = std::move(_dev);
    sensor->setup();
    return sensor;
}

// initialise the sensor
void AP_Airspeed_DLVR::setup()
{
    WITH_SEMAPHORE(dev->get_semaphore());
    dev->set_speed(AP_HAL::Device::SPEED_LOW);
    dev->set_retries(2);
    dev->set_device_type(uint8_t(DevType::DLVR));
    set_bus_id(dev->get_bus_id());

    dev->register_periodic_callback(1000000UL/50U,
                                    FUNCTOR_BIND_MEMBER(&AP_Airspeed_DLVR::timer, void));
}

// probe and initialise the sensor
bool AP_Airspeed_DLVR::init()
{
    dev = hal.i2c_mgr->get_device(get_bus(), DLVR_I2C_ADDR);
    if (!dev) {
        return false;
    }
    setup();
    return true;
}

#define STATUS_SHIFT 30
#define TEMPERATURE_SHIFT 5
#define TEMPERATURE_MASK ((1 << 11) - 1)
#define PRESSURE_SHIFT 16
#define PRESSURE_MASK ((1 << 14) - 1)

#define DLVR_OFFSET 8192.0f
#define DLVR_SCALE 16384.0f

// While an invalid-pull failure is still running, write progress at this
// millisecond spacing. The closing entry is written when a sample is accepted.
#define DLVR_FAIL_LOG_MS 1000

bool AP_Airspeed_DLVR::raw_input_bad_zero(const uint8_t raw_bytes[4], uint32_t data, uint8_t &zero_bytes) const
{
    zero_bytes = 0;
    for (uint8_t i = 0; i < 4; i++) {
        if (raw_bytes[i] == 0) {
            zero_bytes++;
        }
    }

    // 14-bit pressure code straight from the register. Zero is the bottom
    // code, not zero differential pressure (that sits near mid-scale).
    const uint32_t pres_raw = (data >> PRESSURE_SHIFT) & PRESSURE_MASK;
    return (zero_bytes >= 3) || (pres_raw == 0);
}

void AP_Airspeed_DLVR::note_failed_pull(uint32_t now_ms, FailReason reason, const uint8_t *raw_bytes, uint8_t zero_bytes)
{
    const bool first = !fail_active;
    if (first) {
        fail_active = true;
        fail_start_ms = now_ms;
        fail_pulls = 0;
        fail_last_log_ms = 0;
    }

    fail_pulls++;
    fail_reason = reason;
    if (raw_bytes != nullptr) {
        fail_bytes[0] = raw_bytes[0];
        fail_bytes[1] = raw_bytes[1];
        fail_bytes[2] = raw_bytes[2];
        fail_bytes[3] = raw_bytes[3];
        fail_zero_bytes = zero_bytes;
    } else {
        fail_bytes[0] = fail_bytes[1] = fail_bytes[2] = fail_bytes[3] = 0;
        fail_zero_bytes = 0;
    }

    // First bad pull is logged immediately. Later ones are paced.
    if (first ||
        (fail_last_log_ms == 0) ||
        ((now_ms - fail_last_log_ms) >= DLVR_FAIL_LOG_MS)) {
        log_failed_pull(now_ms, false);
    }
}

// @LoggerMessage: DLVZ
// @Description: DLVR invalid-pull failure incident. Pulls is the number of sensor reads during the encounter. DtMS is milliseconds from the first bad read to this entry. End is 0 while running and 1 on the closing entry. Reason: 0=I2C,1=zero-input,2=status,3=range,4=temp-swing. B0-B3/ZB are from the last bad read when available.
// @Field: TimeUS: Time since system startup
// @Field: Inst: airspeed sensor instance
// @Field: Pulls: sensor reads counted during this failure
// @Field: DtMS: milliseconds from the first bad read to this entry
// @Field: StartMS: raw millisecond time of the first bad read
// @Field: End: 1 when this entry closes the failure
// @Field: Reason: failure reason code
// @Field: PRaw: raw 14-bit pressure code from the last bad read
// @Field: TRaw: raw 11-bit temperature code from the last bad read
// @Field: B0: last bad raw byte 0
// @Field: B1: last bad raw byte 1
// @Field: B2: last bad raw byte 2
// @Field: B3: last bad raw byte 3
// @Field: ZB: number of zero bytes in B0-B3

void AP_Airspeed_DLVR::log_failed_pull(uint32_t now_ms, bool ended)
{
    if (!fail_active) {
        return;
    }

#if HAL_LOGGING_ENABLED
    const uint32_t dt_ms = now_ms - fail_start_ms;
    const uint32_t raw_word =
        (uint32_t(fail_bytes[0]) << 24) |
        (uint32_t(fail_bytes[1]) << 16) |
        (uint32_t(fail_bytes[2]) << 8) |
        uint32_t(fail_bytes[3]);
    const uint16_t pres_raw = uint16_t((raw_word >> PRESSURE_SHIFT) & PRESSURE_MASK);
    const uint16_t temp_raw = uint16_t((raw_word >> TEMPERATURE_SHIFT) & TEMPERATURE_MASK);
    // Use WriteStreaming so DLVZ appears alongside DLVR in GCS log browsers.
    AP::logger().WriteStreaming("DLVZ",
                                "TimeUS,Inst,Pulls,DtMS,StartMS,End,Reason,PRaw,TRaw,B0,B1,B2,B3,ZB",
                                "s#------------",
                                "F-------------",
                                "QBIIIBBHHBBBBB",
                                AP_HAL::micros64(),
                                get_instance(),
                                fail_pulls,
                                dt_ms,
                                fail_start_ms,
                                uint8_t(ended ? 1 : 0),
                                uint8_t(fail_reason),
                                pres_raw,
                                temp_raw,
                                fail_bytes[0],
                                fail_bytes[1],
                                fail_bytes[2],
                                fail_bytes[3],
                                fail_zero_bytes);
    fail_last_log_ms = now_ms;
#else
    (void)now_ms;
#endif

    if (ended) {
        fail_active = false;
        fail_pulls = 0;
        fail_last_log_ms = 0;
    }
}

// @LoggerMessage: DLVR
// @Description: DLVR raw register reading. PRaw and TRaw are the pressure and temperature codes before conversion. St is the 2-bit status field. B0-B3 are the four I2C bytes.
// @Field: TimeUS: Time since system startup
// @Field: Inst: airspeed sensor instance
// @Field: PRaw: raw 14-bit pressure code
// @Field: TRaw: raw 11-bit temperature code
// @Field: St: raw 2-bit status
// @Field: B0: raw byte 0
// @Field: B1: raw byte 1
// @Field: B2: raw byte 2
// @Field: B3: raw byte 3

void AP_Airspeed_DLVR::log_raw_reading(uint32_t data, const uint8_t raw_bytes[4])
{
#if HAL_LOGGING_ENABLED
    const uint16_t pres_raw = uint16_t((data >> PRESSURE_SHIFT) & PRESSURE_MASK);
    const uint16_t temp_raw = uint16_t((data >> TEMPERATURE_SHIFT) & TEMPERATURE_MASK);
    const uint8_t status = uint8_t(data >> STATUS_SHIFT);
    AP::logger().WriteStreaming("DLVR",
                                "TimeUS,Inst,PRaw,TRaw,St,B0,B1,B2,B3",
                                "s#-------",
                                "F--------",
                                "QBHHBBBBB",
                                AP_HAL::micros64(),
                                get_instance(),
                                pres_raw,
                                temp_raw,
                                status,
                                raw_bytes[0],
                                raw_bytes[1],
                                raw_bytes[2],
                                raw_bytes[3]);
#else
    (void)data;
    (void)raw_bytes;
#endif
}

// 50Hz timer
void AP_Airspeed_DLVR::timer()
{
    const uint32_t now_ms = AP_HAL::millis();

    uint8_t raw_bytes[4];
    if (!dev->read((uint8_t *)&raw_bytes, sizeof(raw_bytes))) {
        // I2C read failed: still count this pull toward the incident.
        note_failed_pull(now_ms, FailReason::I2C, nullptr, 0);
        return;
    }

    uint32_t data = (raw_bytes[0] << 24) |
                    (raw_bytes[1] << 16) |
                    (raw_bytes[2] << 8)  |
                    raw_bytes[3];

    // Always log the raw register fields, before conversion or health checks.
    log_raw_reading(data, raw_bytes);

    uint8_t zero_bytes = 0;
    if (raw_input_bad_zero(raw_bytes, data, zero_bytes)) {
        note_failed_pull(now_ms, FailReason::ZERO_INPUT, raw_bytes, zero_bytes);
        return;
    }

    if ((data >> STATUS_SHIFT)) {
        // anything other then 00 in the status bits is an error
        Debug("DLVR: Bad status read %u", (unsigned int)(data >> STATUS_SHIFT));
        note_failed_pull(now_ms, FailReason::STATUS, raw_bytes, zero_bytes);
        return;
    }

    uint32_t pres_raw = (data >> PRESSURE_SHIFT)    & PRESSURE_MASK;
    uint32_t temp_raw = (data >> TEMPERATURE_SHIFT) & TEMPERATURE_MASK;

    float press_h2o = 1.25f * 2.0f * range_inH2O * ((pres_raw - DLVR_OFFSET) / DLVR_SCALE);

    if ((press_h2o > range_inH2O) || (press_h2o < -range_inH2O)) {
        Debug("DLVR: Out of range pressure %f", press_h2o);
        note_failed_pull(now_ms, FailReason::RANGE, raw_bytes, zero_bytes);
        return;
    }

    float temp = temp_raw * (200.0f / 2047.0f) - 50.0f;

    WITH_SEMAPHORE(sem);

    const uint32_t now = AP_HAL::millis();

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wfloat-equal" // suppress -Wfloat-equal as we are only worried about the case where we had never read
    if ((temperature != 0) && (fabsf(temperature - temp) > 10) && ((now - last_sample_time_ms) < 2000)) {
        Debug("DLVR: Temperature swing %f", temp);
        note_failed_pull(now_ms, FailReason::TEMP_SWING, raw_bytes, zero_bytes);
        return;
    }
#pragma GCC diagnostic pop

    if (fail_active) {
        log_failed_pull(now_ms, true);
    }

    pressure_sum += INCH_OF_H2O_TO_PASCAL * press_h2o;
    temperature_sum += temp;
    press_count++;
    temp_count++;
    last_sample_time_ms = now;
}

// return the current differential_pressure in Pascal
bool AP_Airspeed_DLVR::get_differential_pressure(float &_pressure)
{
    WITH_SEMAPHORE(sem);

    if ((AP_HAL::millis() - last_sample_time_ms) > 100) {
        return false;
    }

    if (press_count > 0) {
        pressure = pressure_sum / press_count;
        press_count = 0;
        pressure_sum = 0;
    }

    _pressure = pressure;
    return true;
}

// return the current temperature in degrees C, if available
bool AP_Airspeed_DLVR::get_temperature(float &_temperature)
{
    WITH_SEMAPHORE(sem);

    if ((AP_HAL::millis() - last_sample_time_ms) > 100) {
        return false;
    }

    if (temp_count > 0) {
        temperature = temperature_sum / temp_count;
        temp_count = 0;
        temperature_sum = 0;
    }

    _temperature = temperature;
    return true;
}

#endif
