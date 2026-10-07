#include <AP_gtest.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Airspeed/AP_Airspeed_config.h>

const AP_HAL::HAL &hal = AP_HAL::get_HAL();

#if AP_AIRSPEED_ENABLED

#include <AP_Airspeed/AP_Airspeed_Backend.h>

/*
  The MS4525 and DLVR drivers drop I2C samples that collapse toward zero
  through AP_Airspeed_Backend::pressure_sample_ok(). These tests lock that
  behaviour: an isolated mid-scale reading is discarded, and a real drop is
  accepted once it repeats.
 */

class PressureSampleTester : public AP_Airspeed_Backend {
public:
    PressureSampleTester() : AP_Airspeed_Backend(frontend(), 0) {}

    bool init() override { return true; }
    bool get_temperature(float &temperature) override {
        (void)temperature;
        return false;
    }

    bool sample_ok(float pressure_pa) { return pressure_sample_ok(pressure_pa); }

    // MS4525 keeps a pair only when both reads pass, and it does not
    // evaluate the second read when the first one fails.
    bool ms4525_pair_ok(float first_pa, float second_pa) {
        if (!sample_ok(first_pa)) {
            return false;
        }
        return sample_ok(second_pa);
    }

private:
    static AP_Airspeed &frontend() {
        static AP_Airspeed airspeed;
        return airspeed;
    }
};

static uint64_t sim_time_us;
static bool sim_time_started;

static void ensure_clock()
{
    if (sim_time_started) {
        return;
    }
    sim_time_us = 1000ULL * 1000ULL;
    hal.scheduler->stop_clock(sim_time_us);
    sim_time_started = true;
}

static void advance_ms(uint32_t ms)
{
    ensure_clock();
    sim_time_us += uint64_t(ms) * 1000ULL;
    hal.scheduler->stop_clock(sim_time_us);
}

static void jump_to_just_before_millis_wrap()
{
    ensure_clock();
    const uint64_t millis = sim_time_us / 1000ULL;
    const uint64_t next_wrap_ms = (millis & ~uint64_t(0xFFFFFFFF)) + (uint64_t(1) << 32);
    sim_time_us = (next_wrap_ms - 16) * 1000ULL;
    hal.scheduler->stop_clock(sim_time_us);
}

// DLVR calls the filter once per sample. MS4525 calls it once per read.

TEST(AirspeedPressureSample, FirstSampleIsAccepted)
{
    PressureSampleTester filter;
    advance_ms(20);
    EXPECT_TRUE(filter.sample_ok(0.0f));

    PressureSampleTester cruise;
    advance_ms(20);
    EXPECT_TRUE(cruise.sample_ok(200.0f));

    PressureSampleTester suction;
    advance_ms(20);
    EXPECT_TRUE(suction.sample_ok(-200.0f));
}

TEST(AirspeedPressureSample, CruiseNoiseIsAccepted)
{
    PressureSampleTester filter;
    const float samples[] = {200.0f, 198.0f, 203.0f, 195.0f, 205.0f};
    for (float sample : samples) {
        advance_ms(20);
        EXPECT_TRUE(filter.sample_ok(sample));
    }
}

TEST(AirspeedPressureSample, IsolatedZeroIsDropped)
{
    PressureSampleTester filter;
    advance_ms(20);
    EXPECT_TRUE(filter.sample_ok(200.0f));

    advance_ms(20);
    EXPECT_FALSE(filter.sample_ok(0.0f));

    // The following real sample is kept, so one glitch cannot skew the average.
    advance_ms(20);
    EXPECT_TRUE(filter.sample_ok(198.0f));
}

TEST(AirspeedPressureSample, ThirdConsecutiveCollapseIsAccepted)
{
    PressureSampleTester filter;
    advance_ms(20);
    EXPECT_TRUE(filter.sample_ok(200.0f));

    advance_ms(20);
    EXPECT_FALSE(filter.sample_ok(0.0f));
    advance_ms(20);
    EXPECT_FALSE(filter.sample_ok(0.0f));
    advance_ms(20);
    EXPECT_TRUE(filter.sample_ok(0.0f));

    // The new baseline is zero, so further zeros are normal.
    advance_ms(20);
    EXPECT_TRUE(filter.sample_ok(0.0f));
    advance_ms(20);
    EXPECT_TRUE(filter.sample_ok(180.0f));
}

TEST(AirspeedPressureSample, GoodSampleResetsCollapseCount)
{
    PressureSampleTester filter;
    advance_ms(20);
    EXPECT_TRUE(filter.sample_ok(200.0f));

    advance_ms(20);
    EXPECT_FALSE(filter.sample_ok(0.0f));
    advance_ms(20);
    EXPECT_FALSE(filter.sample_ok(0.0f));

    advance_ms(20);
    EXPECT_TRUE(filter.sample_ok(200.0f));

    // Count was cleared, so this is an isolated glitch again.
    advance_ms(20);
    EXPECT_FALSE(filter.sample_ok(0.0f));
}

TEST(AirspeedPressureSample, PartialDropIsKept)
{
    PressureSampleTester filter;
    advance_ms(20);
    EXPECT_TRUE(filter.sample_ok(200.0f));

    // Exactly a quarter of the previous sample is still a plausible step.
    advance_ms(20);
    EXPECT_TRUE(filter.sample_ok(50.0f));

    PressureSampleTester collapsed;
    advance_ms(20);
    EXPECT_TRUE(collapsed.sample_ok(200.0f));
    advance_ms(20);
    EXPECT_FALSE(collapsed.sample_ok(49.0f));
}

TEST(AirspeedPressureSample, BelowGateZeroIsAccepted)
{
    PressureSampleTester at_gate;
    advance_ms(20);
    EXPECT_TRUE(at_gate.sample_ok(30.0f));
    advance_ms(20);
    EXPECT_TRUE(at_gate.sample_ok(0.0f));

    PressureSampleTester above_gate;
    advance_ms(20);
    EXPECT_TRUE(above_gate.sample_ok(31.0f));
    advance_ms(20);
    EXPECT_FALSE(above_gate.sample_ok(0.0f));
}

TEST(AirspeedPressureSample, NegativePressureUsesMagnitude)
{
    PressureSampleTester filter;
    advance_ms(20);
    EXPECT_TRUE(filter.sample_ok(-200.0f));

    advance_ms(20);
    EXPECT_TRUE(filter.sample_ok(-160.0f));

    advance_ms(20);
    EXPECT_FALSE(filter.sample_ok(0.0f));
    advance_ms(20);
    EXPECT_FALSE(filter.sample_ok(-10.0f));

    // A larger suction is not a collapse.
    advance_ms(20);
    EXPECT_TRUE(filter.sample_ok(-220.0f));
}

TEST(AirspeedPressureSample, GradualStopIsAccepted)
{
    PressureSampleTester filter;
    const float samples[] = {200.0f, 160.0f, 120.0f, 90.0f, 70.0f, 50.0f, 40.0f, 32.0f, 24.0f, 16.0f, 8.0f, 0.0f};
    for (float sample : samples) {
        advance_ms(20);
        EXPECT_TRUE(filter.sample_ok(sample)) << sample;
    }
}

TEST(AirspeedPressureSample, RejectedSampleDoesNotRefreshWindow)
{
    PressureSampleTester filter;
    advance_ms(20);
    EXPECT_TRUE(filter.sample_ok(200.0f));

    advance_ms(100);
    EXPECT_FALSE(filter.sample_ok(0.0f));

    // 200 ms after the last accepted sample the window has expired.
    // Updating the timestamp on the rejected sample would still be inside it.
    advance_ms(100);
    EXPECT_TRUE(filter.sample_ok(0.0f));
}

TEST(AirspeedPressureSample, SampleJustInsideWindowIsDropped)
{
    PressureSampleTester filter;
    advance_ms(20);
    EXPECT_TRUE(filter.sample_ok(200.0f));

    advance_ms(199);
    EXPECT_FALSE(filter.sample_ok(0.0f));
}

TEST(AirspeedPressureSample, MillisWrapStillSeesRecentSample)
{
    jump_to_just_before_millis_wrap();

    PressureSampleTester filter;
    EXPECT_TRUE(filter.sample_ok(200.0f));

    advance_ms(32);
    EXPECT_TRUE(filter.sample_ok(190.0f));

    advance_ms(20);
    EXPECT_FALSE(filter.sample_ok(0.0f));
}

TEST(AirspeedPressureSample, MS4525PairDropsASingleZeroRead)
{
    PressureSampleTester filter;
    advance_ms(20);
    EXPECT_TRUE(filter.ms4525_pair_ok(200.0f, 198.0f));

    // The two reads can agree closely enough to pass the raw delta check
    // while one of them is still a zero. The pair must be dropped.
    advance_ms(20);
    EXPECT_FALSE(filter.ms4525_pair_ok(200.0f, 0.0f));
    advance_ms(20);
    EXPECT_FALSE(filter.ms4525_pair_ok(0.0f, 200.0f));

    advance_ms(20);
    EXPECT_TRUE(filter.ms4525_pair_ok(200.0f, 202.0f));
}

TEST(AirspeedPressureSample, MS4525PairAcceptsPersistentZero)
{
    PressureSampleTester filter;
    advance_ms(20);
    EXPECT_TRUE(filter.ms4525_pair_ok(200.0f, 200.0f));

    advance_ms(20);
    EXPECT_FALSE(filter.ms4525_pair_ok(0.0f, 0.0f));
    advance_ms(20);
    EXPECT_FALSE(filter.ms4525_pair_ok(0.0f, 0.0f));
    advance_ms(20);
    EXPECT_TRUE(filter.ms4525_pair_ok(0.0f, 0.0f));
}

#endif  // AP_AIRSPEED_ENABLED

AP_GTEST_MAIN()
