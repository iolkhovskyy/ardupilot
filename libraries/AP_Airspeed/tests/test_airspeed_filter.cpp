#include <AP_gtest.h>

#include <AP_Airspeed/AP_Airspeed_Filter.h>
#include <AP_Math/AP_Math.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

// Typical Plane ARSPD_RATIO
static const float RATIO = 2.0f;
// Match AP_Airspeed_Filter::MAX_HOLDS
static const uint8_t MAX_HOLDS = 3;
// 10 Hz sample period used by AP_Airspeed::update()
static const uint32_t DT_MS = 100;

static void feed_constant(AP_Airspeed_Filter &filt, float pressure_pa, uint8_t count, uint32_t &t_ms)
{
    for (uint8_t i = 0; i < count; i++) {
        EXPECT_FLOAT_EQ(pressure_pa, filt.apply(pressure_pa, RATIO, t_ms));
        t_ms += DT_MS;
    }
}

TEST(AirspeedFilterTest, AcceptsSteadyPressure)
{
    AP_Airspeed_Filter filt;
    uint32_t t_ms = 1000;
    feed_constant(filt, 200.0f, 20, t_ms);
}

TEST(AirspeedFilterTest, AcceptsSmallNoiseAroundBaseline)
{
    AP_Airspeed_Filter filt;
    uint32_t t_ms = 1000;

    // Warm the IQR window on a stable baseline
    feed_constant(filt, 200.0f, 12, t_ms);

    // Within MIN_IQR_PA*m (= 10 Pa) of the median — should pass
    EXPECT_FLOAT_EQ(205.0f, filt.apply(205.0f, RATIO, t_ms));
    t_ms += DT_MS;
    EXPECT_FLOAT_EQ(195.0f, filt.apply(195.0f, RATIO, t_ms));
}

TEST(AirspeedFilterTest, HoldsZeroSpikeThenRecovers)
{
    AP_Airspeed_Filter filt;
    uint32_t t_ms = 1000;
    feed_constant(filt, 200.0f, 12, t_ms);

    // Single zero glitch: hold last good value
    EXPECT_FLOAT_EQ(200.0f, filt.apply(0.0f, RATIO, t_ms));
    t_ms += DT_MS;

    // Next good sample is accepted again
    EXPECT_FLOAT_EQ(200.0f, filt.apply(200.0f, RATIO, t_ms));
}

TEST(AirspeedFilterTest, ResyncAfterMaxHolds)
{
    AP_Airspeed_Filter filt;
    uint32_t t_ms = 1000;
    feed_constant(filt, 200.0f, 12, t_ms);

    // First MAX_HOLDS rejects keep the last good pressure
    for (uint8_t i = 0; i < MAX_HOLDS; i++) {
        EXPECT_FLOAT_EQ(200.0f, filt.apply(0.0f, RATIO, t_ms));
        t_ms += DT_MS;
    }

    // Next reject resyncs onto the new baseline
    EXPECT_FLOAT_EQ(0.0f, filt.apply(0.0f, RATIO, t_ms));
}

TEST(AirspeedFilterTest, ResetHoldsAllowsMoreHolds)
{
    AP_Airspeed_Filter filt;
    uint32_t t_ms = 1000;
    feed_constant(filt, 200.0f, 12, t_ms);

    EXPECT_FLOAT_EQ(200.0f, filt.apply(0.0f, RATIO, t_ms));
    t_ms += DT_MS;
    EXPECT_FLOAT_EQ(200.0f, filt.apply(0.0f, RATIO, t_ms));
    t_ms += DT_MS;

    filt.reset_holds();

    // Counter cleared — can hold MAX_HOLDS more times before resync
    for (uint8_t i = 0; i < MAX_HOLDS; i++) {
        EXPECT_FLOAT_EQ(200.0f, filt.apply(0.0f, RATIO, t_ms));
        t_ms += DT_MS;
    }
    EXPECT_FLOAT_EQ(0.0f, filt.apply(0.0f, RATIO, t_ms));
}

TEST(AirspeedFilterTest, RateOfChangeRejectsSuddenJump)
{
    AP_Airspeed_Filter filt;
    uint32_t t_ms = 1000;

    // Stay in warm-up so IQR is inactive; RoC alone must reject.
    // p=200 Pa, ratio=2 → 20 m/s. Jump to 0 in 0.1 s → 200 m/s² >> 15.
    EXPECT_FLOAT_EQ(200.0f, filt.apply(200.0f, RATIO, t_ms));
    t_ms += DT_MS;
    EXPECT_FLOAT_EQ(200.0f, filt.apply(0.0f, RATIO, t_ms));
}

TEST(AirspeedFilterTest, GradualChangeAccepted)
{
    AP_Airspeed_Filter filt;
    uint32_t t_ms = 1000;

    // Ramp airspeed by ~1 m/s per 0.1 s (10 m/s² < 15 m/s² limit)
    // aspd = sqrt(p * ratio) → p = aspd^2 / ratio
    float aspd = 10.0f;
    for (uint8_t i = 0; i < 15; i++) {
        const float pressure_pa = sq(aspd) / RATIO;
        EXPECT_NEAR(pressure_pa, filt.apply(pressure_pa, RATIO, t_ms), 1.0e-3);
        aspd += 1.0f;
        t_ms += DT_MS;
    }
}

TEST(AirspeedFilterTest, HeldSampleNotPushedIntoWindow)
{
    AP_Airspeed_Filter filt;
    uint32_t t_ms = 1000;
    feed_constant(filt, 200.0f, 12, t_ms);

    // Three held zeros must not poison the IQR window
    for (uint8_t i = 0; i < MAX_HOLDS; i++) {
        EXPECT_FLOAT_EQ(200.0f, filt.apply(0.0f, RATIO, t_ms));
        t_ms += DT_MS;
    }

    // Baseline still ~200 Pa, so a normal sample is accepted
    EXPECT_FLOAT_EQ(200.0f, filt.apply(200.0f, RATIO, t_ms));
}

TEST(AirspeedFilterTest, HoldsUpwardSpike)
{
    AP_Airspeed_Filter filt;
    uint32_t t_ms = 1000;
    feed_constant(filt, 200.0f, 12, t_ms);

    // Large upward glitch (e.g. blockage) should be held
    EXPECT_FLOAT_EQ(200.0f, filt.apply(2000.0f, RATIO, t_ms));
    t_ms += DT_MS;
    EXPECT_FLOAT_EQ(200.0f, filt.apply(200.0f, RATIO, t_ms));
}

TEST(AirspeedFilterTest, IqrRejectsWhenRateOfChangeWouldPass)
{
    AP_Airspeed_Filter filt;
    uint32_t t_ms = 1000;
    feed_constant(filt, 200.0f, 12, t_ms);

    // Spread the zero glitch over 2 s so |Δairspeed|/dt ≈ 10 m/s² < 15.
    // RoC alone would accept; IQR must still reject and hold.
    EXPECT_FLOAT_EQ(200.0f, filt.apply(0.0f, RATIO, t_ms + 2000));
}

TEST(AirspeedFilterTest, WindowWrapContinuesFiltering)
{
    AP_Airspeed_Filter filt;
    uint32_t t_ms = 1000;

    // Fill well past WINDOW_SIZE (20) so the ring wraps
    feed_constant(filt, 150.0f, 25, t_ms);

    EXPECT_FLOAT_EQ(150.0f, filt.apply(0.0f, RATIO, t_ms));
    t_ms += DT_MS;
    EXPECT_FLOAT_EQ(150.0f, filt.apply(150.0f, RATIO, t_ms));
}

AP_GTEST_MAIN()
