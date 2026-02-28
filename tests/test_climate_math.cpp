#include "catch_amalgamated.hpp"
#include "climate/climate_math.h"

#include <cmath>
#include <limits>

using namespace esphome::waterfurnace_aurora;

// Helper: Fahrenheit to Celsius (matches ESPHome helpers.h)
static constexpr float f2c(float f) { return (f - 32.0f) / 1.8f; }

// ============================================================================
// float_changed — dead-band comparison
// ============================================================================

TEST_CASE("float_changed dead-band comparison", "[climate][math]") {
  const float EPS = 0.05f;

  SECTION("both NaN returns false") {
    REQUIRE_FALSE(float_changed(NAN, NAN, EPS));
  }

  SECTION("first NaN, second real returns true") {
    REQUIRE(float_changed(NAN, 20.0f, EPS));
  }

  SECTION("first real, second NaN returns true") {
    REQUIRE(float_changed(20.0f, NAN, EPS));
  }

  SECTION("identical values returns false") {
    REQUIRE_FALSE(float_changed(20.0f, 20.0f, EPS));
  }

  SECTION("difference within epsilon returns false") {
    // |0.04| < 0.05
    REQUIRE_FALSE(float_changed(20.0f, 20.04f, EPS));
    REQUIRE_FALSE(float_changed(20.04f, 20.0f, EPS));
  }

  SECTION("difference at epsilon boundary returns false") {
    // |0.05| is NOT > 0.05 (uses strict greater-than)
    REQUIRE_FALSE(float_changed(20.0f, 20.05f, EPS));
    REQUIRE_FALSE(float_changed(20.05f, 20.0f, EPS));
  }

  SECTION("difference just beyond epsilon returns true") {
    REQUIRE(float_changed(20.0f, 20.051f, EPS));
    REQUIRE(float_changed(20.051f, 20.0f, EPS));
  }

  SECTION("large difference returns true") {
    REQUIRE(float_changed(20.0f, 25.0f, EPS));
    REQUIRE(float_changed(25.0f, 20.0f, EPS));
  }

  SECTION("negative values within epsilon returns false") {
    REQUIRE_FALSE(float_changed(-5.0f, -5.03f, EPS));
    REQUIRE_FALSE(float_changed(-5.03f, -5.0f, EPS));
  }

  SECTION("negative values beyond epsilon returns true") {
    REQUIRE(float_changed(-5.0f, -5.1f, EPS));
  }

  SECTION("works with custom epsilon for humidity") {
    // Humidity uses 0.5 epsilon
    const float HUM_EPS = 0.5f;
    REQUIRE_FALSE(float_changed(50.0f, 50.4f, HUM_EPS));
    REQUIRE(float_changed(50.0f, 50.6f, HUM_EPS));
  }

  SECTION("zero epsilon behaves like exact comparison") {
    REQUIRE_FALSE(float_changed(1.0f, 1.0f, 0.0f));
    // Any nonzero difference triggers with eps=0
    REQUIRE(float_changed(1.0f, 1.0f + std::numeric_limits<float>::epsilon(), 0.0f));
  }
}

// ============================================================================
// apply_ema — exponential moving average
// ============================================================================

TEST_CASE("apply_ema exponential moving average", "[climate][math]") {
  const float ALPHA = 0.3f;

  SECTION("first call seeds state to raw value") {
    float state = NAN;
    float result = apply_ema(70.0f, state, ALPHA);
    REQUIRE(result == Catch::Approx(70.0f));
    REQUIRE(state == Catch::Approx(70.0f));
  }

  SECTION("NaN raw before seed keeps state NaN") {
    float state = NAN;
    float result = apply_ema(NAN, state, ALPHA);
    REQUIRE(std::isnan(result));
    REQUIRE(std::isnan(state));
  }

  SECTION("NaN raw after seed preserves existing state") {
    float state = NAN;
    apply_ema(70.0f, state, ALPHA);  // seed
    float result = apply_ema(NAN, state, ALPHA);
    REQUIRE(result == Catch::Approx(70.0f));
    REQUIRE(state == Catch::Approx(70.0f));
  }

  SECTION("constant input converges to that value") {
    float state = NAN;
    for (int i = 0; i < 20; i++) {
      apply_ema(72.5f, state, ALPHA);
    }
    REQUIRE(state == Catch::Approx(72.5f));
  }

  SECTION("step change moves gradually toward new value") {
    float state = NAN;
    apply_ema(70.0f, state, ALPHA);  // seed at 70.0

    // After one sample of 71.0:
    // ema = 0.3 * 71.0 + 0.7 * 70.0 = 21.3 + 49.0 = 70.3
    float result = apply_ema(71.0f, state, ALPHA);
    REQUIRE(result == Catch::Approx(70.3f));

    // After second sample of 71.0:
    // ema = 0.3 * 71.0 + 0.7 * 70.3 = 21.3 + 49.21 = 70.51
    result = apply_ema(71.0f, state, ALPHA);
    REQUIRE(result == Catch::Approx(70.51f));
  }

  SECTION("alpha=1.0 gives no smoothing (always equals raw)") {
    float state = NAN;
    apply_ema(70.0f, state, 1.0f);
    REQUIRE(state == Catch::Approx(70.0f));

    float result = apply_ema(75.0f, state, 1.0f);
    REQUIRE(result == Catch::Approx(75.0f));

    result = apply_ema(60.0f, state, 1.0f);
    REQUIRE(result == Catch::Approx(60.0f));
  }

  SECTION("alpha=0.0 freezes at seed value") {
    float state = NAN;
    apply_ema(70.0f, state, 0.0f);  // seed at 70.0

    // Any subsequent value is ignored
    float result = apply_ema(99.0f, state, 0.0f);
    REQUIRE(result == Catch::Approx(70.0f));

    result = apply_ema(0.0f, state, 0.0f);
    REQUIRE(result == Catch::Approx(70.0f));
  }

  SECTION("specific numeric check: 20 samples with alpha=0.3") {
    // Verify convergence within 5% after ~10 samples
    float state = NAN;
    apply_ema(70.0f, state, ALPHA);  // seed
    for (int i = 0; i < 10; i++) {
      apply_ema(71.0f, state, ALPHA);
    }
    // After 10 steps toward 71.0 from 70.0:
    // Remaining gap = 1.0 * (1 - 0.3)^10 = 1.0 * 0.7^10 ≈ 0.028
    // state ≈ 71.0 - 0.028 ≈ 70.972
    REQUIRE(state == Catch::Approx(71.0f).margin(0.03f));
  }
}

// ============================================================================
// Integration: EMA + dead-band together suppress jitter
// ============================================================================

TEST_CASE("EMA + dead-band integration", "[climate][math]") {
  const float ALPHA = 0.3f;
  const float TEMP_EPSILON = 0.05f;  // °C

  SECTION("0.1 degF jitter is suppressed after EMA smoothing") {
    // Simulate zone 3 thermostat oscillating between 72.4°F and 72.5°F
    // every 5 seconds.  After the EMA settles, the Celsius-domain
    // variation should be small enough for the dead-band to reject.
    const float lo_f = 72.4f;
    const float hi_f = 72.5f;
    const float lo_c = f2c(lo_f);  // ≈ 22.444°C
    const float hi_c = f2c(hi_f);  // ≈ 22.500°C

    float ema_state = NAN;
    float last_published = NAN;
    int publish_count = 0;

    // Run 40 cycles (200 s at 5 s/cycle) of alternating lo/hi
    for (int i = 0; i < 40; i++) {
      float raw_c = (i % 2 == 0) ? lo_c : hi_c;
      float smoothed = apply_ema(raw_c, ema_state, ALPHA);

      if (std::isnan(last_published) ||
          float_changed(smoothed, last_published, TEMP_EPSILON)) {
        last_published = smoothed;
        publish_count++;
      }
    }

    // Should publish once (initial seed) and then the jitter should be
    // absorbed by the EMA so that subsequent values fall within epsilon.
    // Allow up to 2 publishes to account for the first couple of cycles
    // while the EMA is still settling.
    REQUIRE(publish_count <= 3);
  }

  SECTION("real 1 degF change still publishes through EMA") {
    // Start stable at 70.0°F, then step to 71.0°F.  Verify that
    // publish_state eventually fires (the EMA doesn't suppress real changes).
    float ema_state = NAN;
    float last_published = NAN;

    // Settle at 70.0°F
    for (int i = 0; i < 10; i++) {
      float smoothed = apply_ema(f2c(70.0f), ema_state, ALPHA);
      if (std::isnan(last_published) ||
          float_changed(smoothed, last_published, TEMP_EPSILON)) {
        last_published = smoothed;
      }
    }
    float settled = last_published;

    // Step to 71.0°F — should publish within a few cycles
    bool published_new = false;
    for (int i = 0; i < 10; i++) {
      float smoothed = apply_ema(f2c(71.0f), ema_state, ALPHA);
      if (float_changed(smoothed, last_published, TEMP_EPSILON)) {
        published_new = true;
        last_published = smoothed;
        break;
      }
    }
    REQUIRE(published_new);
    // The new published value should be between the old and new temps
    REQUIRE(last_published > settled);
    REQUIRE(last_published < f2c(71.0f));
  }

  SECTION("humidity dead-band with larger epsilon") {
    // Humidity uses 0.5% epsilon.  Simulate 50.0 / 50.4 oscillation.
    const float HUM_EPS = 0.5f;
    float last = 50.0f;

    // 50.4 is within 0.5 dead-band
    REQUIRE_FALSE(float_changed(50.4f, last, HUM_EPS));

    // 50.6 exceeds dead-band
    REQUIRE(float_changed(50.6f, last, HUM_EPS));
  }
}
