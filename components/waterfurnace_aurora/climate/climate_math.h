#pragma once

// Pure math utilities for climate entity smoothing and deduplication.
// No ESPHome dependencies — only <cmath>. This keeps the functions
// trivially unit-testable without any mocking infrastructure.

#include <cmath>

namespace esphome {
namespace waterfurnace_aurora {

/// NaN-aware dead-band float comparison.
/// Returns true if the two values differ by more than epsilon.
/// Both-NaN is "unchanged"; one-NaN-one-real is always "changed".
inline bool float_changed(float a, float b, float epsilon) {
  if (std::isnan(a) && std::isnan(b))
    return false;
  if (std::isnan(a) || std::isnan(b))
    return true;
  return std::abs(a - b) > epsilon;
}

/// Exponential moving average (EMA).
///
/// @param raw   New sample value.
/// @param state EMA accumulator — passed by reference. Initialize to NAN.
/// @param alpha Smoothing factor in (0, 1]. Higher = less smoothing.
///              alpha=0.3 with 5 s polling gives ~12 s time constant.
/// @return      The updated EMA value (also written into state).
///
/// First call (state == NAN) seeds the accumulator to the raw value so
/// there is no startup lag.  NaN input is ignored — the existing state
/// is returned unchanged.
inline float apply_ema(float raw, float &state, float alpha) {
  if (std::isnan(raw))
    return state;  // bad sample — don't corrupt accumulator
  if (std::isnan(state)) {
    state = raw;   // first valid sample — seed
  } else {
    state = alpha * raw + (1.0f - alpha) * state;
  }
  return state;
}

}  // namespace waterfurnace_aurora
}  // namespace esphome
