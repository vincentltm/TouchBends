#pragma once

namespace bends {
namespace health {

// Rail voltage thresholds:
// 1.00 .. 0.65: Clean operating range
// 0.65 .. 0.45: HF rolloff, soft saturation
// 0.45 .. 0.28: Asymmetric bias, memory bit depth reduction
// 0.14 .. 0.28: Heavy clock sag, oscillator choking
// 0.00 .. 0.14: Brownout threshold
constexpr float kFresh   = 0.65f;
constexpr float kFragile = 0.45f;
constexpr float kTired   = kFragile;
constexpr float kBroken  = 0.28f;
constexpr float kSag     = kBroken;
constexpr float kFail    = 0.14f;

constexpr float kEdges[4] = { kFresh, kFragile, kBroken, kFail };
constexpr float kAmbiguousBand = 0.015f;

} // namespace health
} // namespace bends
