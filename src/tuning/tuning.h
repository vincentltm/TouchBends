#pragma once

#include <cstdint>
#include <cstddef>

namespace bends {
namespace tuning {

// Physical touch & conductance thresholds (physical contact starts at floor_p = 0.04)
constexpr float kBridgeThreshold = 0.020f;
constexpr float kBridgeRelease   = 0.010f;
constexpr float kTouchThreshold  = 0.025f;
constexpr float kTouchRelease    = 0.015f;
constexpr float kSoloThreshold   = 0.025f;

// VCC thresholds
constexpr float kFresh   = 0.65f;
constexpr float kFragile = 0.45f;
constexpr float kBroken  = 0.28f;
constexpr float kFail    = 0.14f;
constexpr float kDying   = 0.06f;
constexpr float kAmbiguousBand = 0.015f;

// Delay memory architecture (Power-of-two 2^20 stereo frames = 8 MB in SDRAM)
constexpr size_t   kDelayFrames = 1048576;
constexpr uint32_t kDelayMask   = 1048575;
constexpr float    kMinDelaySamples = 14.0f;
constexpr uint16_t kXfadeSamples = 1680; // ~35ms at 48kHz (smooth infrasonic crossfade)
constexpr float    kKarplusMaxSamples = 2400.0f; // 50ms threshold between string and echo

// Feedback ceiling and master output safety limits
constexpr float kFeedbackCeiling = 1.15f;
constexpr float kMasterCeiling   = 0.89125f; // -1.0 dBFS true peak ceiling (safe for live & headphones)
constexpr float kMasterKnee      = 0.66834f; // -3.5 dBFS soft-knee start

} // namespace tuning
} // namespace bends
