#pragma once

#include "DataTypes.hpp"
#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace w8band::Motion
{
/// @brief Maximum number of trajectory points retained for transmission.
inline constexpr std::size_t MAX_TRAJECTORY_POINTS = 100;

/// @brief Quantized point in the dominant plane of barbell movement.
struct TrajectoryPoint
{
    /// @brief Horizontal displacement along the principal direction in mm.
    int16_t horizontalMm = 0;

    /// @brief Vertical displacement along gravity-defined up axis in mm.
    int16_t verticalMm = 0;
};

/// @brief Detailed outcome of batch motion processing.
enum class ProcessingStatus : uint8_t
{
    NotProcessed,
    Ok,
    NotEnoughSamples,
    InvalidTimestamp,
    MissingSamples,
    InvalidQuaternion,
    InvalidGravity,
    InvalidFilterConfiguration
};

/// @brief Runtime parameters that do not depend on the hardware platform.
struct ProcessingConfig
{
    /// @brief Low-pass cutoff used by zero-phase batch filtering in Hz.
    float lowPassCutoffHz = 10.0f;

    /// @brief Maximum accepted standard deviation of rotated gravity in mg.
    float maxWorldGravityStdMg = 20.0f;

    /// @brief Total moving mass in kg; zero means power is not calculated.
    float barbellMassKg = 0.0f;

    /// @brief Requested number of output points, clamped to [3, 100].
    uint16_t trajectoryPointCount = MAX_TRAJECTORY_POINTS;
};

/// @brief BLE-ready trajectory and metrics calculated from one repetition.
struct ProcessingResult
{
    /// @brief Uniformly reduced trajectory with the turnaround preserved.
    std::array<TrajectoryPoint, MAX_TRAJECTORY_POINTS> trajectory{};

    /// @brief Number of valid entries in trajectory.
    uint16_t trajectoryPointCount = 0;

    /// @brief Number of raw synchronized samples used by the processor.
    uint16_t rawSampleCount = 0;

    /// @brief Turnaround index in the raw sample buffer.
    uint16_t turnaroundIndex = 0;

    /// @brief Turnaround index in the reduced trajectory array.
    uint16_t turnaroundTrajectoryIndex = 0;

    /// @brief Duration of the complete captured buffer in ms.
    uint32_t durationMs = 0;

    /// @brief Duration from detected movement start to turnaround in ms.
    uint32_t eccentricDurationMs = 0;

    /// @brief Duration from turnaround to highest post-turn point in ms.
    uint32_t concentricDurationMs = 0;

    /// @brief Full vertical range of motion in mm.
    int16_t verticalRomMm = 0;

    /// @brief Maximum upward velocity after turnaround in mm/s.
    int16_t maxVelocityMmPerSec = 0;

    /// @brief Mean upward velocity from turnaround to top position in mm/s.
    int16_t meanConcentricVelocityMmPerSec = 0;

    /// @brief Maximum concentric mechanical power in mW.
    int32_t maxPowerMilliwatts = 0;

    /// @brief Time-averaged concentric mechanical power in mW.
    int32_t meanConcentricPowerMilliwatts = 0;

    /// @brief Maximum standard deviation of rotated gravity axes in mg.
    uint16_t maxWorldGravityStdMg = 0;

    /// @brief True when barbellMassKg was positive and power was calculated.
    bool powerValid = false;

    /// @brief True only when processing completed without validation errors.
    bool valid = false;

    /// @brief Machine-readable processing outcome.
    ProcessingStatus status = ProcessingStatus::NotProcessed;
};

/// @brief Reconstructs a drift-corrected trajectory from one complete capture.
class MotionProcessor
{
public:
    /// @brief Runs the complete batch-processing pipeline.
    /// @param[in] rSamples Synchronized timestamp, acceleration, gravity and
    /// quaternion samples covering rest before and after one repetition.
    /// @param[in] rAccelBias Calibrated accelerometer bias in sensor-frame mg.
    /// @param[in] rConfig Filter, validation, mass and output configuration.
    /// @return Quantized trajectory, repetition metrics and processing status.
    static ProcessingResult Process(
        const std::vector<data::SamplePacket> &rSamples,
        const data::AccBiasVec3 &rAccelBias,
        const ProcessingConfig &rConfig = ProcessingConfig{});

    /// @brief Returns a stable human-readable status description.
    /// @param[in] status Status returned in ProcessingResult.
    /// @return Null-terminated status string.
    static const char *StatusToString(ProcessingStatus status);
};
} // namespace w8band::Motion
