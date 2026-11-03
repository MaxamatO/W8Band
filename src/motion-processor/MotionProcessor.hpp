#pragma once

#include "DataTypes.hpp"
#include "ProcessingDataTypes.hpp"
#include <vector>

namespace w8band::Motion
{

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
