#pragma once

#include "AnalysisHelpers.hpp"
#include "DataTypes.hpp"
#include "motion-processor/ProcessingDataTypes.hpp"

#include <cstddef>
#include <vector>

namespace w8band::Motion::Analysis
{

/// @brief Validates sample timestamps and builds the integration time axis.
ProcessingStatus ValidateAndBuildTimeline(
    const std::vector<data::SamplePacket> &rSamples, float lowPassCutoffHz,
    std::vector<float> &rTime, float &rSampleRateHz);

/// @brief Decodes sensor samples and rotates acceleration and gravity to world.
/// @return True when every quaternion is valid.
bool DecodeAndRotateSamples(const std::vector<data::SamplePacket> &rSamples,
                            const data::AccBiasVec3 &rAccelBias,
                            std::vector<Vec3> &rAcceleration,
                            std::vector<Vec3> &rGravity);

/// @brief Builds an orthonormal basis whose Z axis follows median gravity.
/// @return True when a valid basis can be constructed.
bool BuildGravityAlignedBasis(const std::vector<Vec3> &rGravity,
                              std::vector<float> &rScratch, Vec3 &rHorizontalX,
                              Vec3 &rHorizontalY, Vec3 &rUp);

/// @brief Filters world acceleration and expresses it in a gravity-aligned basis.
/// @return True when the filter parameters are valid.
bool FilterAndProjectAcceleration(std::vector<Vec3> &rAcceleration,
                                  float sampleRateHz, float cutoffHz,
                                  const Vec3 &rHorizontalX,
                                  const Vec3 &rHorizontalY, const Vec3 &rUp,
                                  std::vector<float> &rScratch);

/// @brief Finds the lowest preliminary vertical position away from edge artefacts.
std::size_t FindTurnaroundIndex(const std::vector<Vec3> &rAcceleration,
                                const std::vector<float> &rTime);

/// @brief Integrates acceleration into drift-corrected velocity and position.
void IntegrateMotion(const std::vector<Vec3> &rAcceleration,
                     const std::vector<float> &rTime,
                     std::size_t turnaroundIndex,
                     std::vector<float> &rScratch,
                     std::vector<Vec3> &rVelocity,
                     std::vector<Vec3> &rPosition);

/// @brief Projects horizontal displacement onto its principal movement axis.
void CalculateHorizontalPosition(const std::vector<Vec3> &rPosition,
                                 std::vector<float> &rHorizontalPosition);

/// @brief Calculates metrics and creates the reduced BLE-ready trajectory.
ProcessingResult BuildProcessingResult(
    const std::vector<Vec3> &rAcceleration,
    const std::vector<Vec3> &rVelocity,
    const std::vector<Vec3> &rPosition, const std::vector<float> &rTime,
    const std::vector<float> &rHorizontalPosition,
    std::size_t turnaroundIndex, float maxGravityStd,
    const ProcessingConfig &rConfig);

} // namespace w8band::Motion::Analysis
