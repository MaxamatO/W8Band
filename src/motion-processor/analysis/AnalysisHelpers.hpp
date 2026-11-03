#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

namespace w8band::Motion::Analysis
{

/// @brief Internal three-dimensional vector used by the processing pipeline.
struct Vec3
{
    /// @brief X-axis component.
    float x = 0.0f;

    /// @brief Y-axis component.
    float y = 0.0f;

    /// @brief Z-axis component.
    float z = 0.0f;
};

/// @brief Reads one component of an internal three-dimensional vector.
/// @param[in] rVector Vector whose component is requested.
/// @param[in] axis Component index: 0 for X, 1 for Y and 2 for Z.
/// @return Value of the requested vector component.
float GetComponent(const Vec3 &rVector, std::size_t axis);

/// @brief Replaces one component of an internal three-dimensional vector.
/// @param[in,out] rVector Vector whose component is replaced.
/// @param[in] axis Component index: 0 for X, 1 for Y and 2 for Z.
/// @param[in] value New component value.
void SetComponent(Vec3 &rVector, std::size_t axis, float value);

/// @brief Calculates the scalar product of two three-dimensional vectors.
/// @param[in] rLeft Left-hand vector.
/// @param[in] rRight Right-hand vector.
/// @return Scalar product of rLeft and rRight.
float Dot(const Vec3 &rLeft, const Vec3 &rRight);

/// @brief Calculates the right-handed cross product of two vectors.
/// @param[in] rLeft Left-hand vector.
/// @param[in] rRight Right-hand vector.
/// @return Vector perpendicular to rLeft and rRight.
Vec3 Cross(const Vec3 &rLeft, const Vec3 &rRight);

/// @brief Normalizes a vector in place to unit Euclidean length.
/// @param[in,out] rVector Vector to normalize; replaced by its unit vector.
/// @return True on success, or false for a zero-length/non-finite vector.
bool Normalize(Vec3 &rVector);

/// @brief Sorts timestamp deltas and calculates their statistical median.
/// @param[in,out] rValues Non-empty delta buffer; sorted in ascending order.
/// @return Median delta, including the mean of two middle values when even.
float MedianSortedDeltas(std::vector<uint32_t> &rValues);

/// @brief Calculates a component-wise median without fully sorting the data.
/// @param[in] rValues Non-empty vector collection to inspect.
/// @param[in] axis Component index: 0 for X, 1 for Y and 2 for Z.
/// @param[in,out] rScratch Reusable workspace overwritten with component values.
/// @return Median value of the selected component.
float MedianComponent(const std::vector<Vec3> &rValues, std::size_t axis,
                      std::vector<float> &rScratch);

/// @brief Calculates standard deviation using Welford's algorithm
/// @param[in] rValues Non-empty vector collection to inspect.
/// @return Population standard deviation for the X, Y and Z components.
Vec3 CalculatePopulationStandardDeviation(const std::vector<Vec3> &rValues);

/// @brief Applies a zero-phase second-order Butterworth low-pass filter.
/// @param[in,out] rValues Signal samples, replaced by filtered samples.
/// @param[in] sampleRateHz Effective sample rate derived from sensor timestamps.
/// @param[in] cutoffHz Butterworth cutoff frequency in hertz.
/// @return True when parameters are valid and filtering was completed.
bool ApplyZeroPhaseLowPass(std::vector<float> &rValues, float sampleRateHz,
                           float cutoffHz);

/// @brief Integrates one axis of a vector signal using the trapezoidal rule.
/// @param[in] rSignal Vector samples to integrate.
/// @param[in] rTime Sample timestamps in seconds, matching rSignal in size.
/// @param[in] axis Component index: 0 for X, 1 for Y and 2 for Z.
/// @param[out] rIntegral Integrated samples with the first value set to zero.
void IntegrateAxis(const std::vector<Vec3> &rSignal,
                   const std::vector<float> &rTime, std::size_t axis,
                   std::vector<float> &rIntegral);

/// @brief Integrates a scalar signal using the trapezoidal rule.
/// @param[in] rSignal Scalar samples to integrate.
/// @param[in] rTime Sample timestamps in seconds, matching rSignal in size.
/// @param[out] rIntegral Integrated samples with the first value set to zero.
void IntegrateScalar(const std::vector<float> &rSignal,
                     const std::vector<float> &rTime,
                     std::vector<float> &rIntegral);

/// @brief Removes a linear velocity error between two inclusive anchors.
/// @param[in,out] rVelocity Velocity samples corrected in place.
/// @param[in] rTime Sample timestamps in seconds, matching rVelocity in size.
/// @param[in] begin Index of the first sample included in the correction.
/// @param[in] end Index of the last sample included in the correction.
/// @param[in] beginVelocity Raw velocity error at the begin anchor.
/// @param[in] endVelocity Raw velocity error at the end anchor.
void RemoveLinearVelocityDrift(std::vector<float> &rVelocity,
                               const std::vector<float> &rTime,
                               std::size_t begin, std::size_t end,
                               float beginVelocity, float endVelocity);

/// @brief Integrates acceleration and enforces zero velocity at known anchors.
/// @param[in] rAcceleration Three-axis acceleration samples in m/s^2.
/// @param[in] rTime Sample timestamps in seconds, matching acceleration in size.
/// @param[in] axis Component index: 0 for X, 1 for Y and 2 for Z.
/// @param[in] middleAnchor Optional middle anchor index; zero disables it.
/// @param[out] rVelocity Drift-corrected velocity samples in m/s.
void IntegrateWithZeroVelocityAnchors(const std::vector<Vec3> &rAcceleration,
                                      const std::vector<float> &rTime,
                                      std::size_t axis, std::size_t middleAnchor,
                                      std::vector<float> &rVelocity);

/// @brief Rounds and saturates a floating-point value to signed 16-bit range.
/// @param[in] value Floating-point value to convert.
/// @return Nearest int16_t value, clamped instead of overflowing.
int16_t ToInt16(float value);

/// @brief Rounds and saturates a floating-point value to signed 32-bit range.
/// @param[in] value Floating-point value to convert.
/// @return Nearest int32_t value, clamped instead of overflowing.
int32_t ToInt32(float value);

/// @brief Converts a non-negative duration in seconds to rounded milliseconds.
/// @param[in] seconds Duration in seconds; non-positive values produce zero.
/// @return Duration in milliseconds, saturated to uint32_t range.
uint32_t ToMilliseconds(float seconds);

} // namespace w8band::Motion::Analysis
