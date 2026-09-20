#include "MotionProcessor.hpp"
#include "helpers/ProcessingHelpers.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace w8band::Motion
{
namespace
{
constexpr std::size_t MIN_SAMPLE_COUNT = 20;
constexpr std::size_t FILTER_PAD_LENGTH = 9;
constexpr float MG_TO_MS2 = 9.80665f / 1000.0f;
constexpr float GRAVITY_MS2 = 9.80665f;
constexpr float PI = 3.14159265358979323846f;
constexpr float QUATERNION_MIN_NORM = 0.5f;
constexpr float MOTION_START_VELOCITY_MPS = -0.05f;

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
float GetComponent(const Vec3 &rVector, std::size_t axis)
{
    if(axis == 0)
    {
        return rVector.x;
    }
    if(axis == 1)
    {
        return rVector.y;
    }
    return rVector.z;
}

/// @brief Replaces one component of an internal three-dimensional vector.
/// @param[in,out] rVector Vector whose component is replaced.
/// @param[in] axis Component index: 0 for X, 1 for Y and 2 for Z.
/// @param[in] value New component value.
void SetComponent(Vec3 &rVector, std::size_t axis, float value)
{
    if(axis == 0)
    {
        rVector.x = value;
    } else if(axis == 1)
    {
        rVector.y = value;
    } else
    {
        rVector.z = value;
    }
}

/// @brief Calculates the scalar product of two three-dimensional vectors.
/// @param[in] rLeft Left-hand vector.
/// @param[in] rRight Right-hand vector.
/// @return Scalar product of rLeft and rRight.
float Dot(const Vec3 &rLeft, const Vec3 &rRight)
{ return rLeft.x * rRight.x + rLeft.y * rRight.y + rLeft.z * rRight.z; }

/// @brief Calculates the right-handed cross product of two vectors.
/// @param[in] rLeft Left-hand vector.
/// @param[in] rRight Right-hand vector.
/// @return Vector perpendicular to rLeft and rRight.
Vec3 Cross(const Vec3 &rLeft, const Vec3 &rRight)
{
    return {rLeft.y * rRight.z - rLeft.z * rRight.y,
            rLeft.z * rRight.x - rLeft.x * rRight.z,
            rLeft.x * rRight.y - rLeft.y * rRight.x};
}

/// @brief Normalizes a vector in place to unit Euclidean length.
/// @param[in,out] rVector Vector to normalize; replaced by its unit vector.
/// @return True on success, or false for a zero-length/non-finite vector.
bool Normalize(Vec3 &rVector)
{
    const float norm = std::sqrt(Dot(rVector, rVector));
    if(!std::isfinite(norm) || norm <= std::numeric_limits<float>::epsilon())
    {
        return false;
    }
    rVector.x /= norm;
    rVector.y /= norm;
    rVector.z /= norm;
    return true;
}

/// @brief Creates a value-initialized unsuccessful processing result.
/// @param[in] status Failure status reported to the caller.
/// @param[in] sampleCount Number of supplied raw samples; clamped to uint16_t.
/// @return Invalid processing result containing status and raw sample count.
ProcessingResult Failure(ProcessingStatus status, std::size_t sampleCount)
{
    ProcessingResult result{};
    result.status = status;
    result.rawSampleCount = static_cast<uint16_t>(std::min(
        sampleCount,
        static_cast<std::size_t>(std::numeric_limits<uint16_t>::max())));
    return result;
}

/// @brief Sorts timestamp deltas and calculates their statistical median.
/// @param[in,out] rValues Non-empty delta buffer; sorted in ascending order.
/// @return Median delta, including the mean of two middle values when even.
float MedianSortedDeltas(std::vector<uint32_t> &rValues)
{
    std::sort(rValues.begin(), rValues.end());
    const std::size_t middle = rValues.size() / 2;
    if((rValues.size() % 2U) != 0U)
    {
        return static_cast<float>(rValues[middle]);
    }
    return 0.5f
           * (static_cast<float>(rValues[middle - 1])
              + static_cast<float>(rValues[middle]));
}

/// @brief Calculates a component-wise median without fully sorting the data.
/// @param[in] rValues Non-empty vector collection to inspect.
/// @param[in] axis Component index: 0 for X, 1 for Y and 2 for Z.
/// @param[in,out] rScratch Reusable workspace overwritten with component values.
/// @return Median value of the selected component.
float MedianComponent(const std::vector<Vec3> &rValues, std::size_t axis,
                      std::vector<float> &rScratch)
{
    rScratch.resize(rValues.size());
    for(std::size_t i = 0; i < rValues.size(); ++i)
    {
        rScratch[i] = GetComponent(rValues[i], axis);
    }

    const std::size_t middle = rScratch.size() / 2;
    std::nth_element(rScratch.begin(), rScratch.begin() + middle,
                     rScratch.end());
    const float upper = rScratch[middle];
    if((rScratch.size() % 2U) != 0U)
    {
        return upper;
    }
    const float lower
        = *std::max_element(rScratch.begin(), rScratch.begin() + middle);
    return 0.5f * (lower + upper);
}

/// @brief Applies a zero-phase second-order Butterworth low-pass filter.
/// @param[in,out] rValues Signal samples, replaced by filtered samples.
/// @param[in] sampleRateHz Effective sample rate derived from sensor timestamps.
/// @param[in] cutoffHz Butterworth cutoff frequency in hertz.
/// @return True when parameters are valid and filtering was completed.
bool ApplyZeroPhaseLowPass(std::vector<float> &rValues, float sampleRateHz,
                           float cutoffHz)
{
    if(rValues.size() <= FILTER_PAD_LENGTH || cutoffHz <= 0.0f
       || cutoffHz >= sampleRateHz * 0.5f)
    {
        return false;
    }

    // Bilinear-transform coefficients of a second-order Butterworth low-pass.
    const float k = std::tan(PI * cutoffHz / sampleRateHz);
    const float norm = 1.0f / (1.0f + std::sqrt(2.0f) * k + k * k);
    const float b0 = k * k * norm;
    const float b1 = 2.0f * b0;
    const float b2 = b0;
    const float a1 = 2.0f * (k * k - 1.0f) * norm;
    const float a2 = (1.0f - std::sqrt(2.0f) * k + k * k) * norm;

    // Match scipy.signal.sosfiltfilt for one SOS: odd extension, pad length 9
    // and steady-state initial conditions for both filtering directions.
    std::vector<float> extended(rValues.size() + 2U * FILTER_PAD_LENGTH);
    for(std::size_t i = 0; i < FILTER_PAD_LENGTH; ++i)
    {
        extended[i] = 2.0f * rValues.front() - rValues[FILTER_PAD_LENGTH - i];
    }
    std::copy(rValues.begin(), rValues.end(),
              extended.begin() + FILTER_PAD_LENGTH);
    for(std::size_t i = 0; i < FILTER_PAD_LENGTH; ++i)
    {
        extended[FILTER_PAD_LENGTH + rValues.size() + i]
            = 2.0f * rValues.back() - rValues[rValues.size() - 2U - i];
    }

    const float zi1 = 1.0f - b0;
    const float zi2 = b2 - a2;
    float state1 = zi1 * extended.front();
    float state2 = zi2 * extended.front();
    for(float &rValue : extended)
    {
        const float input = rValue;
        const float output = b0 * input + state1;
        state1 = b1 * input - a1 * output + state2;
        state2 = b2 * input - a2 * output;
        rValue = output;
    }

    state1 = zi1 * extended.back();
    state2 = zi2 * extended.back();
    for(std::size_t i = extended.size(); i-- > 0;)
    {
        const float input = extended[i];
        const float output = b0 * input + state1;
        state1 = b1 * input - a1 * output + state2;
        state2 = b2 * input - a2 * output;
        extended[i] = output;
    }

    std::copy(extended.begin() + FILTER_PAD_LENGTH,
              extended.begin() + FILTER_PAD_LENGTH + rValues.size(),
              rValues.begin());
    return true;
}

/// @brief Integrates one axis of a vector signal using the trapezoidal rule.
/// @param[in] rSignal Vector samples to integrate.
/// @param[in] rTime Sample timestamps in seconds, matching rSignal in size.
/// @param[in] axis Component index: 0 for X, 1 for Y and 2 for Z.
/// @param[out] rIntegral Integrated samples with the first value set to zero.
void IntegrateAxis(const std::vector<Vec3> &rSignal,
                   const std::vector<float> &rTime, std::size_t axis,
                   std::vector<float> &rIntegral)
{
    rIntegral.assign(rSignal.size(), 0.0f);
    for(std::size_t i = 1; i < rSignal.size(); ++i)
    {
        const float dt = rTime[i] - rTime[i - 1];
        rIntegral[i] = rIntegral[i - 1]
                       + 0.5f
                             * (GetComponent(rSignal[i - 1], axis)
                                + GetComponent(rSignal[i], axis))
                             * dt;
    }
}

/// @brief Integrates a scalar signal using the trapezoidal rule.
/// @param[in] rSignal Scalar samples to integrate.
/// @param[in] rTime Sample timestamps in seconds, matching rSignal in size.
/// @param[out] rIntegral Integrated samples with the first value set to zero.
void IntegrateScalar(const std::vector<float> &rSignal,
                     const std::vector<float> &rTime,
                     std::vector<float> &rIntegral)
{
    rIntegral.assign(rSignal.size(), 0.0f);
    for(std::size_t i = 1; i < rSignal.size(); ++i)
    {
        const float dt = rTime[i] - rTime[i - 1];
        rIntegral[i]
            = rIntegral[i - 1] + 0.5f * (rSignal[i - 1] + rSignal[i]) * dt;
    }
}

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
                               float beginVelocity, float endVelocity)
{
    const float beginTime = rTime[begin];
    const float duration = rTime[end] - beginTime;
    const float velocityChange
        = endVelocity - beginVelocity; // This should be our drift b_v
    for(std::size_t i = begin; i <= end; ++i)
    {
        const float fraction
            = duration > 0.0f ? (rTime[i] - beginTime) / duration : 0.0f;
        const float correction = beginVelocity + fraction * velocityChange;
        rVelocity[i] -= correction;
    } // Begin vlocity and End velocity gets zeroed out.
}

/// @brief Integrates acceleration and enforces zero velocity at known anchors.
/// @param[in] rAcceleration Three-axis acceleration samples in m/s^2.
/// @param[in] rTime Sample timestamps in seconds, matching acceleration in size.
/// @param[in] axis Component index: 0 for X, 1 for Y and 2 for Z.
/// @param[in] middleAnchor Optional middle anchor index; zero disables it.
/// @param[out] rVelocity Drift-corrected velocity samples in m/s.
void IntegrateWithZeroVelocityAnchors(const std::vector<Vec3> &rAcceleration,
                                      const std::vector<float> &rTime,
                                      std::size_t axis, std::size_t middleAnchor,
                                      std::vector<float> &rVelocity)
{
    IntegrateAxis(rAcceleration, rTime, axis,
                  rVelocity); // Obtain velocity array
    if(middleAnchor > 0U && middleAnchor < rVelocity.size() - 1U)
    {
        // Keep all three values from the uncorrected integral. Correcting the
        // first segment mutates v[middle], while the second segment still has
        // to interpolate from the original middle-anchor error.
        const float beginVelocity = rVelocity.front();
        const float middleVelocity = rVelocity[middleAnchor];
        const float endVelocity = rVelocity.back();
        RemoveLinearVelocityDrift(rVelocity, rTime, 0U, middleAnchor,
                                  beginVelocity, middleVelocity);
        rVelocity[middleAnchor] = middleVelocity;
        RemoveLinearVelocityDrift(rVelocity, rTime, middleAnchor,
                                  rVelocity.size() - 1U, middleVelocity,
                                  endVelocity);
    } else
    {
        RemoveLinearVelocityDrift(rVelocity, rTime, 0U, rVelocity.size() - 1U,
                                  rVelocity.front(), rVelocity.back());
    }
}

/// @brief Rounds and saturates a floating-point value to signed 16-bit range.
/// @param[in] value Floating-point value to convert.
/// @return Nearest int16_t value, clamped instead of overflowing.
int16_t ToInt16(float value)
{
    const float minimum
        = static_cast<float>(std::numeric_limits<int16_t>::min());
    const float maximum
        = static_cast<float>(std::numeric_limits<int16_t>::max());
    return static_cast<int16_t>(
        std::lround(std::max(minimum, std::min(value, maximum))));
}

/// @brief Rounds and saturates a floating-point value to signed 32-bit range.
/// @param[in] value Floating-point value to convert.
/// @return Nearest int32_t value, clamped instead of overflowing.
int32_t ToInt32(float value)
{
    const double minimum
        = static_cast<double>(std::numeric_limits<int32_t>::min());
    const double maximum
        = static_cast<double>(std::numeric_limits<int32_t>::max());
    const double clamped
        = std::max(minimum, std::min(static_cast<double>(value), maximum));
    return static_cast<int32_t>(std::llround(clamped));
}

/// @brief Converts a non-negative duration in seconds to rounded milliseconds.
/// @param[in] seconds Duration in seconds; non-positive values produce zero.
/// @return Duration in milliseconds, saturated to uint32_t range.
uint32_t ToMilliseconds(float seconds)
{
    if(seconds <= 0.0f)
    {
        return 0U;
    }
    const double milliseconds = static_cast<double>(seconds) * 1000.0;
    return static_cast<uint32_t>(
        std::min(milliseconds + 0.5,
                 static_cast<double>(std::numeric_limits<uint32_t>::max())));
}
} // namespace

ProcessingResult
MotionProcessor::Process(const std::vector<data::SamplePacket> &rSamples,
                         const data::AccBiasVec3 &rAccelBias,
                         const ProcessingConfig &rConfig)
{
    const std::size_t sampleCount = rSamples.size();
    if(sampleCount < MIN_SAMPLE_COUNT)
    {
        return Failure(ProcessingStatus::NotEnoughSamples, sampleCount);
    }

    std::vector<uint32_t> deltaTicks(sampleCount - 1U);
    for(std::size_t i = 1; i < sampleCount; ++i)
    {
        deltaTicks[i - 1U]
            = rSamples[i].timestamp_ticks - rSamples[i - 1U].timestamp_ticks;
        if(deltaTicks[i - 1U] == 0U)
        {
            return Failure(ProcessingStatus::InvalidTimestamp, sampleCount);
        }
    }

    const uint32_t maxDeltaTicks
        = *std::max_element(deltaTicks.begin(), deltaTicks.end());
    const float medianDeltaTicks = MedianSortedDeltas(deltaTicks);
    if(static_cast<float>(maxDeltaTicks) > 1.5f * medianDeltaTicks)
    {
        return Failure(ProcessingStatus::MissingSamples, sampleCount);
    }

    const float medianDt = medianDeltaTicks * LSM_TIMESTAMP_TICK_SECONDS;
    const float sampleRateHz = 1.0f / medianDt;
    if(rConfig.lowPassCutoffHz <= 0.0f
       || rConfig.lowPassCutoffHz >= sampleRateHz * 0.5f)
    {
        return Failure(ProcessingStatus::InvalidFilterConfiguration,
                       sampleCount);
    }
    // Map real dt for integrating, instead of assuming constant 1/120Hz time
    std::vector<float> time(sampleCount, 0.0f);
    for(std::size_t i = 1; i < sampleCount; ++i)
    {
        const uint32_t elapsedTicks
            = rSamples[i].timestamp_ticks - rSamples[i - 1U].timestamp_ticks;
        time[i]
            = time[i - 1U]
              + static_cast<float>(elapsedTicks) * LSM_TIMESTAMP_TICK_SECONDS;
    }

    std::vector<Vec3> acceleration(sampleCount);
    std::vector<Vec3> gravity(sampleCount);
    Vec3 gravityMean{};
    Vec3 gravityM2{};
    helpers::Processing::Quat previousQuat{1.0f, 0.0f, 0.0f, 0.0f};

    for(std::size_t i = 0; i < sampleCount; ++i)
    {
        const data::SamplePacket &rPacket = rSamples[i];
        const float rawQw = rPacket.q[0] / Q14_SCALE;
        const float rawQx = rPacket.q[1] / Q14_SCALE;
        const float rawQy = rPacket.q[2] / Q14_SCALE;
        const float rawQz = rPacket.q[3] / Q14_SCALE;
        const float quaternionNorm = std::sqrt(
            rawQw * rawQw + rawQx * rawQx + rawQy * rawQy + rawQz * rawQz);
        if(!std::isfinite(quaternionNorm) || quaternionNorm < QUATERNION_MIN_NORM)
        {
            return Failure(ProcessingStatus::InvalidQuaternion, sampleCount);
        }

        const helpers::Processing::Quat q
            = helpers::Processing::DecodeQuaternion(rPacket, previousQuat);
        // Obtain linear accel in body frame by subtracting bias and gravity vector
        const float linearBodyX = static_cast<float>(rPacket.a[0])
                                  - static_cast<float>(rAccelBias.axBias)
                                  - static_cast<float>(rPacket.gv[0]);
        const float linearBodyY = static_cast<float>(rPacket.a[1])
                                  - static_cast<float>(rAccelBias.ayBias)
                                  - static_cast<float>(rPacket.gv[1]);
        const float linearBodyZ = static_cast<float>(rPacket.a[2])
                                  - static_cast<float>(rAccelBias.azBias)
                                  - static_cast<float>(rPacket.gv[2]);

        // Rotate both accel and gravity vector to world
        helpers::Processing::RotateBodyToWorld(
            q.w, q.x, q.y, q.z, linearBodyX, linearBodyY, linearBodyZ,
            acceleration[i].x, acceleration[i].y, acceleration[i].z);
        helpers::Processing::RotateBodyToWorld(
            q.w, q.x, q.y, q.z, static_cast<float>(rPacket.gv[0]),
            static_cast<float>(rPacket.gv[1]), static_cast<float>(rPacket.gv[2]),
            gravity[i].x, gravity[i].y, gravity[i].z);

        // Welfords standard deviation in single pass
        const float count = static_cast<float>(i + 1U);
        const Vec3 delta{gravity[i].x - gravityMean.x,
                         gravity[i].y - gravityMean.y,
                         gravity[i].z - gravityMean.z};
        gravityMean.x += delta.x / count;
        gravityMean.y += delta.y / count;
        gravityMean.z += delta.z / count;
        const Vec3 deltaAfter{gravity[i].x - gravityMean.x,
                              gravity[i].y - gravityMean.y,
                              gravity[i].z - gravityMean.z};
        gravityM2.x += delta.x * deltaAfter.x;
        gravityM2.y += delta.y * deltaAfter.y;
        gravityM2.z += delta.z * deltaAfter.z;
    }

    // Obtain standard deviation and max standard deviation
    const Vec3 gravityStd{
        std::sqrt(gravityM2.x / static_cast<float>(sampleCount)),
        std::sqrt(gravityM2.y / static_cast<float>(sampleCount)),
        std::sqrt(gravityM2.z / static_cast<float>(sampleCount))};
    const float maxGravityStd
        = std::max(gravityStd.x, std::max(gravityStd.y, gravityStd.z));

    // Drop gravity's standard deviation greated than maxWorldGravityStdMg -
    // something wen wrong in calculating it
    if(!std::isfinite(maxGravityStd)
       || maxGravityStd > rConfig.maxWorldGravityStdMg)
    {
        return Failure(ProcessingStatus::InvalidGravity, sampleCount);
    }

    std::vector<float> scratch;
    scratch.reserve(sampleCount);

    // Z axis
    Vec3 up{MedianComponent(gravity, 0U, scratch),
            MedianComponent(gravity, 1U, scratch),
            MedianComponent(gravity, 2U, scratch)};

    // Z axis after normalize has length 1
    if(!Normalize(up))
    {
        return Failure(ProcessingStatus::InvalidGravity, sampleCount);
    }

    // Build reference axis
    // If reference vector {1, 0, 0} was parallel to vertical axis, Dot product
    // on ref . e_up, would give us result > 0.9
    Vec3 reference{1.0f, 0.0f, 0.0f};
    if(std::fabs(Dot(reference, up)) > 0.9f)
    {
        reference = {0.0f, 1.0f, 0.0f};
    }

    // Build horizontal axis with Gram-Schmidt, to get orthogonal axis,
    // We can achieve it because we already have our Up vector as a vertical
    // axis, and since we built it using gravity vector rotated to world, we
    // can assume it is pointing up.
    // Then we want to build (x,y) axis around that Z axis, to do that, we use Gram-Schmidts algorithm.
    const float projection = Dot(reference, up);
    Vec3 horizontalX{reference.x - projection * up.x, // hx = r - (r . e_up)e_up
                     reference.y - projection * up.y,
                     reference.z - projection * up.z};
    if(!Normalize(horizontalX))
    {
        return Failure(ProcessingStatus::InvalidGravity, sampleCount);
    }
    Vec3 horizontalY = Cross(up, horizontalX);
    if(!Normalize(horizontalY))
    {
        return Failure(ProcessingStatus::InvalidGravity, sampleCount);
    }

    for(std::size_t axis = 0; axis < 3U; ++axis)
    {
        scratch.resize(sampleCount);
        for(std::size_t i = 0; i < sampleCount; ++i)
        {
            scratch[i] = GetComponent(acceleration[i], axis);
        }
        if(!ApplyZeroPhaseLowPass(scratch, sampleRateHz, rConfig.lowPassCutoffHz))
        {
            return Failure(ProcessingStatus::InvalidFilterConfiguration,
                           sampleCount);
        }
        for(std::size_t i = 0; i < sampleCount; ++i)
        {
            SetComponent(acceleration[i], axis, scratch[i] * MG_TO_MS2);
        }
    }

    // Express filtered acceleration in the gravity-aligned basis.
    for(Vec3 &rSample : acceleration)
    {
        const Vec3 world = rSample;
        rSample = {Dot(world, horizontalX), Dot(world, horizontalY),
                   Dot(world, up)};
    }

    std::vector<float> preliminaryVelocity;

    // Integrate preliminary velocityto obtain zero velocity anchors.
    // We do it once here to get proper middle anchor (turnaround index), the
    // final integration for velocity will be done afterwards
    IntegrateWithZeroVelocityAnchors(acceleration, time, 2U, 0U,
                                     preliminaryVelocity);

    std::vector<float> preliminaryPosition;

    IntegrateScalar(preliminaryVelocity, time, preliminaryPosition);

    // We discard ~5% of samples from the beginning and the end of the array (our
    // motion), since they are the most prone to artefacts, and we can set our anchor in the wrong place
    const std::size_t margin = std::max<std::size_t>(
        3U, static_cast<std::size_t>(0.05f * sampleCount));
    if(2U * margin >= sampleCount)
    {
        return Failure(ProcessingStatus::NotEnoughSamples, sampleCount);
    }

    // Obtain the turnaround iterator and then index, by getting the min
    // element from [margin, N-margin], where N is the count of samples
    const auto turnaroundIt
        = std::min_element(preliminaryPosition.begin() + margin,
                           preliminaryPosition.begin() + (sampleCount - margin));

    const std::size_t turnaroundIndex
        = static_cast<std::size_t>(turnaroundIt - preliminaryPosition.begin());

    std::vector<Vec3> velocity(sampleCount);
    // Integrate again over acceleration, but with turnaround index, to obtain velocity on x,y,z
    for(std::size_t axis = 0; axis < 3U; ++axis)
    {
        // For Z axis, we put our anchor turnaroundIndex, for x,y our anchors
        // will be places only at the beginning and the end
        // We put v(ta)
        IntegrateWithZeroVelocityAnchors(acceleration, time, axis,
                                         axis == 2U ? turnaroundIndex : 0U,
                                         scratch);
        for(std::size_t i = 0; i < sampleCount; ++i)
        {
            SetComponent(velocity[i], axis, scratch[i]);
        }
    }

    // Calculate displacement (relative position, not absolute)
    std::vector<Vec3> position(sampleCount);
    for(std::size_t i = 1; i < sampleCount; ++i)
    {
        const float dt = time[i] - time[i - 1U];
        position[i].x = position[i - 1U].x
                        + 0.5f * (velocity[i - 1U].x + velocity[i].x) * dt;
        position[i].y = position[i - 1U].y
                        + 0.5f * (velocity[i - 1U].y + velocity[i].y) * dt;
        position[i].z = position[i - 1U].z
                        + 0.5f * (velocity[i - 1U].z + velocity[i].z) * dt;
    }

    float meanHorizontalX = 0.0f;
    float meanHorizontalY = 0.0f;
    // Calculate average displacement on X and Y to determine covariance and
    // which axis should be main plane, the device moved across - we will use PCA for that
    for(const Vec3 &rPoint : position)
    {
        meanHorizontalX += rPoint.x - position.front().x;
        meanHorizontalY += rPoint.y - position.front().y;
    }
    meanHorizontalX /= static_cast<float>(sampleCount);
    meanHorizontalY /= static_cast<float>(sampleCount);

    float covarianceXX = 0.0f;
    float covarianceXY = 0.0f;
    float covarianceYY = 0.0f;
    // Calculate covariance xi - x0 - x^
    // mean is already calculated, so we can just subtract it to obtain
    // For the use of PCA, we need to calculate covariance matrix to determine
    // along which axis device moved the most We can not state, that device moved
    // across XY in some type of way, it could be only across X, only across Y
    for(const Vec3 &rPoint : position)
    {
        const float centeredX = rPoint.x - position.front().x - meanHorizontalX;
        const float centeredY = rPoint.y - position.front().y - meanHorizontalY;

        // We do not need to divide by N for full covariance, since PCA needs just the general direction
        covarianceXX += centeredX * centeredX;
        covarianceXY += centeredX * centeredY;
        covarianceYY += centeredY * centeredY;
    }

    float principalX = 1.0f;
    float principalY = 0.0f;
    if(covarianceXX + covarianceYY > std::numeric_limits<float>::epsilon())
    {
        /* Since our matrix is 2x2 and symmetric:
            | a, b |            | Cxx, Cxy |
            | b, c |, we get    | Cxy, Cyy |

            Thats why, we calculate our angle as tan(2theta) = 2b/a-c
            So we get theta = 1/2 * atan2(2Cxy, Cxx - Cyy)

            Later, since we know our vectors are unit vectors, we can get our
                X axis defined as cos(theta) // principalX = cos(angle)
                Y axis defined as sin(theta) // principalY = sin(angle
            */
        const float angle
            = 0.5f
              * std::atan2(2.0f * covarianceXY, covarianceXX - covarianceYY);

        principalX = std::cos(angle);
        principalY = std::sin(angle);
    }
    // Since we've got our axis for direction defined, where PC1 = (principalX,
    // principalY), we can map our (x,y) coordinates onto that unit vector of general direction
    /* We do that, by just multiplying every displacement from origin (xi -
       x0), where xi is i-th Point, and x0 is the position of origin, and then
       multiplying that by our u1 vector
       So basic formula for that would look something like that

       hi = (xi - x0) * cos(angle) + (yi - y0) * sin(angle)

    */
    std::vector<float> horizontalPosition(sampleCount);
    for(std::size_t i = 0; i < sampleCount; ++i)
    {
        horizontalPosition[i]
            = (position[i].x - position.front().x) * principalX
              + (position[i].y - position.front().y) * principalY;
    }
    if(horizontalPosition.back() < 0.0f)
    {
        for(float &rPosition : horizontalPosition)
        {
            rPosition = -rPosition;
        }
    }

    const auto verticalMinimumMaximum
        = std::minmax_element(position.begin(), position.end(),
                              [](const Vec3 &rLeft, const Vec3 &rRight) {
                                  return rLeft.z < rRight.z;
                              });
    const float verticalRom
        = verticalMinimumMaximum.second->z - verticalMinimumMaximum.first->z;

    const auto topIt
        = std::max_element(position.begin() + turnaroundIndex, position.end(),
                           [](const Vec3 &rLeft, const Vec3 &rRight) {
                               return rLeft.z < rRight.z;
                           });
    const std::size_t concentricEndIndex
        = static_cast<std::size_t>(topIt - position.begin());

    std::size_t movementStartIndex = 0U;
    for(std::size_t i = 0; i <= turnaroundIndex; ++i)
    {
        if(velocity[i].z <= MOTION_START_VELOCITY_MPS)
        {
            movementStartIndex = i;
            break;
        }
    }

    float maximumVelocity = 0.0f;
    for(std::size_t i = turnaroundIndex; i < sampleCount; ++i)
    {
        maximumVelocity = std::max(maximumVelocity, velocity[i].z);
    }

    const float concentricDuration
        = time[concentricEndIndex] - time[turnaroundIndex];
    float meanConcentricVelocity = 0.0f;
    if(concentricDuration > 0.0f)
    {
        meanConcentricVelocity
            = (position[concentricEndIndex].z - position[turnaroundIndex].z)
              / concentricDuration;
    }

    float maximumPower = 0.0f;
    float meanPower = 0.0f;
    const bool powerValid = std::isfinite(rConfig.barbellMassKg)
                            && rConfig.barbellMassKg > 0.0f
                            && concentricEndIndex > turnaroundIndex;
    if(powerValid)
    {
        float work = 0.0f;
        float previousPower = rConfig.barbellMassKg
                              * (GRAVITY_MS2 + acceleration[turnaroundIndex].z)
                              * velocity[turnaroundIndex].z;
        maximumPower = std::max(0.0f, previousPower);
        for(std::size_t i = turnaroundIndex + 1U; i <= concentricEndIndex; ++i)
        {
            const float power = rConfig.barbellMassKg
                                * (GRAVITY_MS2 + acceleration[i].z)
                                * velocity[i].z;
            maximumPower = std::max(maximumPower, power);
            work += 0.5f * (previousPower + power) * (time[i] - time[i - 1U]);
            previousPower = power;
        }
        meanPower = work / concentricDuration;
    }

    ProcessingResult result{};
    result.rawSampleCount = static_cast<uint16_t>(std::min(
        sampleCount,
        static_cast<std::size_t>(std::numeric_limits<uint16_t>::max())));
    result.turnaroundIndex = static_cast<uint16_t>(turnaroundIndex);
    result.durationMs = ToMilliseconds(time.back() - time.front());
    result.eccentricDurationMs
        = ToMilliseconds(time[turnaroundIndex] - time[movementStartIndex]);
    result.concentricDurationMs = ToMilliseconds(concentricDuration);
    result.verticalRomMm = ToInt16(verticalRom * 1000.0f);
    result.maxVelocityMmPerSec = ToInt16(maximumVelocity * 1000.0f);
    result.meanConcentricVelocityMmPerSec
        = ToInt16(meanConcentricVelocity * 1000.0f);
    result.maxPowerMilliwatts = ToInt32(maximumPower * 1000.0f);
    result.meanConcentricPowerMilliwatts = ToInt32(meanPower * 1000.0f);
    result.maxWorldGravityStdMg = static_cast<uint16_t>(std::lround(std::max(
        0.0f,
        std::min(maxGravityStd,
                 static_cast<float>(std::numeric_limits<uint16_t>::max())))));
    result.powerValid = powerValid;

    const std::size_t requestedCount = std::max<std::size_t>(
        3U, std::min<std::size_t>(rConfig.trajectoryPointCount,
                                  MAX_TRAJECTORY_POINTS));
    const std::size_t outputCount = std::min(sampleCount, requestedCount);
    std::size_t turnaroundOutputIndex = static_cast<std::size_t>(
        std::lround(static_cast<double>(turnaroundIndex)
                    * static_cast<double>(outputCount - 1U)
                    / static_cast<double>(sampleCount - 1U)));
    turnaroundOutputIndex = std::max<std::size_t>(
        1U, std::min(turnaroundOutputIndex, outputCount - 2U));

    for(std::size_t outputIndex = 0; outputIndex < outputCount; ++outputIndex)
    {
        std::size_t sampleIndex;
        if(outputIndex <= turnaroundOutputIndex)
        {
            sampleIndex = static_cast<std::size_t>(
                std::lround(static_cast<double>(turnaroundIndex)
                            * static_cast<double>(outputIndex)
                            / static_cast<double>(turnaroundOutputIndex)));
        } else
        {
            sampleIndex
                = turnaroundIndex
                  + static_cast<std::size_t>(std::lround(
                      static_cast<double>(sampleCount - 1U - turnaroundIndex)
                      * static_cast<double>(outputIndex - turnaroundOutputIndex)
                      / static_cast<double>(outputCount - 1U
                                            - turnaroundOutputIndex)));
        }
        result.trajectory[outputIndex].horizontalMm
            = ToInt16(horizontalPosition[sampleIndex] * 1000.0f);
        result.trajectory[outputIndex].verticalMm
            = ToInt16((position[sampleIndex].z - position.front().z) * 1000.0f);
    }

    result.trajectoryPointCount = static_cast<uint16_t>(outputCount);
    result.turnaroundTrajectoryIndex
        = static_cast<uint16_t>(turnaroundOutputIndex);
    result.status = ProcessingStatus::Ok;
    result.valid = true;
    return result;
}

const char *MotionProcessor::StatusToString(ProcessingStatus status)
{
    switch(status)
    {
    case ProcessingStatus::NotProcessed: return "not processed";
    case ProcessingStatus::Ok: return "ok";
    case ProcessingStatus::NotEnoughSamples: return "not enough samples";
    case ProcessingStatus::InvalidTimestamp: return "invalid timestamp";
    case ProcessingStatus::MissingSamples: return "missing samples";
    case ProcessingStatus::InvalidQuaternion: return "invalid quaternion";
    case ProcessingStatus::InvalidGravity: return "invalid gravity";
    case ProcessingStatus::InvalidFilterConfiguration:
        return "invalid filter configuration";
    default: return "unknown";
    }
}
} // namespace w8band::Motion
