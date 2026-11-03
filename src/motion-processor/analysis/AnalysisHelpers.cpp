#include "AnalysisHelpers.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace w8band::Motion::Analysis
{
namespace
{
constexpr std::size_t FILTER_PAD_LENGTH = 9;
constexpr float PI = 3.14159265358979323846f;
} // namespace

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

float Dot(const Vec3 &rLeft, const Vec3 &rRight)
{ return rLeft.x * rRight.x + rLeft.y * rRight.y + rLeft.z * rRight.z; }

Vec3 Cross(const Vec3 &rLeft, const Vec3 &rRight)
{
    return {rLeft.y * rRight.z - rLeft.z * rRight.y,
            rLeft.z * rRight.x - rLeft.x * rRight.z,
            rLeft.x * rRight.y - rLeft.y * rRight.x};
}

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

Vec3 CalculatePopulationStandardDeviation(const std::vector<Vec3> &rValues)
{
    Vec3 mean{};
    Vec3 sumOfSquaredDifferences{};

    for(std::size_t i = 0; i < rValues.size(); ++i)
    {
        const float count = static_cast<float>(i + 1U);
        const Vec3 delta{rValues[i].x - mean.x, rValues[i].y - mean.y,
                         rValues[i].z - mean.z};
        mean.x += delta.x / count;
        mean.y += delta.y / count;
        mean.z += delta.z / count;

        const Vec3 deltaAfter{rValues[i].x - mean.x, rValues[i].y - mean.y,
                              rValues[i].z - mean.z};
        sumOfSquaredDifferences.x += delta.x * deltaAfter.x;
        sumOfSquaredDifferences.y += delta.y * deltaAfter.y;
        sumOfSquaredDifferences.z += delta.z * deltaAfter.z;
    }

    const float count = static_cast<float>(rValues.size());
    return {std::sqrt(sumOfSquaredDifferences.x / count),
            std::sqrt(sumOfSquaredDifferences.y / count),
            std::sqrt(sumOfSquaredDifferences.z / count)};
}

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
    std::copy(rValues.begin(), rValues.end(), extended.begin() + FILTER_PAD_LENGTH);
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

int16_t ToInt16(float value)
{
    const float minimum
        = static_cast<float>(std::numeric_limits<int16_t>::min());
    const float maximum
        = static_cast<float>(std::numeric_limits<int16_t>::max());
    return static_cast<int16_t>(
        std::lround(std::max(minimum, std::min(value, maximum))));
}

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

} // namespace w8band::Motion::Analysis
