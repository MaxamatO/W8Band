#include "AnalysisCore.hpp"
#include "helpers/ProcessingHelpers.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace w8band::Motion::Analysis
{
namespace
{
constexpr std::size_t MIN_SAMPLE_COUNT = 20;
constexpr float MAX_SAMPLE_INTERVAL_FACTOR = 1.5f;
constexpr float MG_TO_MS2 = 9.80665f / 1000.0f;
constexpr float GRAVITY_MS2 = 9.80665f;
constexpr float QUATERNION_MIN_NORM = 0.5f;
constexpr float MOTION_START_VELOCITY_MPS = -0.05f;
} // namespace

ProcessingStatus ValidateAndBuildTimeline(
    const std::vector<data::SamplePacket> &rSamples, float lowPassCutoffHz,
    std::vector<float> &rTime, float &rSampleRateHz)
{
    if(rSamples.size() < MIN_SAMPLE_COUNT)
    {
        return ProcessingStatus::NotEnoughSamples;
    }

    std::vector<uint32_t> deltaTicks(rSamples.size() - 1U);
    for(std::size_t i = 1; i < rSamples.size(); ++i)
    {
        deltaTicks[i - 1U]
            = rSamples[i].timestamp_ticks - rSamples[i - 1U].timestamp_ticks;
        if(deltaTicks[i - 1U] == 0U)
        {
            return ProcessingStatus::InvalidTimestamp;
        }
    }

    const float medianDeltaTicks = MedianSortedDeltas(deltaTicks);
    const uint32_t maxDeltaTicks = deltaTicks.back();
    if(static_cast<float>(maxDeltaTicks)
       > MAX_SAMPLE_INTERVAL_FACTOR * medianDeltaTicks)
    {
        return ProcessingStatus::MissingSamples;
    }

    const float medianDt = medianDeltaTicks * LSM_TIMESTAMP_TICK_SECONDS;
    rSampleRateHz = 1.0f / medianDt;
    if(lowPassCutoffHz <= 0.0f || lowPassCutoffHz >= rSampleRateHz * 0.5f)
    {
        return ProcessingStatus::InvalidFilterConfiguration;
    }

    rTime.assign(rSamples.size(), 0.0f);
    for(std::size_t i = 1; i < rSamples.size(); ++i)
    {
        const uint32_t elapsedTicks
            = rSamples[i].timestamp_ticks - rSamples[i - 1U].timestamp_ticks;
        rTime[i] = rTime[i - 1U]
                   + static_cast<float>(elapsedTicks)
                         * LSM_TIMESTAMP_TICK_SECONDS;
    }
    return ProcessingStatus::Ok;
}

bool DecodeAndRotateSamples(const std::vector<data::SamplePacket> &rSamples,
                            const data::AccBiasVec3 &rAccelBias,
                            std::vector<Vec3> &rAcceleration,
                            std::vector<Vec3> &rGravity)
{
    rAcceleration.resize(rSamples.size());
    rGravity.resize(rSamples.size());
    helpers::Processing::Quat previousQuat{1.0f, 0.0f, 0.0f, 0.0f};

    for(std::size_t i = 0; i < rSamples.size(); ++i)
    {
        const data::SamplePacket &rPacket = rSamples[i];
        const float rawQw = rPacket.q[0] / Q14_SCALE;
        const float rawQx = rPacket.q[1] / Q14_SCALE;
        const float rawQy = rPacket.q[2] / Q14_SCALE;
        const float rawQz = rPacket.q[3] / Q14_SCALE;
        const float quaternionNorm = std::sqrt(
            rawQw * rawQw + rawQx * rawQx + rawQy * rawQy + rawQz * rawQz);
        if(!std::isfinite(quaternionNorm)
           || quaternionNorm < QUATERNION_MIN_NORM)
        {
            return false;
        }

        const helpers::Processing::Quat q
            = helpers::Processing::DecodeQuaternion(rPacket, previousQuat);
        const float linearBodyX = static_cast<float>(rPacket.a[0])
                                  - static_cast<float>(rAccelBias.axBias)
                                  - static_cast<float>(rPacket.gv[0]);
        const float linearBodyY = static_cast<float>(rPacket.a[1])
                                  - static_cast<float>(rAccelBias.ayBias)
                                  - static_cast<float>(rPacket.gv[1]);
        const float linearBodyZ = static_cast<float>(rPacket.a[2])
                                  - static_cast<float>(rAccelBias.azBias)
                                  - static_cast<float>(rPacket.gv[2]);

        helpers::Processing::RotateBodyToWorld(
            q.w, q.x, q.y, q.z, linearBodyX, linearBodyY, linearBodyZ,
            rAcceleration[i].x, rAcceleration[i].y, rAcceleration[i].z);
        helpers::Processing::RotateBodyToWorld(
            q.w, q.x, q.y, q.z, static_cast<float>(rPacket.gv[0]),
            static_cast<float>(rPacket.gv[1]), static_cast<float>(rPacket.gv[2]),
            rGravity[i].x, rGravity[i].y, rGravity[i].z);
    }
    return true;
}

bool BuildGravityAlignedBasis(const std::vector<Vec3> &rGravity,
                              std::vector<float> &rScratch, Vec3 &rHorizontalX,
                              Vec3 &rHorizontalY, Vec3 &rUp)
{
    rUp = {MedianComponent(rGravity, 0U, rScratch),
           MedianComponent(rGravity, 1U, rScratch),
           MedianComponent(rGravity, 2U, rScratch)};
    if(!Normalize(rUp))
    {
        return false;
    }

    Vec3 reference{1.0f, 0.0f, 0.0f};
    if(std::fabs(Dot(reference, rUp)) > 0.9f)
    {
        reference = {0.0f, 1.0f, 0.0f};
    }

    const float projection = Dot(reference, rUp);
    rHorizontalX = {reference.x - projection * rUp.x,
                    reference.y - projection * rUp.y,
                    reference.z - projection * rUp.z};
    if(!Normalize(rHorizontalX))
    {
        return false;
    }

    rHorizontalY = Cross(rUp, rHorizontalX);
    return Normalize(rHorizontalY);
}

bool FilterAndProjectAcceleration(std::vector<Vec3> &rAcceleration,
                                  float sampleRateHz, float cutoffHz,
                                  const Vec3 &rHorizontalX,
                                  const Vec3 &rHorizontalY, const Vec3 &rUp,
                                  std::vector<float> &rScratch)
{
    for(std::size_t axis = 0; axis < 3U; ++axis)
    {
        rScratch.resize(rAcceleration.size());
        for(std::size_t i = 0; i < rAcceleration.size(); ++i)
        {
            rScratch[i] = GetComponent(rAcceleration[i], axis);
        }
        if(!ApplyZeroPhaseLowPass(rScratch, sampleRateHz, cutoffHz))
        {
            return false;
        }
        for(std::size_t i = 0; i < rAcceleration.size(); ++i)
        {
            SetComponent(rAcceleration[i], axis, rScratch[i] * MG_TO_MS2);
        }
    }

    for(Vec3 &rSample : rAcceleration)
    {
        const Vec3 world = rSample;
        rSample = {Dot(world, rHorizontalX), Dot(world, rHorizontalY),
                   Dot(world, rUp)};
    }
    return true;
}

std::size_t FindTurnaroundIndex(const std::vector<Vec3> &rAcceleration,
                                const std::vector<float> &rTime)
{
    std::vector<float> preliminaryVelocity;
    IntegrateWithZeroVelocityAnchors(rAcceleration, rTime, 2U, 0U,
                                     preliminaryVelocity);

    std::vector<float> preliminaryPosition;
    IntegrateScalar(preliminaryVelocity, rTime, preliminaryPosition);

    const std::size_t margin = std::max<std::size_t>(
        3U, static_cast<std::size_t>(0.05f * rAcceleration.size()));
    const auto turnaroundIt = std::min_element(
        preliminaryPosition.begin() + margin,
        preliminaryPosition.begin() + (rAcceleration.size() - margin));
    return static_cast<std::size_t>(turnaroundIt
                                    - preliminaryPosition.begin());
}

void IntegrateMotion(const std::vector<Vec3> &rAcceleration,
                     const std::vector<float> &rTime,
                     std::size_t turnaroundIndex,
                     std::vector<float> &rScratch,
                     std::vector<Vec3> &rVelocity,
                     std::vector<Vec3> &rPosition)
{
    rVelocity.assign(rAcceleration.size(), Vec3{});
    for(std::size_t axis = 0; axis < 3U; ++axis)
    {
        IntegrateWithZeroVelocityAnchors(
            rAcceleration, rTime, axis,
            axis == 2U ? turnaroundIndex : 0U, rScratch);
        for(std::size_t i = 0; i < rAcceleration.size(); ++i)
        {
            SetComponent(rVelocity[i], axis, rScratch[i]);
        }
    }

    rPosition.assign(rAcceleration.size(), Vec3{});
    for(std::size_t i = 1; i < rAcceleration.size(); ++i)
    {
        const float dt = rTime[i] - rTime[i - 1U];
        rPosition[i].x = rPosition[i - 1U].x
                         + 0.5f * (rVelocity[i - 1U].x + rVelocity[i].x) * dt;
        rPosition[i].y = rPosition[i - 1U].y
                         + 0.5f * (rVelocity[i - 1U].y + rVelocity[i].y) * dt;
        rPosition[i].z = rPosition[i - 1U].z
                         + 0.5f * (rVelocity[i - 1U].z + rVelocity[i].z) * dt;
    }
}

void CalculateHorizontalPosition(const std::vector<Vec3> &rPosition,
                                 std::vector<float> &rHorizontalPosition)
{
    float meanX = 0.0f;
    float meanY = 0.0f;
    for(const Vec3 &rPoint : rPosition)
    {
        meanX += rPoint.x - rPosition.front().x;
        meanY += rPoint.y - rPosition.front().y;
    }
    meanX /= static_cast<float>(rPosition.size());
    meanY /= static_cast<float>(rPosition.size());

    float covarianceXX = 0.0f;
    float covarianceXY = 0.0f;
    float covarianceYY = 0.0f;
    for(const Vec3 &rPoint : rPosition)
    {
        const float centeredX = rPoint.x - rPosition.front().x - meanX;
        const float centeredY = rPoint.y - rPosition.front().y - meanY;
        covarianceXX += centeredX * centeredX;
        covarianceXY += centeredX * centeredY;
        covarianceYY += centeredY * centeredY;
    }

    float principalX = 1.0f;
    float principalY = 0.0f;
    if(covarianceXX + covarianceYY > std::numeric_limits<float>::epsilon())
    {
        const float angle
            = 0.5f
              * std::atan2(2.0f * covarianceXY, covarianceXX - covarianceYY);
        principalX = std::cos(angle);
        principalY = std::sin(angle);
    }

    rHorizontalPosition.resize(rPosition.size());
    for(std::size_t i = 0; i < rPosition.size(); ++i)
    {
        rHorizontalPosition[i]
            = (rPosition[i].x - rPosition.front().x) * principalX
              + (rPosition[i].y - rPosition.front().y) * principalY;
    }
    if(rHorizontalPosition.back() < 0.0f)
    {
        for(float &rValue : rHorizontalPosition)
        {
            rValue = -rValue;
        }
    }
}

namespace
{
void PopulateMotionMetrics(
    ProcessingResult &rResult, const std::vector<Vec3> &rAcceleration,
    const std::vector<Vec3> &rVelocity,
    const std::vector<Vec3> &rPosition, const std::vector<float> &rTime,
    std::size_t turnaroundIndex, float maxGravityStd, float barbellMassKg)
{
    const auto verticalMinimumMaximum = std::minmax_element(
        rPosition.begin(), rPosition.end(),
        [](const Vec3 &rLeft, const Vec3 &rRight) {
            return rLeft.z < rRight.z;
        });
    const float verticalRom
        = verticalMinimumMaximum.second->z - verticalMinimumMaximum.first->z;

    const auto topIt = std::max_element(
        rPosition.begin() + turnaroundIndex, rPosition.end(),
        [](const Vec3 &rLeft, const Vec3 &rRight) {
            return rLeft.z < rRight.z;
        });
    const std::size_t concentricEndIndex
        = static_cast<std::size_t>(topIt - rPosition.begin());

    std::size_t movementStartIndex = 0U;
    for(std::size_t i = 0; i <= turnaroundIndex; ++i)
    {
        if(rVelocity[i].z <= MOTION_START_VELOCITY_MPS)
        {
            movementStartIndex = i;
            break;
        }
    }

    float maximumVelocity = 0.0f;
    for(std::size_t i = turnaroundIndex; i < rVelocity.size(); ++i)
    {
        maximumVelocity = std::max(maximumVelocity, rVelocity[i].z);
    }

    const float concentricDuration
        = rTime[concentricEndIndex] - rTime[turnaroundIndex];
    float meanConcentricVelocity = 0.0f;
    if(concentricDuration > 0.0f)
    {
        meanConcentricVelocity
            = (rPosition[concentricEndIndex].z - rPosition[turnaroundIndex].z)
              / concentricDuration;
    }

    float maximumPower = 0.0f;
    float meanPower = 0.0f;
    const bool powerValid = std::isfinite(barbellMassKg)
                            && barbellMassKg > 0.0f
                            && concentricEndIndex > turnaroundIndex;
    if(powerValid)
    {
        float work = 0.0f;
        float previousPower = barbellMassKg
                              * (GRAVITY_MS2
                                 + rAcceleration[turnaroundIndex].z)
                              * rVelocity[turnaroundIndex].z;
        maximumPower = std::max(0.0f, previousPower);
        for(std::size_t i = turnaroundIndex + 1U; i <= concentricEndIndex; ++i)
        {
            const float power = barbellMassKg
                                * (GRAVITY_MS2 + rAcceleration[i].z)
                                * rVelocity[i].z;
            maximumPower = std::max(maximumPower, power);
            work += 0.5f * (previousPower + power)
                    * (rTime[i] - rTime[i - 1U]);
            previousPower = power;
        }
        meanPower = work / concentricDuration;
    }

    rResult.rawSampleCount = static_cast<uint16_t>(std::min(
        rPosition.size(),
        static_cast<std::size_t>(std::numeric_limits<uint16_t>::max())));
    rResult.turnaroundIndex = static_cast<uint16_t>(turnaroundIndex);
    rResult.durationMs = ToMilliseconds(rTime.back() - rTime.front());
    rResult.eccentricDurationMs = ToMilliseconds(
        rTime[turnaroundIndex] - rTime[movementStartIndex]);
    rResult.concentricDurationMs = ToMilliseconds(concentricDuration);
    rResult.verticalRomMm = ToInt16(verticalRom * 1000.0f);
    rResult.maxVelocityMmPerSec = ToInt16(maximumVelocity * 1000.0f);
    rResult.meanConcentricVelocityMmPerSec
        = ToInt16(meanConcentricVelocity * 1000.0f);
    rResult.maxPowerMilliwatts = ToInt32(maximumPower * 1000.0f);
    rResult.meanConcentricPowerMilliwatts = ToInt32(meanPower * 1000.0f);
    rResult.maxWorldGravityStdMg = static_cast<uint16_t>(std::lround(std::max(
        0.0f,
        std::min(maxGravityStd,
                 static_cast<float>(std::numeric_limits<uint16_t>::max())))));
    rResult.powerValid = powerValid;
}

void PopulateReducedTrajectory(
    ProcessingResult &rResult, const std::vector<Vec3> &rPosition,
    const std::vector<float> &rHorizontalPosition,
    std::size_t turnaroundIndex, uint16_t requestedPointCount)
{
    const std::size_t requestedCount = std::max<std::size_t>(
        3U, std::min<std::size_t>(requestedPointCount,
                                  MAX_TRAJECTORY_POINTS));
    const std::size_t outputCount
        = std::min(rPosition.size(), requestedCount);
    std::size_t turnaroundOutputIndex = static_cast<std::size_t>(std::lround(
        static_cast<double>(turnaroundIndex)
        * static_cast<double>(outputCount - 1U)
        / static_cast<double>(rPosition.size() - 1U)));
    turnaroundOutputIndex = std::max<std::size_t>(
        1U, std::min(turnaroundOutputIndex, outputCount - 2U));

    for(std::size_t outputIndex = 0; outputIndex < outputCount; ++outputIndex)
    {
        std::size_t sampleIndex;
        if(outputIndex <= turnaroundOutputIndex)
        {
            sampleIndex = static_cast<std::size_t>(std::lround(
                static_cast<double>(turnaroundIndex)
                * static_cast<double>(outputIndex)
                / static_cast<double>(turnaroundOutputIndex)));
        } else
        {
            sampleIndex
                = turnaroundIndex
                  + static_cast<std::size_t>(std::lround(
                      static_cast<double>(rPosition.size() - 1U
                                          - turnaroundIndex)
                      * static_cast<double>(outputIndex - turnaroundOutputIndex)
                      / static_cast<double>(outputCount - 1U
                                            - turnaroundOutputIndex)));
        }
        rResult.trajectory[outputIndex].horizontalMm
            = ToInt16(rHorizontalPosition[sampleIndex] * 1000.0f);
        rResult.trajectory[outputIndex].verticalMm = ToInt16(
            (rPosition[sampleIndex].z - rPosition.front().z) * 1000.0f);
    }

    rResult.trajectoryPointCount = static_cast<uint16_t>(outputCount);
    rResult.turnaroundTrajectoryIndex
        = static_cast<uint16_t>(turnaroundOutputIndex);
}
} // namespace

ProcessingResult BuildProcessingResult(
    const std::vector<Vec3> &rAcceleration,
    const std::vector<Vec3> &rVelocity,
    const std::vector<Vec3> &rPosition, const std::vector<float> &rTime,
    const std::vector<float> &rHorizontalPosition,
    std::size_t turnaroundIndex, float maxGravityStd,
    const ProcessingConfig &rConfig)
{
    ProcessingResult result{};
    PopulateMotionMetrics(result, rAcceleration, rVelocity, rPosition, rTime,
                          turnaroundIndex, maxGravityStd,
                          rConfig.barbellMassKg);
    PopulateReducedTrajectory(result, rPosition, rHorizontalPosition,
                              turnaroundIndex, rConfig.trajectoryPointCount);
    result.status = ProcessingStatus::Ok;
    result.valid = true;
    return result;
}

} // namespace w8band::Motion::Analysis
