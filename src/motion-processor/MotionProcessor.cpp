#include "MotionProcessor.hpp"
#include "analysis/AnalysisCore.hpp"
#include "analysis/AnalysisHelpers.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace w8band::Motion
{
namespace
{
ProcessingResult MakeFailureResult(ProcessingStatus status,
                                   std::size_t sampleCount)
{
    ProcessingResult result{};
    result.status = status;
    result.rawSampleCount = static_cast<uint16_t>(std::min(
        sampleCount,
        static_cast<std::size_t>(std::numeric_limits<uint16_t>::max())));
    return result;
}
} // namespace

ProcessingResult
MotionProcessor::Process(const std::vector<data::SamplePacket> &rSamples,
                         const data::AccBiasVec3 &rAccelBias,
                         const ProcessingConfig &rConfig)
{
    std::vector<float> time;
    float sampleRateHz = 0.0f;
    const ProcessingStatus timelineStatus = Analysis::ValidateAndBuildTimeline(
        rSamples, rConfig.lowPassCutoffHz, time, sampleRateHz);
    if(timelineStatus != ProcessingStatus::Ok)
    {
        return MakeFailureResult(timelineStatus, rSamples.size());
    }

    std::vector<Analysis::Vec3> acceleration;
    std::vector<Analysis::Vec3> gravity;
    if(!Analysis::DecodeAndRotateSamples(rSamples, rAccelBias, acceleration,
                                         gravity))
    {
        return MakeFailureResult(ProcessingStatus::InvalidQuaternion,
                                 rSamples.size());
    }

    const Analysis::Vec3 gravityStd
        = Analysis::CalculatePopulationStandardDeviation(gravity);
    const float maxGravityStd
        = std::max(gravityStd.x, std::max(gravityStd.y, gravityStd.z));
    if(!std::isfinite(maxGravityStd)
       || maxGravityStd > rConfig.maxWorldGravityStdMg)
    {
        return MakeFailureResult(ProcessingStatus::InvalidGravity,
                                 rSamples.size());
    }

    std::vector<float> scratch;
    scratch.reserve(rSamples.size());
    Analysis::Vec3 horizontalX;
    Analysis::Vec3 horizontalY;
    Analysis::Vec3 up;
    if(!Analysis::BuildGravityAlignedBasis(gravity, scratch, horizontalX,
                                            horizontalY, up))
    {
        return MakeFailureResult(ProcessingStatus::InvalidGravity,
                                 rSamples.size());
    }

    if(!Analysis::FilterAndProjectAcceleration(
           acceleration, sampleRateHz, rConfig.lowPassCutoffHz, horizontalX,
           horizontalY, up, scratch))
    {
        return MakeFailureResult(ProcessingStatus::InvalidFilterConfiguration,
                                 rSamples.size());
    }

    const std::size_t turnaroundIndex
        = Analysis::FindTurnaroundIndex(acceleration, time);

    std::vector<Analysis::Vec3> velocity;
    std::vector<Analysis::Vec3> position;
    Analysis::IntegrateMotion(acceleration, time, turnaroundIndex, scratch,
                              velocity, position);

    std::vector<float> horizontalPosition;
    Analysis::CalculateHorizontalPosition(position, horizontalPosition);

    return Analysis::BuildProcessingResult(
        acceleration, velocity, position, time, horizontalPosition,
        turnaroundIndex, maxGravityStd, rConfig);
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
