#include "motion-processor/MotionProcessor.hpp"

#include <array>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
constexpr std::size_t CSV_COLUMN_COUNT = 14;

std::array<long, CSV_COLUMN_COUNT> ParseRow(const std::string &rLine)
{
    std::array<long, CSV_COLUMN_COUNT> values{};
    std::stringstream stream(rLine);
    std::string field;
    for(std::size_t i = 0; i < values.size(); ++i)
    {
        if(!std::getline(stream, field, ','))
        {
            throw std::runtime_error("CSV row has too few columns");
        }
        values[i] = std::stol(field);
    }
    return values;
}

data::SamplePacket MakePacket(
    const std::array<long, CSV_COLUMN_COUNT> &rValues)
{
    data::SamplePacket packet{};
    packet.timestamp_ticks = static_cast<uint32_t>(rValues[0]);
    for(std::size_t i = 0; i < 4U; ++i)
    {
        packet.q[i] = static_cast<int16_t>(rValues[1U + i]);
    }
    for(std::size_t i = 0; i < 3U; ++i)
    {
        packet.a[i] = static_cast<int16_t>(rValues[5U + i]);
        packet.gv[i] = static_cast<int16_t>(rValues[8U + i]);
    }
    return packet;
}
} // namespace

int main(int argc, char **argv)
{
    if(argc < 2 || argc > 3)
    {
        std::cerr << "usage: motion_processor_cli DATASET.csv [mass_kg]\n";
        return 2;
    }

    std::ifstream input(argv[1]);
    if(!input)
    {
        std::cerr << "cannot open: " << argv[1] << '\n';
        return 2;
    }

    std::string line;
    std::getline(input, line); // Named header produced by ProcessingState.

    std::vector<data::SamplePacket> packets;
    data::AccBiasVec3 bias{};
    bool biasInitialized = false;
    while(std::getline(input, line))
    {
        if(line.empty())
        {
            continue;
        }
        const auto row = ParseRow(line);
        packets.push_back(MakePacket(row));
        if(!biasInitialized)
        {
            bias.axBias = static_cast<int32_t>(row[11]);
            bias.ayBias = static_cast<int32_t>(row[12]);
            bias.azBias = static_cast<int32_t>(row[13]);
            biasInitialized = true;
        }
    }

    w8band::Motion::ProcessingConfig config{};
    if(argc == 3)
    {
        config.barbellMassKg = std::stof(argv[2]);
    }
    const auto result
        = w8band::Motion::MotionProcessor::Process(packets, bias, config);

    std::cout << "status="
              << w8band::Motion::MotionProcessor::StatusToString(result.status)
              << '\n';
    std::cout << "samples=" << result.rawSampleCount << '\n';
    std::cout << "turnaround_index=" << result.turnaroundIndex << '\n';
    std::cout << "duration_ms=" << result.durationMs << '\n';
    std::cout << "eccentric_ms=" << result.eccentricDurationMs << '\n';
    std::cout << "concentric_ms=" << result.concentricDurationMs << '\n';
    std::cout << "vertical_rom_mm=" << result.verticalRomMm << '\n';
    std::cout << "max_velocity_mm_s=" << result.maxVelocityMmPerSec << '\n';
    std::cout << "mean_velocity_mm_s="
              << result.meanConcentricVelocityMmPerSec << '\n';
    std::cout << "max_power_mw=" << result.maxPowerMilliwatts << '\n';
    std::cout << "mean_power_mw=" << result.meanConcentricPowerMilliwatts
              << '\n';
    std::cout << "gravity_std_max_mg=" << result.maxWorldGravityStdMg << '\n';
    std::cout << "trajectory_points=" << result.trajectoryPointCount << '\n';
    std::cout << "turnaround_trajectory_index="
              << result.turnaroundTrajectoryIndex << '\n';

    if(!result.valid)
    {
        return 1;
    }
    for(uint16_t i = 0; i < result.trajectoryPointCount; ++i)
    {
        const auto &rPoint = result.trajectory[i];
        std::cout << "point=" << i << ',' << rPoint.horizontalMm << ','
                  << rPoint.verticalMm << '\n';
    }
    return 0;
}
