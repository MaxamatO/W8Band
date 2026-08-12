#pragma once
#include "DataContext.hpp"
#include "IState.hpp"
#include "W8BandFsm.hpp"
#include <string>

namespace w8band::StateMachine
{

/// @brief Enum class of possible Motion Phases, that might prove useful for
/// proper data processing. We want to split obtained data into phases, so we
/// can apply ZUPT at REST, and TURNAROUND, se we split whole movement into 2
/// phases, hopefully reducing our drift just enough for rather precise trajectory.
enum class MotionPhase
{
    REST,
    CONCENTRIC,
    TURNAROUND,
    ECCENTRIC
};

/// @brief Processing state - responsible for analysing complete data and
/// calculating bar trajectory instantly after recording stops.
class ProcessingState : public fsm::IState<DataContext, StateId>
{
public:
    ProcessingState(DataContext &rDataCtx, W8BandFsm &rFsm);

    /// @brief @see IState::OnEnter
    void OnEnter() override;

    /// @brief @see IState::OnExit
    void OnExit() override;

    /// @brief @see IState::Update
    void Update() override;

    /// @brief @see IState::GetStateId
    StateId GetStateId() const override;

    /// @brief Human-readable name, kept separate from GetStateId() so the
    ///        FSM's internal lookups stay a cheap enum compare.
    std::string GetStateName() const override;

private:
    W8BandFsm &m_rFsm;
};

} // namespace w8band::StateMachine
