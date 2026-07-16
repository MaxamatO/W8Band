#pragma once
#include "DataContext.hpp"
#include "IState.hpp"
#include "StateId.hpp"
#include "W8BandFsm.hpp"

namespace w8band::StateMachine
{
class ArmedState : public fsm::IState<DataContext, StateId>
{
public:
    ArmedState(DataContext &rDataCtx, W8BandFsm &rFsm);
    void OnEnter() override;
    void OnExit() override;
    void Update() override;
    StateId GetStateId() const override;

    static std::string GetStateName();

private:
    /// @brief Reference to FSM for requesting state change.
    W8BandFsm &m_rFsm;

    data::SamplePacket m_SamplePacket;
};
} // namespace w8band::StateMachine
