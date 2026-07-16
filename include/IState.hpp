#pragma once

namespace fsm
{

/// @brief Generic FSM state interface
/// @tparam TContext Shared data struct passed to every state
/// @tparam TStateId Enum class identyfying each state
template <typename TContext, typename TStateId> class IState
{
public:
    virtual ~IState() = default;

    /// @brief Called when transition into this state is applied
    virtual void OnEnter() {}

    /// @brief Called when transition out of this state is applied
    virtual void OnExit() {}

    /// @brief Called every StateMachine::Update() while this state is active
    virtual void Update() = 0;

    /// @brief Identifier of this state
    /// @return one of StateId defined in enum class StateId
    virtual TStateId GetStateId() const = 0;

protected:
    explicit IState(TContext &rContext) : m_rContext(rContext) {}

    /// @brief Shared data struct
    TContext &m_rContext;
};

} // namespace fsm