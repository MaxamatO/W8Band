#pragma once
#include "IState.hpp"
#include <memory>
#include <vector>

namespace fsm
{
/// @brief Generic FSM
/// @tparam TContext Shared data struct passed to every state.
/// @tparam TStateId Enum class identyfin each state.
template <typename TContext, typename TStateId, std::size_t MaxStates = 8>
class StateMachine
{
public:
    using State = IState<TContext, TStateId>;

    explicit StateMachine(TContext &rContext) : m_rContext(rContext) {}

    /// @brief Registers a state with the machine.
    /// @param[in] id StateId to register
    /// @param[in] pState Pointer to a state for ownership transfer
    void AddState(TStateId id, std::unique_ptr<State> pState)
    {
        if(m_StateCount >= MaxStates)
        {
            return;
        }
        m_States[m_StateCount].id = id;
        m_States[m_StateCount].pState = std::move(pState);
        m_StateCount++;
    }

    /// @brief Selects the initial state and calls its OnEnter()
    /// @param initialId Initial StateId to start with
    /// @return True if the id was found and fsm has started, false otherwise
    bool Start(TStateId initialId)
    {
        State *pState = FindState(initialId);
        if(pState == nullptr)
        {
            return false;
        }

        m_pCurrentState = pState;
        m_pCurrentState->OnEnter();
        return true;
    }

    /// @brief Requests a transition to a new state. Called from within a state
    /// for a transition.
    /// @param id StateId to which state has requested a transition.
    void RequestTransition(TStateId id)
    {
        m_HasPendingTransition = true;
        m_PendingStateId = id;
    }

    /// @brief Applies pending transition and calls current state's Update().
    void Update()
    {
        if(m_HasPendingTransition)
        {
            ApplyPendingTransition();
        }
        if(m_pCurrentState != nullptr)
        {
            m_pCurrentState->Update();
        }
    }

    /// @brief Returns current state's Id
    /// @return Current state's Id
    TStateId GetCurrentStateId() const
    { return m_pCurrentState->GetStateId(); }

    /// @brief Checks if any state is currently started
    /// @return True if any state is started, false otherwise.
    bool IsStarted() const { return m_pCurrentState != nullptr; }

private:
    struct StateEntry
    {
        TStateId id{};
        std::unique_ptr<State> pState;
    };
    /// @brief Method used for finding specific state
    /// @param id StateId to find
    /// @return Pointer to found State
    State *FindState(TStateId id)
    {
        for(std::size_t i = 0; i < m_StateCount; i++)
        {
            if(m_States[i].id == id)
                return m_States[i].pState.get();
        }
        return nullptr;
    }

    /// @brief Applies any pending transition. Calls OnExit on current state,
    /// and OnEnter on next state.
    void ApplyPendingTransition()
    {
        m_HasPendingTransition = false;

        State *pNextState = FindState(m_PendingStateId);
        if(pNextState == nullptr || pNextState == m_pCurrentState)
        {
            return;
        }

        if(m_pCurrentState != nullptr)
        {
            m_pCurrentState->OnExit();
        }
        m_pCurrentState = pNextState;
        m_pCurrentState->OnEnter();
    }

    /// @brief Shared data context
    TContext &m_rContext;

    /// @brief Vector of pairs containing StateId and pointer to such state.
    std::array<StateEntry, MaxStates> m_States{};

    /// @brief Keeps track of currently registered states count.
    std::size_t m_StateCount = 0;

    /// @brief Currently run state
    State *m_pCurrentState = nullptr;

    /// @brief Value for checking if any transition is pending
    bool m_HasPendingTransition = false;

    /// @brief Variable for holding pedning state's id
    TStateId m_PendingStateId{};
};
} // namespace w8band::StateMachine
