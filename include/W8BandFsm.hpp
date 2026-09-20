#pragma once
#include "DataContext.hpp"
#include "StateId.hpp"
#include "StateMachine.hpp"

namespace w8band::StateMachine
{
/// @brief State machine type used by the W8Band application.
using W8BandFsm = fsm::StateMachine<w8band::DataContext, StateId>;
} // namespace w8band::StateMachine
