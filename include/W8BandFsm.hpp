#pragma once
#include "DataContext.hpp"
#include "StateId.hpp"
#include "StateMachine.hpp"

namespace w8band::StateMachine
{
using W8BandFsm = fsm::StateMachine<w8band::DataContext, StateId>;
} // namespace w8band::StateMachine
