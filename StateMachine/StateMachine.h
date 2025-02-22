#pragma once

#include "State.h"

/**
 * @brief Class to handle running a state machine made of classes derived from
 * `State`
 * @note See the `State` class doc comment for description of the template
 * parameters
 */
template <typename Context, typename StateId, typename millis>
class TStateMachine {
  public:
    using State = TState<Context, StateId, millis>;

    TStateMachine(State *initialState) : currentState(initialState) {}

    void initialize() { currentState->initialize(); }

    void loop() {
        State *nextState = currentState->loop();
        if (nextState != nullptr) {
            delete currentState;
            currentState = nextState;
            currentState->initialize();
        }
    }

    StateId getCurrentStateId() { return currentState->getId(); }

  private:
    State *currentState;
};
