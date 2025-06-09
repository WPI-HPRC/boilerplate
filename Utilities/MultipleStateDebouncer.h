#pragma once

template <typename T>
class MultipleStateDebouncer {
  public:
    MultipleStateDebouncer(long timeHigh, int numStates, T errorState) : timeHigh(timeHigh), numStates(numStates), errorState(errorState) {}

    T update(T state, long now) {
      if (state != lastState) {
        lastTime = -1;
        return errorState;
      } else {
        if (lastTime == -1) lastTime = now;
        if (now - lastTime >= timeHigh) {
          return lastState;
        } else {
          return errorState;
        }
      }
    }

  private:
    long timeHigh;
    long lastTime = 0;
    long numStates;
    T lastState;
    T errorState;
};
