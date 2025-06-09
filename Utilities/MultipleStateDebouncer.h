#pragma once

template <typename T>
class MultipleStateDebouncer {
  public:
    MultipleStateDebouncer(long timeHigh, int numStates) : timeHigh(timeHigh), numStates(numStates) {}

    int update(T state, long now) {
      if (state != lastState) {
        lastTime = -1;
        return -1;
      } else {
        if (lastTime == -1) lastTime = now;
        if (now - lastTime >= timeHigh) {
          return lastState;
        } else {
          return -1;
        }
      }
    }

  private:
    long timeHigh;
    long lastTime = 0;
    long numStates;
    T lastState;
};
