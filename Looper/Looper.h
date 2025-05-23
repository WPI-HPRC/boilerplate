#pragma once

#include "HardwareTimer.h"
#include <Arduino.h>
#include <cstdint>
#include <initializer_list>

using loop_fn_t = void (*)();

template <loop_fn_t... fns> class Looper {
  public:
    /* @params
     - Polling delay in microseconds. Make this smaller than the smallest
     delay for a function handled by this looper.

     - Priority of the interrupt, higher number is lower priority (more
     preemptable). To not interfere with the arduino rtos, make this at
     least 10.

     - TIM instance to use for the hardware timer, such as TIM1, TIM2, etc..
     Make sure it is not in use elsewhere.

     - List of delays in microseconds corresponding to functions in the template
     parameter.
    */
    template <typename... Args>
    Looper(uint32_t pollingDelay, uint32_t prio, TIM_TypeDef *timInst,
           Args... fn_delays)
        : pollingDelay(pollingDelay), prio(prio), timer(timInst),
          delays{fn_delays...} {}

    void init() {
        timer.setOverflow(pollingDelay, MICROSEC_FORMAT);
        timer.attachInterrupt([this]() { handler(); });
        timer.setInterruptPriority(prio, 0);

        timer.resume();
    }

  private:
    uint32_t pollingDelay;
    uint32_t prio;
    HardwareTimer timer;
    uint32_t delays[sizeof...(fns)];
    uint32_t lastCall[sizeof...(fns)] = {};

    void handler() {
        uint64_t now = micros();
        size_t i = 0;
        for (auto f : std::initializer_list<loop_fn_t>{fns...}) {
            if (now - lastCall[i] >= delays[i]) {
                lastCall[i] = now;
                f();
            }
            i++;
        }
    }
};
