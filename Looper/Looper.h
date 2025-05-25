#pragma once

#include "HardwareTimer.h"
#include <Arduino.h>
#include <cassert>
#include <cstdint>
#include <initializer_list>

using loop_fn_t = void (*)();

template <loop_fn_t...> struct FunctionsList {};

template <uint32_t...> struct FunctionDelaysList {};

template <typename FList, typename DelayList> class Looper;

//! @template_params
//! - List of function pointers
//! - Corresponding delays in ms
template <loop_fn_t... fns, uint32_t... fn_delays>
class Looper<FunctionsList<fns...>, FunctionDelaysList<fn_delays...>> {
  public:
    /** @params
     - Polling delay in microseconds. Make this smaller than the smallest
     delay for a function handled by this looper.

     - Priority of the interrupt, higher number is lower priority (more
     preemptable). To not interfere with the arduino rtos, make this at
     least 10.

     - TIM instance to use for the hardware timer, such as TIM1, TIM2, etc..
     Make sure it is not in use elsewhere (i.e., don't use TIM6/TIM7, as they
     are used by the Tone and Servo libraries).
    */
    Looper(uint32_t pollingDelay, uint32_t prio, TIM_TypeDef *timInst)
        : pollingDelay(pollingDelay), prio(prio), timer(timInst) {}

    void init() {
        timer.setOverflow(pollingDelay, MICROSEC_FORMAT);
        timer.attachInterrupt([this]() { handler(); });
        timer.setInterruptPriority(prio, 0);

        timer.resume();
    }

    static_assert(sizeof...(fns) == sizeof...(fn_delays),
                  "There must be the same number of functions and delays");

  private:
    uint32_t pollingDelay;
    uint32_t prio;
    HardwareTimer timer;
    uint32_t lastCall[sizeof...(fns)] = {};

    void handler() {
        constexpr auto f_list = std::array<loop_fn_t, sizeof...(fns)>{fns...};
        constexpr auto d_list =
            std::array<uint32_t, sizeof...(fn_delays)>{1000 * fn_delays...};
        uint64_t now = micros();
        size_t i = 0;
        for (size_t i = 0; i < sizeof...(fns); i++) {
            if (now - lastCall[i] >= d_list[i]) {
                lastCall[i] = now;
                f_list[i]();
            }
        }
    }
};
