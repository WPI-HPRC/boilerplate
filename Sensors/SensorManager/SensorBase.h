// Sensor.h
#pragma once
#include "ArduinoLog.h"
#include "PinNames.h"
#include "SensorData.h"
#include "WInterrupts.h"
#include "pins_arduino.h"
#include "wiring_constants.h"

template <typename Derived, typename DataT> class Sensor {
  public:
    using data_type = DataT;
    using descriptor_type = SensorData<DataT>;

    bool begin() {
        beginStatus_ = derived().begin_impl();
        if (isInterruptBased_) {
            uint32_t arduinoPin = pinNametoDigitalPin(intPin_);
            pinMode(arduinoPin, INPUT);
            attachInterrupt(
                digitalPinToInterrupt(arduinoPin),
                [this, arduinoPin]() { shouldPoll_ = true; }, RISING);
        }
        return beginStatus_;
    }

    bool init() {
        initStatus_ = derived().init_impl();
        return initStatus_;
    }

    bool getInitStatus() const { return initStatus_; }

    uint32_t getPollingPeriod() const { return pollingPeriodMs_; }

    uint32_t getLastTimePolled() const { return data_.getLastUpdated(); }

    bool isInterruptBased() const { return isInterruptBased_; }

    // Called by SensorManager
    void poll(uint32_t now_ms) {
        if ((isInterruptBased_ && shouldPoll_) ||
            (!isInterruptBased_ &&
             now_ms - getLastTimePolled() >= pollingPeriodMs_)) {

            shouldPoll_ = false;
            // let the derived class fill in data_.value
            bool updated = derived().poll_impl(now_ms, data_.data);
            if (updated) {
                data_.markUpdated(now_ms);
            }
        }
    }

    // Access to the data
    // const version allows for read only access to the data
    // non-const version allows for read and write access to the data
    const descriptor_type &get_descriptor() const { return data_; }
    descriptor_type &get_descriptor() { return data_; }

  protected:
    explicit Sensor(uint32_t polling_ms)
        : pollingPeriodMs_(polling_ms), isInterruptBased_(false) {}
    explicit Sensor(PinName drdy_int_pin)
        : intPin_(drdy_int_pin), isInterruptBased_(true) {}

    Derived &derived() { return static_cast<Derived &>(*this); }
    const Derived &derived() const {
        return static_cast<const Derived &>(*this);
    }

    descriptor_type data_; // contains {value, lastUpdated}
    uint32_t pollingPeriodMs_;
    PinName intPin_;
    bool isInterruptBased_;
    volatile bool shouldPoll_ = false;

    bool initStatus_ = false;
    bool beginStatus_ = false;
};
