#pragma once

template <typename T>
class RunningExpAverage {
    public:
        RunningExpAverage(T alpha) : alpha(alpha) {}

        void init(T initialVal) {
            val = initialVal;
        }

        void update(T newVal) {
            val = alpha * newVal + (1 - alpha) * val;
        }

        T getAvg() {
            return val;
        }

    private:
        T alpha;
        T val;
};
