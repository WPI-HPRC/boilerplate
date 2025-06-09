#pragma once

template <typename T>
class RunningExpAverage {
    public:
        RunningExpAverage(T alpha) : alpha(alpha), isInit(false) {}

        void update(T newVal) {
            if (isInit){
                val = alpha * newVal + (1 - alpha) * val;
            }
            else {
                val = newVal;
                isInit = true;
            }
        }

        T getAvg() {
            return val;
        }

    private:
        T alpha;
        T val;
        bool isInit;
};
