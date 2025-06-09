#pragma once
#include <cstddef>

/**
 * @brief Fixed-size moving average calculator using a static circular buffer.
 *
 * This class maintains a running sum over a window of N samples to provide an O(1)
 * update and average retrieval. Suitable for low-capability processors (no dynamic
 * allocation required).
 *
 * @tparam T Numeric type of each sample (e.g., float, double, int).
 * @tparam N Number of samples in the moving window.
 */
template <typename T, std::size_t N>
class TimeAverage {
public:
    /**
     * @brief Default constructor.
     *
     * Initializes internal counters and sum to zero.
     */
    TimeAverage()
        : sum(T{}), count(0), index(0) {}

    /**
     * @brief Fill the buffer with an initial value.
     *
     * Sets every entry in the circular buffer to @p initialVal,
     * and adjusts the running sum and sample count accordingly.
     *
     * @param initialVal Value to initialize each buffer element with.
     */
    void init(const T& initialVal) {
        for (std::size_t i = 0; i < N; ++i) {
            buffer[i] = initialVal;
        }
        sum = initialVal * static_cast<T>(N);
        count = N;    ///< Buffer is now full
        index = 0;    ///< Next write will overwrite buffer[0]
    }

    /**
     * @brief Insert a new sample and update the moving average.
     *
     * If the buffer is not yet full, the new sample is appended.
     * Otherwise, the oldest sample is overwritten.
     * The running sum is adjusted accordingly.
     *
     * @param newVal The new sample value to include in the average.
     */
    void update(const T& newVal) {
        if (count < N) {
            buffer[index] = newVal;
            sum += newVal;
            count++;
        } else {
            sum -= buffer[index];
            buffer[index] = newVal;
            sum += newVal;
        }
        index = (index + 1) % N;
    }

    /**
     * @brief Retrieve the current moving average.
     *
     * Computes the average over the number of samples currently in the buffer.
     *
     * @return The average of the stored samples. Returns zero if no samples yet.
     */
    T getAvg() const {
        if (count == 0) {
            return T{};  ///< Avoid division by zero
        }
        return sum / static_cast<T>(count);
    }

private:
    T buffer[N];            /**< Static circular buffer storage */
    std::size_t count;      /**< Number of valid samples (<= N) */
    std::size_t index;      /**< Next write index (0..N-1) */
    T sum;                  /**< Running sum of buffer contents */
};
