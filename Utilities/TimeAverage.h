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
        : sum(T{}), count(0), index(0), bufferFull(false) {}

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
        if (!bufferFull) {
            buffer[index] = newVal;
            sum += newVal;
            count++;
            if (count >= N) {
                bufferFull = true;
            }
        } else {
            sum -= buffer[index];
            buffer[index] = newVal;
            sum += newVal;
        }
        index = (index + 1) % N;
    }

    /**
     * @brief clears buffer and current count of values in the buffer
     */

     void clearBuffer() {
        bufferFull = false;
        count = 0;
        sum = 0;
     }

    /**
     * @brief Checks if the buffer is full for the given size
     * 
     * If the buffer has not been filled with values for the given N, returns false
     * 
     */

     bool isBufferSaturated() {
        return bufferFull;
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
    bool bufferFull;        /**< bool if the buffer has been filled with independent values */
};
