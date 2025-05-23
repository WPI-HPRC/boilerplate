#pragma once

#include <cstddef>
#include <cstdint>
#include <cstdlib>

#include <Arduino.h>

template <typename T> class TimedPointer;

// Void ptr is specialized since we can't know the size at compile time.
template <> class TimedPointer<void> {
  public:
    TimedPointer(size_t size)
        : ptr(malloc(size)),
          lastUpdated((uint32_t *)malloc(sizeof(lastUpdated))) {}

    /* @brief The following method probably does not work as expected, since
     `void *` cannot be dereferenced. Thus the "dereference" operators actually
     just return the pointer, which could then be casted to a useable type.
     __However__ the intended API is to first static_cast to a TimedPointer<T>,
     where all of the dereference operators work normally.
    */
    void *operator*() {
        *lastUpdated = millis();
        return ptr;
    }

    /* @brief The following method probably does not work as expected, since
     `void *` cannot be dereferenced. Thus the "dereference" operators actually
     just return the pointer, which could then be casted to a useable type.
     __However__ the intended API is to first static_cast to a TimedPointer<T>,
     where all of the dereference operators work normally.
    */
    const void *operator*() const { return ptr; }

    uint32_t getLastUpdated() { return *lastUpdated; }

  private:
    void *ptr;
    uint32_t *lastUpdated;

    template <typename T> friend class TimedPointer;
};

template <typename T> class TimedPointer {
  public:
    TimedPointer()
        : ptr((T *)malloc(sizeof(T))),
          lastUpdated((uint32_t *)malloc(sizeof(lastUpdated))) {}

    //! @brief Use static_cast instead of this function.
    TimedPointer(TimedPointer<void> &other)
        : ptr(const_cast<T *>(static_cast<const T *>(other.ptr))),
          lastUpdated(other.lastUpdated) {}

    T *operator->() {
        *lastUpdated = millis();
        return ptr;
    }
    T &operator*() {
        *lastUpdated = millis();
        return *ptr;
    }

    const T *operator->() const { return ptr; }
    const T &operator*() const { return *ptr; }

    uint32_t getLastUpdated() { return *lastUpdated; }

  private:
    T *ptr;
    uint32_t *lastUpdated;
};
;
