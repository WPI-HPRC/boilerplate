#pragma once

/**
 * @brief Abstract class representing a state in a state machine.
 * @param Context can be any type storing user data
 * @param StateId should be an enum or integer type, with a unique one passed
 * into the `State` constructor for each subclass
 * @param millis a callable that gives the number of
 * milliseconds elapsed since boot. This function can be made to have state by
 * making it a struct with `operator()`, potentially passing it in as a global
 * singleton if needed
 * @note Templates should not bleed into subclasses, only the base class needs
 * them.
 */
template <typename Context, typename StateId, typename MillisFn> class TState {
  public:
    /**
     * @brief Code to be run once when the state begins.
     */
    void initialize() {
        this->startTime = millis();
        initialize_impl();
    }

    /**
     * @brief Code to be run every iteration of the loop while the rocket is in
     * this state.
     * @return The pointer to the next state or nullptr if the state has not
     * changed.
     */
    TState *loop() {
        long long now = millis();
        // These values may be used in the state code
        this->currentTime = now - this->startTime;
        this->deltaTime = now - this->lastLoopTime;
        this->lastLoopTime = now;
        this->loopCount++;

        TState *next = loop_impl();

#ifndef NO_TRANSITION
        return next;
#else
        return nullptr;
#endif
    }

    /**
     * @brief Get the ID of this state
     */
    StateId getId() { return id; }

    virtual ~TState() {}

  protected:
    //! @note Constructor to be called from subclasses (subclass constructors
    //! should only need to take in ctx)
    TState<Context, StateId, MillisFn>(StateId id, MillisFn millis, Context *ctx)
        : ctx(ctx), id(id), millis(millis) {}
    //! @brief number of milliseconds since the initialize call
    long long currentTime = 0;
    //! @brief number of milliseconds since the last loop call
    long long deltaTime = 0;
    //! @brief loop count since initialization
    long long loopCount = 0;
    //! @brief "global" context object
    Context *ctx;

  private:
    //! @brief number of milliseconds from boot to the initialize call
    long long startTime = 0;
    //! @brief number of milliseconds from boot to the previous loop call
    long long lastLoopTime = 0;
    //! @brief id of the current state
    StateId id;
    //! @brief function to get the time, not needed in subclasses
    MillisFn millis;

    virtual void initialize_impl() = 0;
    virtual TState *loop_impl() = 0;
};
