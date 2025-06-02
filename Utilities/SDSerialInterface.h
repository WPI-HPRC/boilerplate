#include <Arduino.h>
#include "Context.h"

// Call near start of init
void setupSDInterface(Context *ctx);

// Call in loop, will do nothing if setupSDInterface was not called
void handleSDInterface(Context *ctx);
