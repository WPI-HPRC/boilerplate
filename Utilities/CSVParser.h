#pragma once

#include "Stream.h"

bool skipCSVRow(Stream *f);

// buffer should be right size
bool loadCSVRow(Stream *f, int numVals, float *buff);
