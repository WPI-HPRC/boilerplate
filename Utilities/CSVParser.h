#pragma once

#include "Stream.h"

bool skipCSVRow(Stream *f) {
    char buff[1024];
    size_t n = f->readBytesUntil('\n', buff, 1024);
    return n != 0;
}

// buffer should be right size
bool loadCSVRow(Stream *f, int numVals, float *buff) {
    char rowBuff[1024];
    size_t n = f->readBytesUntil('\n', rowBuff, 1024);

    if (n == 0) {
        return false;
    }

    rowBuff[n] = '\0';
    char *fStart = rowBuff;
    char *end = NULL;
    

    for (size_t i = 0; i < numVals; i++) {
        buff[i] = strtof(fStart, &end);
        if (*end == '\0' && i < numVals - 1) {
            return false;
        }

        fStart = end + 1;
    }
    return true;
}
