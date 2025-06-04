#pragma once

#include "Print.h"

#define MAKE_LOGGABLE(LOG_DESC)                                                \
    void logFieldName(Print &p, size_t idx) override {                         \
        switch (idx) {                                                         \
            LOG_DESC(NAME_CASE)                                                \
        default:                                                               \
            break;                                                             \
        }                                                                      \
    }                                                                          \
                                                                               \
    void logFieldValue(Print &p, size_t idx) override {                        \
        switch (idx) {                                                         \
            LOG_DESC(VALUE_CASE)                                               \
        default:                                                               \
            break;                                                             \
        }                                                                      \
    }

#define NAME_CASE(idx, name, valueExpr)                                        \
    case idx:                                                                  \
        p.print(name);                                                         \
        break;

#define VALUE_CASE(idx, name, valueExpr)                                       \
    case idx:                                                                  \
        valueExpr;                                                             \
        break;

#define NUM_FIELDS(LOG_DESC) LOG_DESC(COUNT)

#define COUNT(idx, name, valueExpr) +1

class Loggable {
  public:
    uint32_t debugLog(Print &p, uint32_t lastLoggedTime = 0) {
        for (size_t i = 0; i < numFields; i++) {
            if (lastLoggedTime < dataUpdatedAt()) {
                logFieldName(p, i);
                p.print(": ");
                logFieldValue(p, i);

                if (i < numFields - 1) {
                    p.print(", ");
                }
            }
        }

        p.println();

        return dataUpdatedAt();
    }

    uint32_t logCsvRow(Print &p, uint32_t lastLoggedTime = 0) {
        for (size_t i = 0; i < numFields; i++) {
            if (lastLoggedTime < dataUpdatedAt()) {
                logFieldValue(p, i);
            }

            if (i < numFields - 1) {
                p.print(",");
            }
        }

        return dataUpdatedAt();
    }
    void logCsvHeader(Print &p) {
        for (size_t i = 0; i < numFields; i++) {
            logFieldName(p, i);

            if (i < numFields - 1) {
                p.print(",");
            }
        }
    }

  protected:
    Loggable(size_t numFields) : numFields(numFields) {}

  private:
    size_t numFields;

    virtual void logFieldName(Print &p, size_t idx) = 0;
    virtual void logFieldValue(Print &p, size_t idx) = 0;
    virtual uint32_t dataUpdatedAt() = 0;
};
