#include "Print.h"

#ifndef MAX_LOG_CONTEXTS
  #define MAX_LOG_CONTEXTS 5
#endif

enum class LogLevel {
  TRACE = 0,
  DEBUG,
  INFO,
  WARN,
  ERROR,
  NONE,
};

enum class ExtraLog {
  ALL,
  ONLY_PREFIX,
  ONLY_SUFFIX,
  NONE,
};

class Logging {
  public:
    Logging();

    void begin(Print *p, LogLevel lvl) {
      this->p = p;
      this->selectedLevel = lvl;
    }

    void printf(LogLevel lvl, const char *fmt, ...) {
      if (selectedLevel <= lvl) {
        va_list args;
        va_start(args, fmt);
        p->vprintf(fmt, args);
        va_end(args);
      }
    }
  private:
    Print *p;
    LogLevel selectedLevel;
};

extern Logging Log;
