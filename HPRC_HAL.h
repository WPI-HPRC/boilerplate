#pragma once

#define SOFTWARE

#ifndef SOFTWARE
#include <Arduino.h>
#include <ArduinoLog.h>

#include <HardwareSerial.h>
#include <SPI.h>
#include <Wire.h>

#include <STM32SD.h>
#include <Servo.h>

// Sensor stuff
#include <LSM6DSO32Sensor.h>
#include <LPS22HBSensor.h>
#include "ASM330LHHSensor.h"
#include <LIS2MDLSensor.h>

#include "PinNames.h"
#include "WInterrupts.h"
#include "pins_arduino.h"
#include "wiring_constants.h"

#else

#include <stdint.h>
#include <stdarg.h>
#include <stddef.h>
#include <string>
#include <string.h>

typedef std::string String;

#define FA_WRITE 1
#define FA_READ 1
#define FA_CREATE_ALWAYS 1

#define LOG_LEVEL_SILENT  0
#define LOG_LEVEL_FATAL   1
#define LOG_LEVEL_ERROR   2
#define LOG_LEVEL_WARNING 3
#define LOG_LEVEL_INFO    4
#define LOG_LEVEL_NOTICE  4
#define LOG_LEVEL_TRACE   5
#define LOG_LEVEL_VERBOSE 6

class Print {
    private:
        int write_error;
        size_t printNumber(unsigned long, uint8_t);
        size_t printFloat(double, uint8_t);
    protected:
        void setWriteError(int err = 1) { write_error = err; }
    public:
        Print() : write_error(0) {}

        int getWriteError() { return write_error; }
        void clearWriteError() { setWriteError(0); }

        virtual size_t write(uint8_t) = 0;
        size_t write(const char *str) {
            if (str == NULL) return 0;
            return write((const uint8_t *)str, strlen(str));
        }
        virtual size_t write(const uint8_t *buffer, size_t size);
        size_t write(const char *buffer, size_t size) {
            return write((const uint8_t *)buffer, size);
        }

        // default to zero, meaning "a single write may block"
        // should be overridden by subclasses with buffering
        virtual int availableForWrite() { return 0; }

        size_t print(const String &);
        size_t print(const char[]);
        size_t print(char);
        size_t print(unsigned char, int = DEC);
        size_t print(int, int = DEC);
        size_t print(unsigned int, int = DEC);
        size_t print(long, int = DEC);
        size_t print(unsigned long, int = DEC);
        size_t print(double, int = 2);

        size_t println(const String &s);
        size_t println(const char[]);
        size_t println(char);
        size_t println(unsigned char, int = DEC);
        size_t println(int, int = DEC);
        size_t println(unsigned int, int = DEC);
        size_t println(long, int = DEC);
        size_t println(unsigned long, int = DEC);
        size_t println(double, int = 2);
        size_t println(void);

        virtual void flush() { /* Empty implementation for backward compatibility */ }
};

typedef void (*printfunction)(Print*, int);


class File {
    public: 
        void printf(const char *format, ...) {
            return;
        }
};

class SD {
    static bool begin() {
        return true;
    }

    static bool exists(const char *fileName) {
        return false;
    }

    static File open(const char *fileName, uint32_t number) {
        return File {};
    }
};

class Logging {
    public: 
        void begin(int level, void *output, bool showLevel = true) {
            return;
        }

        void setPrefix(printfunction p) {
            return;
        };

        void infoln(const char *format, ...) {
            return;
        }

        void errorln(const char *format, ...) {
            return;
        }

        void fatal   (const char *format, ...) {
            return;
        }

        void error   (const char *format, ...) {
            return;
        }

        void warning (const char *format, ...) {
            return;
        }

        void warningln (const char *format, ...) {
            return;
        }

        void notice  (const char *format, ...) {
            return;
        }

        void trace   (const char *format, ...) {
            return;
        }

        void verbose (const char *format, ...) {
            return;
        }
};

Logging Log = Logging{};

void pinMode(uint32_t dwpin, uint32_t dwmode) {
    return;
}

uint32_t pinNametoDigitalPin(uint32_t p) {
    return 0;
}

uint32_t digitalPinToInterrupt(uint32_t p) {
    return p;
}

#endif

