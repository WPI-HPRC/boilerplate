#include "SDSerialInterface.h"
#include "SPI.h"
#include "config.h"

bool inSdInterfaceMode = false;
HardwareTimer sdInterfaceStatusTimer(TIM6);

template <int ledPin> void blinkLED() {
    static bool state = false;

    digitalWrite(ledPin, state);
    state = !state;
}


void setupSDInterface(Context *ctx) {
  inSdInterfaceMode = true;

  Serial.begin(230400);
  SPI.setSCLK(SD_SCLK);
  SPI.setMISO(SD_MISO);
  SPI.setMOSI(SD_MOSI);
  SPI.begin();

  ctx->sd.begin(SD_CS, SD_SPI_SPEED);

  pinMode(PG15, OUTPUT);

  sdInterfaceStatusTimer.attachInterrupt(blinkLED<PG15>);
  sdInterfaceStatusTimer.setOverflow(50 * 1000, MICROSEC_FORMAT);

  while (!Serial) {
      delay(5);
  }

  digitalWrite(PG15, HIGH);
}

void handleSDInterface(Context *ctx) {
  if (inSdInterfaceMode) { 
    if (Serial.available() > 0) {
        char cmd = Serial.read();

        switch (cmd) {
        case 'L': {
            // ls
            sdInterfaceStatusTimer.resume();

            File root = ctx->sd.open("/");
            File f;
            char fname[256];

            while ((f = root.openNextFile())) {
                size_t len = f.getName(fname, 256);
                Serial.write(fname, len + 1);

                f.close();
            }

            root.close();

            digitalWrite(PG15, HIGH);
            sdInterfaceStatusTimer.pause();
            break;
        }
        case 'R': {
            // rm
            sdInterfaceStatusTimer.resume();

            while (!Serial.available()) {
                delay(1);
            }
            size_t fnameLen = Serial.read();
            char fname[256];
            size_t bytesRead = 0;

            while (bytesRead < fnameLen) {
                while (Serial.available()) {
                    fname[bytesRead++] = Serial.read();
                }
            }
            fname[bytesRead] = '\0';

            ctx->sd.remove(fname);

            digitalWrite(PG15, HIGH);
            sdInterfaceStatusTimer.pause();
            break;
        }
        case 'M': {
            // mv
            sdInterfaceStatusTimer.resume();

            while (!Serial.available()) {
                delay(1);
            }
            size_t fnameLen = Serial.read();
            char fname[256];
            size_t bytesRead = 0;

            while (bytesRead < fnameLen) {
                while (Serial.available()) {
                    fname[bytesRead++] = Serial.read();
                }
            }
            fname[bytesRead] = '\0';

            File f = ctx->sd.open(fname);

            fnameLen = Serial.read();
            bytesRead = 0;
            
            while (bytesRead < fnameLen) {
                while (Serial.available()) {
                    fname[bytesRead++] = Serial.read();
                }
            }
            fname[bytesRead] = '\0';

            f.rename(fname);

            f.close();

            digitalWrite(PG15, HIGH);
            sdInterfaceStatusTimer.pause();
            break;
        }
        case 'P': {
            // print
            sdInterfaceStatusTimer.resume();

            while (!Serial.available()) {
                delay(1);
            }
            size_t fnameLen = Serial.read();
            char fname[256];
            size_t bytesRead = 0;

            while (bytesRead < fnameLen) {
                while (Serial.available()) {
                    fname[bytesRead++] = Serial.read();
                }
            }
            fname[bytesRead] = '\0';

            File f = ctx->sd.open(fname);

            if (f) {
                while (f.available()) {
                    Serial.write(f.read());
                }
            } else {
                Serial.print("ERR opening file");
            }

            digitalWrite(PG15, HIGH);
            sdInterfaceStatusTimer.pause();
            break;
        }
        case 'C': {
            // clear
            sdInterfaceStatusTimer.resume();

            File root = ctx->sd.open("/");
            File f;

            while ((f = root.openNextFile())) {
                char fname[256];
                f.getName(fname, sizeof(fname));
                f.close();

                root.remove(fname);
            }

            root.close();

            digitalWrite(PG15, HIGH);
            sdInterfaceStatusTimer.pause();
            break;
        }
        case 'F': {
            // format
            sdInterfaceStatusTimer.resume();

            ctx->sd.format();
            ctx->sd.begin(SD_CS, SD_SPI_SPEED);

            Serial.write((uint8_t)0);

            digitalWrite(PG15, HIGH);
            sdInterfaceStatusTimer.pause();
            break;
        }
        default:
            Serial.print("ERR unknown command");
            break;
        }
    }  
  }
}
