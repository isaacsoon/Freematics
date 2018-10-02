#include <SPI.h>
#include <FS.h>
#include <SD.h>
#include <SPIFFS.h>
#include <httpd.h>
#include <FreematicsPlus.h>
#include "speedlogger.h"
#include "config.h"
//#include "driver/uart.h"

#define RAM_CACHE_SIZE 1024 /* bytes */

// logger states
#define STATE_STORAGE_READY 0x1
#define STATE_OBD_READY 0x2
#define STATE_GPS_READY 0x4
#define STATE_MEMS_READY 0x8
#define STATE_NET_READY 0x10
#define STATE_NET_CONNECTED 0x20
#define STATE_WORKING 0x40
#define STATE_OBD_FOUND 0x80
#define STATE_STANDBY 0x100

class State {
public:
  bool check(uint16_t flags) { return (m_state & flags) == flags; }
  void set(uint16_t flags) { m_state |= flags; }
  void clear(uint16_t flags) { m_state &= ~flags; }
private:
  uint16_t m_state = 0;
};

FreematicsESP32 sys;
//CStorageRAM cache;
State state;

COBD* obd = 0;

uint32_t pidErrors = 0;


typedef struct {
  byte pid;
  byte tier;
  int16_t value;
  uint32_t ts;
} PID_POLLING_INFO;

PID_POLLING_INFO obdData[]= {
  {PID_SPEED, 1},
  {PID_RPM, 1},
  {PID_THROTTLE, 1},
  {PID_ENGINE_LOAD, 1},
  {PID_FUEL_PRESSURE, 2},
  {PID_TIMING_ADVANCE, 2},
  {PID_COOLANT_TEMP, 3},
  {PID_INTAKE_TEMP, 3},
};

/*******************************************************************************
  Initializing all components and network
*******************************************************************************/
bool initialize()
{
  state.clear(STATE_WORKING);
  
#if ENABLE_OBD
  // initialize OBD communication
  if (!state.check(STATE_OBD_READY)) {
    timeoutsOBD = 0;
    Serial.print("OBD...");
    if (obd->init()) {
      Serial.println("OK");
      state.set(STATE_OBD_READY | STATE_OBD_FOUND);
    } else {
      Serial.println("NO");
      if (state.check(STATE_OBD_FOUND)) {
        // if OBD was ever connected, require connection
        return false;
      }
    }
  }
#endif



  // re-try OBD if connection not established
#if ENABLE_OBD
  if (!state.check(STATE_OBD_READY) && obd->init()) {
    state.set(STATE_OBD_READY | STATE_OBD_FOUND);
  }
  if (state.check(STATE_OBD_READY)) {
    char buf[128];
    if (obd->getVIN(buf, sizeof(buf))) {
      strncpy(vin, buf, sizeof(vin) - 1);
      Serial.print("VIN:");
      Serial.println(vin);
    }
    int dtcCount = obd->readDTC(dtc, sizeof(dtc) / sizeof(dtc[0]));
    if (dtcCount > 0) {
      Serial.print("DTC:");
      Serial.println(dtcCount);
    }
  }
#endif


  // check system time
  time_t utc;
  time(&utc);
  struct tm *btm = gmtime(&utc);
  if (btm->tm_year > 100) {
    // valid system time available
    char buf[64];
    sprintf(buf, "%04u-%02u-%02u %02u:%02u:%02u",
      1900 + btm->tm_year, btm->tm_mon + 1, btm->tm_mday, btm->tm_hour, btm->tm_min, btm->tm_sec);
    Serial.print("UTC:");
    Serial.println(buf);
  }


  state.set(STATE_WORKING);
  return true;
}

void setup()
{
//    char buf[32];

    delay(1000);

    // initialize USB serial
    Serial.begin(115200);
    Serial.print("ESP32 ");
    Serial.print(ESP.getCpuFreqMHz());
    Serial.print("MHz ");
    Serial.print(getFlashSize() >> 10);
    Serial.println("MB Flash");

    // init LED pin
    pinMode(PIN_LED, OUTPUT);
    pinMode(PIN_LED, HIGH);

    // startup initializations
    sys.begin();
#if ENABLE_OBD
    while (!(obd = createOBD()));
    Serial.print("OBD Firmware Version ");
    Serial.println(obd->getVersion());
#endif

    // allocate for data cache
 //   cache.init(RAM_CACHE_SIZE);

    // initializing components
    initialize();
    Serial.print("isaac after init");

    digitalWrite(PIN_LED, LOW);
}

//byte pid;
//byte pid = obdData[1].pid;  //rpm

void loop()
{
    int value;

  if (state.check(STATE_OBD_READY)) {

    obd->readPID(PID_SPEED, value);
    Serial.print("Speed: ");
    Serial.print (value);
    Serial.println (" km/h");

    obd->readPID(PID_RPM, value);
    Serial.print("RPM: ");
    Serial.print (value);
    Serial.println(" rpm");

    obd->readPID(PID_COOLANT_TEMP, value);
    Serial.print("Coolant Temp: ");
    Serial.print (value);
    Serial.println(" celcius?");
  } else {
    Serial.println("OBD not ready");
  }

}