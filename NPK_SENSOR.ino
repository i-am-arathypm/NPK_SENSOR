
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <ModbusMaster.h>

ModbusMaster mode;
int nitrogen, phosphorus, potassium ,err=0 ;
int sensor_pin = 7;
void npk()
{
 
 
uint8_t result = mode.readHoldingRegisters(0x001E, 3);  // Start address, number of registers

  // Check if reading was successful
  if (result == mode.ku8MBSuccess) {
    // Read the received data
    nitrogen = mode.getResponseBuffer(0);
    phosphorus = mode.getResponseBuffer(1);
    potassium = mode.getResponseBuffer(2);
    err=0;
   

    // Print the values to Serial Monitor
//    Serial.print("Nitrogen: ");
//    Serial.println(nitrogen);
//    Serial.print("Phosphorus: ");
//    Serial.println(phosphorus);
//    Serial.print("Potassium: ");
//    Serial.println(potassium);
  } else {
    // Print error message and error code
//    Serial.print("Error reading data. Error code: ");
//    Serial.println(result);
      err=1;
  }

  // Delay before next read
  delay(1000);  // Adjust as per your application's requirement  
}

// LoRaWAN NwkSKey, network session key
// This should be in big-endian (aka msb).
static const PROGMEM u1_t NWKSKEY[16] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

// LoRaWAN AppSKey, application session key
// This should also be in big-endian (aka msb).
static const u1_t PROGMEM APPSKEY[16] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

// LoRaWAN end-device address (DevAddr)
// See http://thethingsnetwork.org/wiki/AddressSpace
// The library converts the address to network byte order as needed, so this should be in big-endian (aka msb) too.
static const u4_t DEVADDR = 0x00000000 ; // <-- Change this address for every node!


// show debug statements; comment next line to disable debug statements
#define DEBUG

// use low power sleep; comment next line to not use low power sleep
#define SLEEP

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 900;

//struct {
//int a;
//int16_t b;
//} mydata;

#ifdef SLEEP
#include "LowPower.h"
bool next = false;
#endif

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

static osjob_t sendjob;

static uint8_t mydata[20];

// Pin mapping
const lmic_pinmap lmic_pins = {
.nss = 6,
.rxtx = LMIC_UNUSED_PIN,
.rst = 5,
.dio = {2, 3, 4},
};

void onEvent (ev_t ev) {
#ifdef DEBUG
//Serial.println(F("Enter onEvent"));
#endif

switch (ev) {
#ifdef DEBUG
case EV_SCAN_TIMEOUT:
//Serial.println(F("EV_SCAN_TIMEOUT"));
break;
case EV_BEACON_FOUND:
//Serial.println(F("EV_BEACON_FOUND"));
break;
case EV_BEACON_MISSED:
//Serial.println(F("EV_BEACON_MISSED"));
break;
case EV_BEACON_TRACKED:
//Serial.println(F("EV_BEACON_TRACKED"));
break;
case EV_JOINING:
//Serial.println(F("EV_JOINING"));
break;
case EV_JOINED:
//Serial.println(F("EV_JOINED"));
break;
case EV_RFU1:
//Serial.println(F("EV_RFU1"));
break;
case EV_JOIN_FAILED:
//Serial.println(F("EV_JOIN_FAILED"));
break;
case EV_REJOIN_FAILED:
//Serial.println(F("EV_REJOIN_FAILED"));
break;
#endif
case EV_TXCOMPLETE:
//Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
if (LMIC.dataLen) {
// data received in rx slot after tx
//Serial.print(F("Data Received: "));
//Serial.write(LMIC.frame + LMIC.dataBeg, LMIC.dataLen);
//Serial.println();
}
// Schedule next transmission
#ifndef SLEEP
os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
#else
next = true;
#endif

break;

#ifdef DEBUG
case EV_LOST_TSYNC:
//Serial.println(F("EV_LOST_TSYNC"));
break;
case EV_RESET:
//Serial.println(F("EV_RESET"));
break;
case EV_RXCOMPLETE:
// data received in ping slot
//Serial.println(F("EV_RXCOMPLETE"));
break;
case EV_LINK_DEAD:
//Serial.println(F("EV_LINK_DEAD"));
break;
case EV_LINK_ALIVE:
//Serial.println(F("EV_LINK_ALIVE"));
break;
default:
//Serial.println(F("Unknown event"));
break;
#endif
}
#ifdef DEBUG
//Serial.println(F("Leave onEvent"));
#endif

}

void do_send(osjob_t* j) {

int a = batterycalc();
digitalWrite(sensor_pin, HIGH);
//delay(3000);
npk();
npk();
npk();
npk();
npk();
     
      mydata[0] = highByte(nitrogen);
      mydata[1] = lowByte(nitrogen);
     
      mydata[2] = highByte(phosphorus);
      mydata[3] = lowByte(phosphorus);

      mydata[4] = highByte(potassium);
      mydata[5] = lowByte(potassium);

      mydata[6] = highByte(err);
      mydata[7] = lowByte(err);

      mydata[8] = highByte(a);
      mydata[9] = lowByte(a);
      
digitalWrite(sensor_pin, LOW);
//mydata.b = soil_moisture();
#ifdef DEBUG
//Serial.println(F("Enter do_send"));
#endif

// Check if there is not a current TX/RX job running
if (LMIC.opmode & OP_TXRXPEND) {
//Serial.println(F("OP_TXRXPEND, not sending"));
} else {
// Prepare upstream data transmission at the next possible time.
LMIC_setTxData2(1, (unsigned char *)&mydata, sizeof(mydata) - 1, 0);
//Serial.println(F("Packet queued"));
}
// Next TX is scheduled after TX_COMPLETE event.
#ifdef DEBUG
//Serial.println(F("Leave do_send"));
#endif

}

//Soil Moisture
//int sensor_pin = A5, moisture;
//const int sensorEn = 8;
//
//int soil_moisture()
//{
//digitalWrite(sensorEn, HIGH);
//delay(200);
//moisture = analogRead(sensor_pin);
//moisture = map(moisture, 850, 350, 0, 100);
//digitalWrite(sensorEn, LOW);
//#ifdef DEBUG
//Serial.println("Moisture = ");
//Serial.print(moisture);
//Serial.print("%");
//#endif
//return moisture;
//}

//Battery Monitor
int batv = 0, bat_status = 0;

int batteryv()
{

int val = analogRead(A3) >> 2; // read the input pin
batv = (val * 1.85 * 4.2 / 255) * 1000; //1.85 is the voltage multiplier
delay(0);
return batv;
}

int batterycalc()
{
int diff = 0;
if (bat_status == 0) { //Baseline calibration
for (int j = 0; j < 5; j++) {
bat_status = bat_status + batteryv();
}
bat_status = bat_status / 5;
}
#ifdef DEBUG
//Serial.println("Battery_status = ");
//Serial.print(bat_status);
#endif
for (int i = 0; i < 5; i++) { //Read current average
batv = batv + batteryv();
}
batv = batv / 5;
diff = abs(bat_status - batv);
#ifdef DEBUG
//Serial.println("Now Avg = ");
//Serial.println(batv);
//Serial.println("Value diff");
//Serial.println(diff);
#endif
if (diff > 25 && diff < 70) { //Define Band Pass
bat_status = batv;
}
batv = 0;
#ifdef DEBUG
//Serial.println("Battery Voltage");
//Serial.println(bat_status);
#endif
return bat_status;
}


void setup() {
  while (!Serial);
  Serial.begin(9600);
  mode.begin(1, Serial);
    delay(100);

//Serial.print("Starting..");
//delay(5000);
pinMode(sensor_pin, OUTPUT);
//Serial.println(F("Enter setup"));

#ifdef VCC_ENABLE
// For Pinoccio Scout boards
pinMode(VCC_ENABLE, OUTPUT);
digitalWrite(VCC_ENABLE, HIGH);
delay(1000);
#endif
// LMIC init
os_init();
// Reset the MAC state. Session and pending data transfers will be discarded.
LMIC_reset();

// Set static session parameters. Instead of dynamically establishing a session
// by joining the network, precomputed session parameters are be provided.
#ifdef PROGMEM
// On AVR, these values are stored in flash and only copied to RAM
// once. Copy them to a temporary buffer here, LMIC_setSession will
// copy them into a buffer of its own again.
uint8_t appskey[sizeof(APPSKEY)];
uint8_t nwkskey[sizeof(NWKSKEY)];
memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
LMIC_setSession (0x13, DEVADDR, nwkskey, appskey);
#else
// If not running an AVR with PROGMEM, just use the arrays directly
LMIC_setSession (0x13, DEVADDR, NWKSKEY, APPSKEY);
#endif

#if defined(CFG_eu868)
LMIC_setupChannel(0, 865062500, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI); // g-band
LMIC_setupChannel(1, 865402500, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI); // g-band
LMIC_setupChannel(2, 865985000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI); // g-band
#elif defined(CFG_us915)
LMIC_selectSubBand(1);
#endif

// Disable link check validation
LMIC_setLinkCheckMode(0);

// TTN uses SF9 for its RX2 window.
LMIC.dn2Dr = DR_SF9;

// Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
LMIC_setDrTxpow(DR_SF7, 14);

// Start job
do_send(&sendjob);
// Wait a maximum of 10s for Serial Monitor
// while (!debugSerial && millis() < 10000);

}

void loop() {

#ifndef SLEEP

os_runloop_once();

#else
extern volatile unsigned long timer0_overflow_count;

if (next == false) {

os_runloop_once();

} else {

int sleepcycles = TX_INTERVAL / 8; // calculate the number of sleepcycles (8s) given the TX_INTERVAL
#ifdef DEBUG
//Serial.print(F("Enter sleeping for "));
//Serial.print(sleepcycles);
//Serial.println(F(" cycles of 8 seconds"));
#endif
//Serial.flush(); // give the serial print chance to complete
for (int i = 0; i < sleepcycles; i++) {
// Enter power down state for 8 s with ADC and BOD module disabled
LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
//LowPower.idle(SLEEP_8S, ADC_OFF, TIMER2_OFF, TIMER1_OFF, TIMER0_OFF, SPI_OFF, USART0_OFF, TWI_OFF);

// LMIC uses micros() to keep track of the duty cycle, so
// hack timer0_overflow for a rude adjustment:
cli();
timer0_overflow_count += 8 * 64 * clockCyclesPerMicrosecond();
sei();
}
#ifdef DEBUG
//Serial.println(F("Sleep complete"));
#endif
next = false;
// Start job
do_send(&sendjob);
}

#endif

}
