/****************************************************************************************
 * Basic Td5 ECU communication
 * Logic from:
 * - LRDuinoTD5 - https://github.com/BennehBoy/LRDuinoTD5/blob/master/td5comm.cpp
 * - BinOwl_Td5Gauge - https://github.com/k0sci3j/BinOwl_Td5Gauge/blob/main/main/KLine.cpp
 * - Td5 keygen - https://github.com/pajacobson/td5keygen
 * - Ekaitza_Itzali - https://github.com/EA2EGA/Ekaitza_Itzali/blob/master/main.py
 *
 * Hardware 5v <-> 12v level shifter using LM393:
 * https://www.netcult.ch/elmue/HUD%20ECU%20Hacker/VAG%20KKL%20Circuit.png
 * https://github.com/Obeisance/Arduino_OBD_interface_for_Torque
 * 
 * Animated bar graph from Upir - https://wokwi.com/projects/333991258811794002
 * 
 * K-Line Communication Notes:
 * - TD5 ECU uses K-line, as defined in ISO 14230.
 * - The K-line is a single wire and the Arduino's RX and TX are both on it, 
 *   so you *will* see your TX echoed back on RX!
 * - Each time you send a command, it's also stored in the RX buffer.
 * - Therefore you need to ignore the same number of bytes sent in order to get to the response.
 * - The last byte in a command (i.e. request) sequence is the checksum.
 * 
 ***************************************************************************************/


#include <Arduino.h>

#include <LiquidCrystalIO.h>
#include <IoAbstractionWire.h>
#include <Wire.h>
LiquidCrystalI2C_RS_EN(lcd, 0x27, false)

#include "gauge.h"

#include "keygen.h"

// for debugging
// code branches -- read a digital pin on boot and branch on that
// set affected digital pin internal pullup, so they are high by default
// and the selected pin just needs to be grounded, and read low to activate that code branch
// ignore case when > 1 pin is grounded, this is for testing only
// change code path by grounding Pin 2 as implemented 
int mode = 0;


/****************************************************************************************
 * ISO 14230 Timings:
 * - P1 Inter byte time for ECU response: default 0ms, min 0ms, max 20ms 
 * - P2 Time between tester request and ECU response or two ECU responses: default 25ms, min 0ms, max 50ms
 * - P3 Time between end of ECU responses and start of new tester request: default 55ms, min 0ms, max 5000ms
 * - P4 Inter byte time for tester request: default 5ms, min 0ms, max 20ms
 * 
 ***************************************************************************************/
#define REQUEST_DELAY        55  // P3 default or min
//TODO:
//#define INTER_BYTE_DELAY      1  // P4 default or min
//#define ECU_RESPONSE_TIMEOUT 50  // P2 max


/****************************************************************************************
 * Td5 PIDs (Parameter IDs, used to request data from the ECU) lslsls
 * Last byte is checksum, precomputed for the common PIDs.
 * Only calculating the key response checksum dynamically.
 * 
 * PIDs are 4 bytes in length, except for Init at 5 bytes, and Key Response at 6 bytes. 
 * These include the checksum byte.
 * 
 * 
 * On the ECU Response:
 * Some reponses, such as "Fuelling", return multiple values. 
 * Not all values are understood! So only keep and use the documented ones.
 * Of the discovered/available PIDs, not all of them are used!
 * 
 * 
 * PID scheduling:
 * ECU comm is "single-threaded". All PIDs have to be sent to the ECU sequentially, 
 * and only after receiveing a response for the previous PID.
 * Therefore the more PIDs we request, the slower turnaround it would be for all the PIDs.
 * 
 * So a PID timer could be used by the main UI loop to determine how often a particular PID is called, 
 * say by using a typical millis loop, to check the time elapsed.
 * Or better still, create a set of of configs that cycles through a set of PIDs to request, 
 * and the results array is sent to the UI.
 * 
 ***************************************************************************************/
const byte init_req[] = {0x81, 0x13, 0xF7, 0x81, 0x0C};  // ECU Init
const byte diag_req[] = {0x02, 0x10, 0xA0, 0xB2};  // ECU Diagnostic Request
const byte seed_req[] = {0x02, 0x27, 0x01, 0x2A};  // ECU Seed Request
const byte  rpm_req[] = {0x02, 0x21, 0x09, 0x2C};  // Engine RPM
const byte  aap_req[] = {0x02, 0x21, 0x23, 0x46};  // Ambient Air Pressure
const byte  map_req[] = {0x02, 0x21, 0x1C, 0x3F};  // Returns both Mass Air Flow (MAF) and Manifold Absolute Pressure (MAP)
//const byte vbat_req[] = {0x02, 0x21, 0x10, 0x33};  // Battery Voltage
//const byte fuelling[] = {0x02, 0x21, 0x1D, 0x40};  // Fuelling... Returns 20 params, see https://github.com/BennehBoy/LRDuinoTD5/blob/master/td5comm.cpp
//const byte speed_req[] = {0x02, 0x21, 0x0D, 0x30};
//const byte temp_req[] = {0x02, 0x21, 0x1A, 0x3D};


/*
 * Read ECU response from serial buffer

 * received: the buffer to read from
 * ignore: skip the first n bytes of the received array, as this is the request transmitted
 * ans_size: size of the expected response from the ECU
 * 
 * TODO: should also check the ECU response checksum
 * TODO: timeout if ECU does not / takes too long to respond
*/
void get_ans(byte *received, byte ignore, byte ans_size) {
  byte ser_i = 0, received_i = 0, tmp = 0;

  while (ser_i < ignore + ans_size) {
    if (Serial.available() > 0) {
      // read the incoming bytes
      tmp = Serial.read();
      // save what we received
      if (ser_i >= ignore) {
        received[received_i] = tmp;
        received_i++;
      }
      ser_i++;
    }
  }
}


/* 
 * Calculate checksum
 *
 * inspired by SternOBDII\code\checksum.c
 * also see https://blog.perquin.com/prj/obdii/
 *
 * When the byte overflows it means that once the value reaches 256, it will go back to 0.
 * Therefore, by letting the crc byte overlow, we achieved the modulo 256 behaviour.
 * Similarly this could be accomplished (as BinOwl) by summing up the bytes sent, then bitwise AND with 0xff;
 * or by sum % 256 used in Ekaitza_Itzali
 */ 
byte checksum(byte *data, byte len) {
  byte crc = 0;

  for (byte i = 0; i < len; i++)
    crc = crc + data[i];
  return crc;
}




/****************************************************************************************
 * Setup the ECU so that we can send PID request to it after this
 * Steps:
 * 1. Bit-bang ISO 14230 fast init
 * 2. Switch Arduino serial port to 10400 baud
 * 3. Authenticate with the ECU with the seed-key challenge
 ***************************************************************************************/

 /*
  * ISO 14230 fast init sequence
  * Bit-bang the Idle-HI, then LO-HI sequence as in the spec
  */
void fast_init() {
  // Setup the pins
  pinMode(1, OUTPUT);  // Pin 1 is also serial TX on the Arduino Pro Mini

  digitalWrite(1, HIGH);
  delay(2000);  // idle time prior to transmission, 'W5' minimum is actually 300ms; but 'P3max' timeout is 5000ms
  digitalWrite(1, LOW);
  delay(25);
  digitalWrite(1, HIGH);
  delay(25);
}

/*
 * Start Arduino serial and authenticate with ECU
 */
void ecu_auth() {
  byte response[8];
  keyBytes_t myKey;

  lcd.setCursor(0, 1);
  lcd.print("Init...");

  // Initialise comms
  fast_init();

  // Begin ECU OBD communication
  Serial.begin(10400);
  Serial.write(init_req, sizeof(init_req));
  get_ans(response, 5, 5);
  delay(REQUEST_DELAY);

  Serial.write(diag_req, sizeof(diag_req));
  get_ans(response, 4, 3);
  delay(REQUEST_DELAY);

  Serial.write(seed_req, sizeof(seed_req));
  get_ans(response, 4, 6);
  delay(REQUEST_DELAY);
  
  // the ECU should respond with the seed values
  // the prefix is: response[0] = 0x04, response[1] = 0x67, response[2] = 0x01
  // response[3] and response[4] contain the seed values
  // response[5] is the checksum for this sequence sent by the ECU
  myKey.high_byte = response[3];
  myKey.low_byte = response[4];
  keyGenerate(&myKey);

  // this is the response sent back to the ECU
  // 04 27 02 is the prefix, 
  // then 2 bytes for the key, and 1 byte for checksum
  // last 3 bytes is computed below
  byte response_key[6] = {0x04, 0x27, 0x02, 0x00, 0x00, 0x00};
  response_key[3] = myKey.high_byte;
  response_key[4] = myKey.low_byte;

  response_key[5] = checksum(response_key, 5);

  lcd.setCursor(0, 1);
  lcd.print("Sending key...");
  lcd_auth_update(response[3], response[4], response_key[3], response_key[4], response_key[5]);

  Serial.write(response_key, sizeof(response_key));
  get_ans(response, 6, 4);

  lcd.setCursor(0, 1);
  lcd.print("Waiting for ECU...");

  // ECU auth success response string from https://github.com/pajacobson/td5keygen
  if (response[0]==0x02 && response[1]==0x67 && response[2]==0x02 && response[3]==0x6B) {  
    lcd.setCursor(17, 0);
    lcd.print("OK!");
    
    delay(REQUEST_DELAY);
    // Done, this function ends and we continue to the main loop
  }
  //TODO: If not positive response, should reset
}


/*
 * Show seed, key, and checksum. Only useful for debugging.
 */
void lcd_auth_update(byte sh, byte sl, byte kh, byte kl, byte c) {
  lcd.setCursor(0, 2);
  lcd.print("S: ");
  lcd.print(sh, HEX);
  lcd.print(" ");
  lcd.print(sl, HEX);
  lcd.print(" K:");
  lcd.print(kh, HEX);
  lcd.print(" ");
  lcd.print(kl, HEX);
  lcd.setCursor(0, 3);
  lcd.print("C:");
  lcd.print(c, HEX);
}


/*
 * Setup the HD44780 LCD connected through i2c backpack
 * Also setup the custom characters used to draw the graphs, see gauge.h
 */
void lcd_setup() {
  Wire.begin();  // for i2c lcd, this must be called first.
  lcd.begin(20, 4);  // set up the lcd columns and rows, must be called.
  lcd.configureBacklightPin(3);  // most i2c backpacks have the backlight on 'P3' of the i2c expander chip. this is physically pin 7 on the PCF8574.
  lcd.backlight();  // switch on lcd backlight
  
  lcd.createChar(7, gauge_empty);   // middle empty gauge  // was byte 7
  lcd.createChar(1, gauge_fill_1);  // filled gauge - 1 column
  lcd.createChar(2, gauge_fill_2);  // filled gauge - 2 columns
  lcd.createChar(3, gauge_fill_3);  // filled gauge - 3 columns
  lcd.createChar(4, gauge_fill_4);  // filled gauge - 4 columns
  // 0-7, so 5, 6, and 0 still available
  // beware in strings, byte 0 terminates it!

  lcd.clear();
  lcd.print("Td5 Spy ");
  //lcd.print(__DATE__);  // Build date, useful for debugging
  //lcd.print(__TIME__);  // Build time, useful for debugging
}




/****************************************************************************************
 * Animated bar graph for showing engine revs
 * 
 * Range needed is 500 to 5000 rpm
 * Td5 engines idle at 745 rpm and governed to 4850 rpm max
 * 
 ***************************************************************************************/
void lcd_rev_counter(int revs) {
  char rpms[5] = {0};
  const int gauge_size_chars = 20;  // width of the gauge in number of characters
  char gauge_string[gauge_size_chars + 1] = {0};  // string that will include all the gauge character to be printed
  int move_offset = 0;  // used to shift bits for the custom characters

  //float units_per_pixel = (gauge_size_chars * 5.0) / 5000.0;  // Every character is 5px wide. RPM range is 0-5000
  const float units_per_pixel = 0.02;  // We aren't dynamically resizing, so could pre-calc this

  // rev_gauge value converted to pixel width
  // the minus 2 shifts the centre to the middle column of the pixel block
  // the minus 10 allows us to ignore the first 500 rpm
  int value_in_pixels = round(revs * units_per_pixel) - 2 - 10;
  move_offset = 4 - ((value_in_pixels-1) % 5);  // offseting the pixels for the smooth filling
  
  for (int i = 0; i < gauge_size_chars; i++) {  // set all the characters for the gauge
    if (value_in_pixels <= i*5) {gauge_string[i] = byte(7);}   // empty character
    else if (value_in_pixels > i*5 && value_in_pixels < (i+1)*5) {gauge_string[i] = byte(5-move_offset);}  // the 'moving needle'
    else {gauge_string[i] = byte(255);}                        // filled character
  }
  
  lcd.setCursor(0, 1);
  lcd.print(gauge_string);  // display the gauge

  snprintf(rpms, 5, "%4d ", revs);
  lcd.setCursor(16, 0);
  lcd.print(rpms);  // show numeric rpm on the top-right corner
}

/*
 * Setup the tick marks for the rev counter
 * No need to redraw this as LCD only refreshes areas that have been overwritten
 * 
 * TODO tick marks
 */
void lcd_setup_rev() {
  lcd.clear();
  lcd.print(" 1   2   3   4");
}




/****************************************************************************************
 * Turbo Boost
 * 
 * ECU retuns both Manifold Absolute Pressure (MAP) and Ambient Air Pressure (AAP)
 * These are given as 5-digit integers.
 * Thus can be treated as 1/10000 of a bar.
 * 
 * 1 bar = 100,000 Pa (mostly)
 * So each returned unit is 10 Pa
 * Therefore to convert to hPa, divide returned value by 10 which will give 4 digits, 
 * thus giving ambient pressure of ~1000 hPa, which is right.
 *
 ****************************************************************************************/

/*
 * Show both MAP and AAP
 */
void lcd_show_pressures(int map, int aap) {
  char pressures[21] = {0};
  
  snprintf(pressures, 21, "M: %5d  A: %5d  ", map, aap);
  lcd.setCursor(0, 2);
  lcd.print(pressures);
}

/*
 * Animated bar graph boost gauge
 */
void lcd_boost_gauge(int map, int aap) {
  char boosts[5] = {0};
  const int gauge_size_chars = 15;  // width of the gauge in number of characters
  char gauge_string[gauge_size_chars + 1] = {0};  // string that will include all the gauge character to be printed
  int move_offset = 0;  // used to shift bits for the custom characters

  int relative_pressure = map - aap;
  int gauge_pressure = round(max(relative_pressure, 1) / 10.0);  // keep above zero, ignore vacuum conditions
  
  //float units_per_pixel = (gauge_size_chars * 5.0) / 1500.0;  // Every character is 5px wide. Boost range is 0-1500, ECU limits to ~1420
  const float units_per_pixel = 0.05;  // pre-calculate the above
  int value_in_pixels = round((gauge_pressure) * units_per_pixel);  // gauge_pressure value converted to pixel width
  move_offset = 4 - ((value_in_pixels-1) % 5);  // offseting the pixels for the smooth filling
  
  char problem[21] = {0};
  for (int i = 0; i < gauge_size_chars; i++) {  // set all the characters for the gauge
    if (value_in_pixels <= i*5) {gauge_string[i] = byte(7);}   // empty character
    else if (value_in_pixels > i*5 && value_in_pixels < (i+1)*5) {gauge_string[i] = byte(5-move_offset);}  // the 'moving needle'
    else {gauge_string[i] = byte(255);}                        // filled character
  }

  lcd.setCursor(0, 3);
  lcd.print(gauge_string);  // display the gauge

  snprintf(boosts, 5, "%4d", gauge_pressure);
  lcd.setCursor(16, 3);
  lcd.print(boosts);
}




/* Used only to debug graphics. Does not connect to ECU. */
void cp_high_setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  lcd_setup_rev();
}

void cp_high_loop() {
  lcd_rev_counter(3000);
  delay(REQUEST_DELAY);
  delay(REQUEST_DELAY);
  lcd_show_pressures(10140, 10130);
  lcd_boost_gauge(10140, 10130);
  delay(REQUEST_DELAY);
}


/* Run mode. Keep Pin 2 connected to GND */
void cp_low_setup() {
  ecu_auth();
  lcd_setup_rev();
}

void cp_low_loop() {
  byte response[50];
  int rpm = 0;
  int map = 0;
  int aap = 0;

  Serial.write(rpm_req, sizeof(rpm_req));
  get_ans(response, 4, 6);
  rpm = (response[3] * 256) + response[4];
  
  lcd_rev_counter(rpm);
  delay(REQUEST_DELAY);
  
  Serial.write(map_req, sizeof(map_req));
  get_ans(response, 4, 12);
  map = ((response[3] * 256) + response[4]);  // Given as 1/10000 bar, returns 5 digits
  //maf = ((response[7] * 256) + response[8]) / 10.0;  // no need for MAF data
  delay(REQUEST_DELAY);
  
  Serial.write(aap_req, sizeof(aap_req));
  get_ans(response, 4, 8);
  aap = ((response[3] * 256) + response[4]);  // 1/10000 bar, returns 5 digits
  
  lcd_show_pressures(map, aap);
  lcd_boost_gauge(map, aap);
  delay(REQUEST_DELAY);
}


/* Arduino defined setup section, one-shot on startup */
void setup() {
  pinMode(2, INPUT_PULLUP);

  lcd_setup();

  if(digitalRead(2) == HIGH) {
    mode = HIGH;
    cp_high_setup();
  } else {
    mode = LOW;
    cp_low_setup();
  }
}

/* Arduino defined main logic loop section, repeats forever */
void loop() {
  if(mode == HIGH) {
    cp_high_loop();
  } else {
    cp_low_loop();
  }
}
