//  Developed from my original OLED_BMS_u8g2 sketch
//  Added two additional pages Amperage and Voltage Analog needles
//  Intended for the cabin of the boat for more simplistic classic indications
//  Original pages are the exact same
//
//
//  WRITTEN BY:
//  Tom Storebø 7th of May 2019
//  GAUGE PAGE DEVELOPED FROM:
//  Rudi Imbrechts "Analog Gauge v1.02" - 21st of December 2015
//
//  Change log
//  25/05/19  5A scale amp needle, 50 degree needle movements amp and volt, scale lines voltmeter.
//  01/06/19  Temp hi and lo on bottom right text screen, whilst cycles removed and Ah taken its place.
//  03/06/19  Shifted the middle of text page 4 pixels left; Added BMS status to flags as well as change name from faults to flag. Various pos adjustments on text page.
//  11/06/19  Swapped low and high cell in bars
//  24/06/19  Edited CANBus data from Orion Jr as follows: removed BMS calculations for processCanData()[0], DCL, CCL; set Big Endian byte order on BMS status. Changed to 1 byte hex (0x01) instead of 2 byte (0x0001) in BMS status on text page.
//  27/06/19  Added delayed amperage reading for more stability in watt and time indications and changed processCanData()[1] from float to int.
//  30/06/19  Added rxBuf for "ry" in gauge for displaying "sun" only when relay enabled; added bms status to new variable "fs" for "wrench" icon display; adjusted "0A" DCH & CHG limits.
//  09/07/19  Replaced counter with total pack cycles
//  27/07/19  Added counter and adjusted cycles on text page; Adjusted Ah digit position
//  30/08/19  Removed constraints on values and assigned minimum values to variables to voltamp_angleoid out of scale indications when power up; Changed low SOC warning to display below 20% SOC as long as charge safety relay in open.
//  05/09/19  Returned limits of bars and text to previous values. **Never change things that work**; Improved Amp bar code; Refined when sun charge icon should appear.
//  11/09/19  Changed time and watt calculations to use voltamp_angleerage current. Changed charging symbols to use voltamp_angleerage current.
//  21/09/19  Removed the absolute and voltamp_angleerage current algorithm as I detected no values from CANBUS.
//  22/09/19  Edited CANBUS data to fit Avg and Abs current as I suspect the rxId int is unable to fit a 5th Id. AvgI used for watt and clock computations; **** modified clock calculations. ****
//  23/09/19  Adjusted clock position two pixels left when hours exceed 99.
//  24/09/19  Replaced avgI with processCanData()[1] for displaying lightning symbol whilst charging.
//  25/09/19  Added button press to swap between voltamp_angleerage and instant Watt reading. But had no success changing between the values. **** Deleted ****
//  29/09/19  Added button press to change contrast level. ** Replaced "if" in void loop button pressing for "else if & else" same with "hits". Saves 20 bytes.
//  01/10/19  Changed abbreviatons on BMS status messages to easier understand their meaning.
//  02/01/25  Removed code repeats in clock statements. Added "hrs" plural condition and associated string, and arithmetic expression for hrs above 120 displayed as days.
//  21/05/25  Serial set to 115200. Reduced clock memory usage with only one string object. Changed weak cell to not use char array but print low og high cell number directly to save memory.
//  04/06/25  Merged cabin v2 (working) with added CAN send feature to clear BMS faults.
//  14/06/25  Modified min high cell bars algo as resolution was changed from 0,001 to 0,01 in BMS. Replaced int types for byte in bars
//  18/06/25  Added pre-computed SCALING_RADIANS macro for radians and angle compensation to reduce the floating point calculations. Added hits to be 5 for text screen to be shown as it would always show if hits were not 1/3 - 5
//  19/06/25  Simplified degree to radian computations by utilising macro and replacing 2 * PI / 360 for PI / 180
//  22/06/25  Set Power page as else to show it if either no hits or 3 detected.
//  06/07/25  Removed contrast do loop from loop, and added initial setting in main. Set 500ms initialisation for button_touch to avoid false button press during start.
//  28/07/25  Trying hits = 0 instead of = STATION to see if it fixes the page jump during startup.
//  31/07/25  Increased DCL to 8 bit due overflow, had to move relay state to next rxId. Set lightening bolt as priority over sun icon. Added warning symbol if current nearing dcl.
//  03/08/25  Hits now STATION - 1 to start at correct page. Changed data types to save memory. *** Need some Amp gauge damping for the resolution change ***
//  26/08/25  Crash and lag issues. Changed SCALING_RADIANS from I 8 decimals to 3 and increased time_str from 11 to 12.
//  20/09/25  CAN sort function added to return pointer array. Much better performance and also replaced global variables
//
//  Sketch 27716 bytes
//
//  HARDWARE:
//  Arduino Uno clone
//  SH1106 128x64 OLED
//  MCP2515 TJ1A050 CANBus Transeiver
//  Pushbutton and 10kOhm pull-DOWN resistor (At the time I was unaware of the boards internal pull-UP resistor)

#include <U8g2lib.h>		                // U8g2 GUI library
#include <mcp_can.h>		                // MCP CAN library
#include <SPI.h>                        // SPI library (CAN)
#include <Wire.h>                       // I2C library (OLED)

//  SH1106 OLED I2C U8g2 LIBRARY AND pinout (3,3 - 5V)
U8G2_SH1106_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);
				                                // SDA pin A4 (I2C serial data)
				                                // SCL pin A5 (I2C serial clock)

//  MCP2515 SPI CAN pinout (5V)
#define CAN0_INT 9                      // Set INT to pin 9 (interrupt output)
MCP_CAN CAN0(10);                       // Set CS to pin 10 (chip select)
                                        // SI pin 11 (SPI data in)
				                                // SO pin 12 (SPI data out)
				                                // SCK pin 13 (SPI clock)

// MACROS
#define BUTTON_PIN 2
#define X_MAX 128                       // Display width
#define Y_MAX 64                        // Display height
#define SCALING_RADIANS 0.035           // (2 * PI / 180) - As angles are half values we need to multiply by 2 before converting degrees to radians
#define STATION 1                       // 1 for cabin position 3 for helm

//  CANBUS RX
long unsigned int rxId;                 // Stores 4 bytes 32 bits
unsigned char len = 0;                  // Stores at least 1 byte
unsigned char rxBuf[8];                 // Stores 8 bytes, 1 character  = 1 byte

//  CANBUS TX
byte mpo[8] = {0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // Multi-purpose output activation signal
/*byte mpe[8] = {0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};*/ // Multi-purpose enable activation signal

//  Global variables
byte wrench = 0;                        // Wrench icon variable
unsigned int p;                         // Watt reading
byte contrast = 255;                    // 8 bit unsigned integer range from 0-255 (low - high contrast)

byte m = 0;
byte va_angle = 0;
byte p_angle = 0;

//  Button settings
long button_held_ms = 0;                // 4 byte variable for storing duration button is held down
unsigned long button_touch_ms = 500;    // 4 byte variable for storing time button is first pushed; 500ms startup delay to avoid false button press detection
byte button_previous_state = HIGH;      // Pin state before pushing or releasing button
byte button_state = LOW;                // Variable for button pushed or not
byte hits = STATION;                    // Initialised with -1 to start on correct page as button press is registered at startup

// ------------------------- Button press handling function -----------------------

// Button checking function for PULL-DOWN
void checkButton() {
  button_state = digitalRead(BUTTON_PIN);
  
  // Button pressed (rising edge) - HIGH means pressed with pull-down
  if (button_state == HIGH && button_previous_state == LOW) {
    button_touch_ms = millis();
  }
  
  // Button released (falling edge) - LOW means released with pull-down
  if (button_state == LOW && button_previous_state == HIGH) {
    button_held_ms = millis() - button_touch_ms;
    
    // Only process if button was held for reasonable time (debounce)
    if (button_held_ms > 50 && button_held_ms < 5000) {
      handleButtonPress(button_held_ms);
    }
  }
  button_previous_state = button_state;
}

// Button press handling function - SAME as before
void handleButtonPress(unsigned long duration) {
  // Very long press (3000ms or more) - send CAN message
  if (duration >= 3000) {
    CAN0.sendMsgBuf(0x32, 0, 8, mpo);
    return;
  }
  
  // Long press (500ms or more) - change contrast
  if (duration >= 500) {
    if (contrast == 255) {
      contrast = 100;
    } else if (contrast == 100) {
      contrast = 180;
    } else {
      contrast = 255;
    }
    u8g2.setContrast(contrast);
    return;
  }
  
  // Short press - change pages
  if (hits < 5) {
    hits++;
  } else {
    hits = STATION;
  }
}

// ------------------------ setup ------------------------------

void setup() {
  // Start serial monitor communication
  //Serial.begin(115200);

  // Initialise MCP2515 for 2-way data with baudrate 250kbps and 8MHz clock speed
  CAN0.begin(MCP_ANY, CAN_250KBPS, MCP_8MHZ);

  CAN0.setMode(MCP_NORMAL);
  
  // Configure pin for INT input
  pinMode(CAN0_INT, INPUT);

  // Starts display
  u8g2.begin();

  // Set standard font
  u8g2.setFont(u8g2_font_chikita_tf);

  // Set initial contrast
  u8g2.setContrast(contrast);
}

// ------------------ CAN data sorter returning array pointer ---------------
int* processCanData() {

  //  CANBUS data Identifier List
  //  ID 0x03B BYT0+1:INST_VOLT BYT2+3:INST_AMP BYT4+5:ABS_AMP BYT6:SOC BYT7:CRC **** ABS_AMP from OrionJr errendous ****
  //  ID 0x6B2 BYT0+1:LOW_CELL BYT2+3:HIGH_CELL BYT4:HEALTH BYT5+6:CYCLES BYT7:CRC
  //  ID 0x0A9 BYT0:CCL BYT1+2:DCL BYT3+4:PACK_AH BYT5+6:AVG_AMP BYT7:CRC
  //  ID 0x0BD BYT0+1:CUSTOM_FLAGS BYT2:HI_TMP BYT3:LO_TMP BYT4:COUNTER BYT5:BMS_STATUS BYT6:RELAY_STATE BYT7:CRC
  //  ID 0x0BE BYT0:HI_CELL_VOLT BYT1:LO_CELL_VOLT BYT3:BLANK BYT4:CRC

  static int canDataArr[20];

  // Sort CANBus data buffer
   if (rxId == 0x03B) {
    canDataArr[0] = (rxBuf[0] << 8) + rxBuf[1];   //rawU
    canDataArr[1] = (rxBuf[2] << 8) + rxBuf[3];   //rawI
    canDataArr[2] = rxBuf[6];                     //soc
  }
  if(rxId == 0x6B2) {
    canDataArr[3] = ((rxBuf[0] << 8) + rxBuf[1]); //lC
    canDataArr[4] = ((rxBuf[2] << 8) + rxBuf[3]); //hC
    canDataArr[5] = (rxBuf[4]);                   //h
    canDataArr[6] = (rxBuf[5] << 8) + rxBuf[6];   //cc
  }
  if (rxId == 0x0A9) {
    canDataArr[7] = rxBuf[0];                     //ccl
    canDataArr[8] = (rxBuf[1] << 8) + rxBuf[2];   //dcl
    canDataArr[9] = (rxBuf[3] << 8) + rxBuf[4];   //ah
    canDataArr[10] = (rxBuf[5] << 8) + rxBuf[6];  //avgI
  }
  if (rxId == 0x0BD) {
    canDataArr[11] = (rxBuf[0] << 8) + rxBuf[1];  //fu
    canDataArr[12] = rxBuf[2];                    //hT
    canDataArr[13] = rxBuf[3];                    //lT
    canDataArr[14] = rxBuf[4];                    //ct
    canDataArr[15] = rxBuf[5];                    //st
    canDataArr[16] = rxBuf[6];                    //ry
    // Saves fault & status to "wrench" after reviewing text page
    canDataArr[17] = rxBuf[0] + rxBuf[1] + rxBuf[5]; //fs
  }
  if (rxId == 0x0BE) {
    canDataArr[18] = rxBuf[0];                    //hCid
    canDataArr[19] = rxBuf[1];                    //lCid
  }
  return canDataArr;
}

// --------------------- volt display * 2 bytes from rxBuf-----------------------

void voltage(byte angle) {

  // Map voltage from 44,0V - 64,0V between 0 - 50 degrees
  m = map(processCanData()[0], 440, 640, 0, 50);

  // Display dimensions
  byte xcenter = X_MAX/2;
  byte ycenter = 80;
  byte arc = 64;

  // Draw arc and scale lines
  u8g2.drawCircle(xcenter, ycenter+4, arc+8);
  u8g2.drawCircle(xcenter, ycenter+4, arc+12);
  // Draw far left line
  u8g2.drawLine(6, 31, 12, 36);
  // Draw left line
  u8g2.drawLine(12, 30, 14, 32);
  // Draw quarter left line
  u8g2.drawLine(31, 13, 34, 20);
  // Draw center line
  u8g2.drawVLine(64, 7, 7);
  // Draw quarter right line
  u8g2.drawLine(95, 12, 91, 19);
  // Draw right line
  u8g2.drawLine(99, 17, 97, 20);
  // Draw far right line
  u8g2.drawLine(122, 31, 116, 36);

  // Min shading lines
  u8g2.drawLine(12, 30, 9, 33);
  u8g2.drawLine(13, 30, 10, 33);
  u8g2.drawLine(13, 31, 10, 34);
  u8g2.drawLine(14, 31, 11, 34);
  u8g2.drawLine(14, 32, 11, 35);
  // Max shading lines
  u8g2.drawLine(94, 15, 99, 18);
  u8g2.drawLine(94, 16, 98, 18);
  u8g2.drawLine(94, 17, 98, 19);
  u8g2.drawLine(93, 17, 97, 19);
  u8g2.drawLine(93, 18, 98, 20);

  // Draw the needle and disc
  float x1 = sin(angle * SCALING_RADIANS);
  float y1 = cos(angle * SCALING_RADIANS);
  u8g2.drawLine(xcenter, ycenter, xcenter + x1 * arc, ycenter - y1 * arc);
  u8g2.drawDisc(xcenter, Y_MAX+10, 20, U8G2_DRAW_UPPER_LEFT);
  u8g2.drawDisc(xcenter, Y_MAX+10, 20, U8G2_DRAW_UPPER_RIGHT);
  // Draw scale labels
  u8g2.drawStr(0, 29, "44");
  u8g2.drawStr(24, 11, "49");
  u8g2.drawStr(59, 5, "54");
  u8g2.drawStr(92, 10, "59");
  u8g2.drawStr(118, 29, "64");

  // Draw unit
  u8g2.drawStr(48, 38, "Voltage");
}

// -------------------- amperage display * 2 bytes from rxBuf -------------------------

void amperage(byte angle) {

  // Map various amp readings to between 0 - 100 degrees. 0 = discharge 50 = charge
  if (processCanData()[1] > 1000 || processCanData()[1] < -1000) {
    m = map(processCanData()[1], -2500, 2500, 50, 0);
  }
  else if (processCanData()[1] > 500 || processCanData()[1] < -500) {
    m = map(processCanData()[1], -1000, 1000, 50, 0);
  }
  else if (processCanData()[1] > 100 || processCanData()[1] < -100) {
    m = map(processCanData()[1], -500, 500, 50, 0);
  }
  else  if (processCanData()[1] > 50 || processCanData()[1] < -50) {
    m = map(processCanData()[1], -100, 100, 50, 0);
  }
  else {
    m = map(processCanData()[1], -50, 50, 50, 0);
  }

  // Display dimensions
  byte xcenter = X_MAX / 2;
  byte ycenter = 80;
  byte arc = 64;

  // Draw arc and scale lines
  u8g2.drawCircle(xcenter, ycenter+4, arc+8);
  u8g2.drawCircle(xcenter, ycenter+4, arc+12);
  // Draw far left line
  u8g2.drawLine(6, 31, 12, 36);
  // Draw quarter left line
  u8g2.drawLine(33, 12, 36, 19);
  // Draw center line
  u8g2.drawVLine(64, 8, 4);
  // Draw quarter right line
  u8g2.drawLine(97, 13, 93, 20);
  // Draw far right line
  u8g2.drawLine(122, 31, 116, 36);
  
    
  // Draw the needle and disc
  float x1 = sin(angle * SCALING_RADIANS);
  float y1 = cos(angle * SCALING_RADIANS);
  u8g2.drawLine(xcenter, ycenter, xcenter + x1 * arc, ycenter - y1 * arc);
  u8g2.drawDisc(xcenter, Y_MAX+10, 20, U8G2_DRAW_UPPER_LEFT);
  u8g2.drawDisc(xcenter, Y_MAX+10, 20, U8G2_DRAW_UPPER_RIGHT);

  // Draw 3 different scale labels
  u8g2.setFont(u8g2_font_chikita_tf);
  // Scale from -250 till 250
  if (processCanData()[1] > 1000 || processCanData()[1] < -1000) {
    u8g2.drawStr(0, 22, "250");                   
    u8g2.drawStr(106, 22, "-250");
  }
  // Scale from -100 till 100
  else if (processCanData()[1] > 500 || processCanData()[1] < -500) {
    u8g2.drawStr(0, 24, "100");                   
    u8g2.drawStr(110, 24, "-100");
  }
  // Scale from -50 till 50
  else if (processCanData()[1] > 100 || processCanData()[1] <-100) {
    u8g2.drawStr(0, 26, "50");                   
    u8g2.drawStr(112, 26, "-50");
  }
  // Scale from -10 till 10
  else if (processCanData()[1] > 50 || processCanData()[1] < -50) {
    u8g2.drawStr(0, 28, "10");                   
    u8g2.drawStr(115, 28, "-10");
  }
  // Scale from -5 till 5
  else {
    u8g2.drawStr(0, 29, "5");                   
    u8g2.drawStr(117, 29, "-5");
  }
  // Zero never changes
  u8g2.drawStr(62, 5, "0");

  // Draw unit
  u8g2.drawStr(48, 38, "Ampere");
}

// --------------------- power display * 13 bytes from rxBuf ----------------------

void power(byte angle) {

  // Map watt readings 0-10000 to between 0 - 90 degrees
  m = map(p, 0, 10000, 0, 90);

  // Watt calculation
  p = abs(processCanData()[1]) / 10.0 * processCanData()[0] / 10.0; // have tried multiplying and then divide by 100, but p needs to be 32-bit and still cause occasional 0 reading
  
  // Display dimensions
  byte xcenter = X_MAX/2;
  byte ycenter = Y_MAX/2+10;
  byte arc = Y_MAX/2;

  // Draw border of the gauge
  u8g2.drawCircle(xcenter, ycenter, arc+6, U8G2_DRAW_UPPER_RIGHT);
  u8g2.drawCircle(xcenter, ycenter, arc+4, U8G2_DRAW_UPPER_RIGHT);
  u8g2.drawCircle(xcenter, ycenter, arc+6, U8G2_DRAW_UPPER_LEFT);
  u8g2.drawCircle(xcenter, ycenter, arc+4, U8G2_DRAW_UPPER_LEFT);

  // Draw the needle
  float x1 = sin(angle * SCALING_RADIANS);
  float y1 = cos(angle * SCALING_RADIANS);
  u8g2.drawLine(xcenter, ycenter, xcenter + x1 * arc, ycenter - y1 * arc);
  u8g2.drawDisc(xcenter, ycenter, 5, U8G2_DRAW_UPPER_LEFT);
  u8g2.drawDisc(xcenter, ycenter, 5, U8G2_DRAW_UPPER_RIGHT);
 
  // Draw scale labels
  u8g2.drawStr(20, 42, "0");                   
  u8g2.drawStr(24, 18, "25");
  u8g2.drawStr(60, 14, "50");
  u8g2.drawStr(95, 18, "75");
  u8g2.drawStr(105, 42, "100");

  // Draw gauge label
  u8g2.drawStr(45, 32, "% POWER");

  // Draw unit description
  u8g2.drawStr(88, 60, " WATT");
     
  // Draw digital value and align its position
  u8g2.setFont(u8g2_font_profont22_tn);             
  u8g2.setCursor(65,60);
  // Draw leading 0 when values below 10W
  if (p < 10){
    u8g2.print("0");
  }
  // Shift position of values above 99W
  if (p > 99 && p <= 999) {
    u8g2.setCursor(53, 60);
  }
  // Shift position of values above 999W
  else if (p > 999 && p <= 9999) {
    u8g2.setCursor(41, 60);
  }
  // Shift position of values above 9999W
  else if (p > 9999) {
    u8g2.setCursor(29,60);
  }
  u8g2.print(p);
  
  // Draw lightening bolt when charger safety relay is energised and battery charging
  if (processCanData()[10] < 0 && (processCanData()[16] & 0x04) == 0x04) {
    u8g2.setFont(u8g2_font_open_iconic_embedded_2x_t);
    u8g2.drawGlyph(4, 40, 67);
  }
   // Draw sun when charge relay is closed and battery charging
  else if ( (processCanData()[16] & 0x02) == 0x02 && processCanData()[10] < 0 ) {
    u8g2.setFont(u8g2_font_open_iconic_weather_2x_t);
    u8g2.drawGlyph(4, 39, 69);
  }
  // Draw warning symbol at and below 20% State of Charge if charge safety relay is open or if discharge current is 90% of dcl
  else if ( (processCanData()[16] & 0x04) != 0x04 && processCanData()[2] <= 40 || ( processCanData()[8] * 9 ) < processCanData()[10] ) {   // processCanData()[2] from canbus is multiplied by 2 and avgI is multiplied by 2 hence 90% of DCL
    u8g2.setFont(u8g2_font_open_iconic_embedded_2x_t);
    u8g2.drawGlyph(4, 39, 71);
  }

  // Draw wrench icon if BMS flags hasn't been seen
  if (processCanData()[17] != wrench) {
    u8g2.setFont(u8g2_font_open_iconic_embedded_2x_t);
    u8g2.drawGlyph(3, 62, 72);
  }

  // Draw battery icon
  u8g2.drawFrame( 1, 0, 22, 9);
  u8g2.drawFrame( 23, 2, 2, 5);
  u8g2.drawBox( 3, 2, processCanData()[2]/2*0.18, 5);

  // Draw state of charge
  u8g2.setCursor(4,16);
  u8g2.setFont(u8g2_font_chikita_tf);
  u8g2.print(processCanData()[2] / 2); u8g2.print('%');
  
  // Draw clock
  uint16_t h;
  byte m;
  char time_str[12];
  // Discharge
  if (processCanData()[10] > 0) {
    h = processCanData()[2] / (processCanData()[10]/10.0);
    m = (processCanData()[2] / (processCanData()[10]/10.0) - h) * 60;
  }
  // Charge
  else {
    h = (200 - processCanData()[2]) / (abs(processCanData()[10])/10.0);
    m = ((200 - processCanData()[2]) / (abs(processCanData()[10])/10.0) - h) * 60;
  }
  // Adjust x-positon
  if (h > 120) { // days
    u8g2.setCursor(94, 5);
  }
  else if (h > 99) {
    u8g2.setCursor(84, 5);
  }
  else {
    u8g2.setCursor(88, 5);
  }

  // Set days if above 120 hrs
  if (h > 120) {
    byte d = h / 24;
    // Generate string
    snprintf(time_str, sizeof(time_str), "%d days", d);
  }
  else {
    const char *plural = ( h == 1 ) ? "" : "s";
    // Generate string
    snprintf(time_str, sizeof(time_str), "%02d:%02dhr%s", h, m, plural);
  }

  // Print string
  u8g2.print(time_str);
}
 
// ------------------------ bars gauge * 7 bytes from rxBuf ------------------------

void bars() {

  // Draw pack volt bar
  byte pV = ((processCanData()[0]/10.0-43.2)*2.083); // Box length 35/16.8 (volt difference max to min)
  u8g2.setCursor(2, 5);
  u8g2.print(processCanData()[0] / 10.0, 1); // One decimal
  u8g2.drawStr(1, 56, "Pack");
  u8g2.drawStr(2, 63, "Volt");
  u8g2.drawFrame(5, 8, 11, 38);
  u8g2.drawBox(7, 44-pV, 7, pV);
  
  // Draw Min and Max cell voltage bars
  byte lCb = (processCanData()[3] - 270) * 0.34;
  byte hCb = (processCanData()[4] - 270) * 0.34;

  u8g2.setCursor(28, 5);
  u8g2.print(processCanData()[3] / 100.0, 2);  // Two decimals
  u8g2.drawStr(28, 56, "Low");
  u8g2.drawStr(29, 63, "Cell");
  u8g2.drawFrame(31, 8, 11, 38);
  if (processCanData()[3] <= 400 && processCanData()[3] >= 270) {
    u8g2.drawBox(33, 44-lCb, 7, lCb);
  }
  u8g2.setCursor(54, 5);
  u8g2.print(processCanData()[4] / 100.0, 2); // Two decimals
  u8g2.drawStr(54, 56, "High");
  u8g2.drawStr(55, 63, "Cell");
  u8g2.drawFrame(57, 8, 11, 38);
  if (processCanData()[4] <= 400 && processCanData()[4] >= 270) {
    u8g2.drawBox(59, 44-hCb, 7, hCb);
  }
  
  // Draw health bar
  byte hBar = processCanData()[5] * 0.34;
  if (processCanData()[5] >= 0 && processCanData()[5] <= 9) {
    u8g2.setCursor(88, 5);  // Shift position at and above 0%
  }
  else {                    // Shift position above 9%
    u8g2.setCursor(84, 5);
  }
  u8g2.print(processCanData()[5]);
  u8g2.drawStr(76, 56, "Health");
  u8g2.drawStr(86, 63, "%");
  u8g2.drawFrame(84, 8, 11, 38);
  if (processCanData()[5] <= 100 && processCanData()[5] >= 0) {
    u8g2.drawBox(86, 44-hBar, 7, hBar);
  }
  
  // Draw ampere bar
  byte aBar = abs(processCanData()[1])*0.0137;
  if (processCanData()[1] >= 0 && processCanData()[1] <= 90) {               // Shift position at and above 0A
    u8g2.setCursor(111, 5);
  }
  else if (processCanData()[1] > 90 && processCanData()[1] <= 199) {         // Shift position above 9A
    u8g2.setCursor(113, 5);
  }
  else if (processCanData()[1] > 190 && processCanData()[1] <= 999) {        // Shift position above 19A
    u8g2.setCursor(111, 5);
  }  
  else if (processCanData()[1] > 999 && processCanData()[1] <= 1999) {       // Shift position above 99A
    u8g2.setCursor(110, 5);
  }
  else if (processCanData()[1] > 1999) {                    // Shift position above 199A
    u8g2.setCursor(108, 5);
  }
  else if (processCanData()[1] < 0 && processCanData()[1] >= -99) {        // Shift position below 0A
    u8g2.setCursor(105, 5);
  }
  else if (processCanData()[1] < -99 && processCanData()[1] > -200) {       // Shift position below -9A
    u8g2.setCursor(107, 5);
  }
  else if (processCanData()[1] < -199 && processCanData()[1] >= -999) {     // Shift position below -19A
    u8g2.setCursor(106, 5);
  }
  else if (processCanData()[1] < -999) {                  // Shift position below -99A
    u8g2.setCursor(104, 5);
  }
  if (processCanData()[1] < 100 && processCanData()[1] > -100) {            // Prints single decimal above -10A and below 10A
    u8g2.print(processCanData()[1]/10.0, 1);
  }
  else {                                // Prints amp without decimal at and above 10A and at and below -10A
    u8g2.print(processCanData()[1]/10.0, 0);
  }
  u8g2.drawStr(108, 56, "Amp");
  u8g2.drawFrame(111, 8, 11, 38);
  // No bar displayed below 7,3A
  if (abs(processCanData()[1]) > 72 && abs(processCanData()[1]) <= 250) {
    u8g2.drawBox(113, 45-aBar, 7, aBar);
  }
  // Full bar at 250A
  else if (abs(processCanData()[1]) > 250) {
    u8g2.drawBox(113, 10, 7, 34);
  }
}
// ------------------------ text display * 18 bytes from rxBuf ---------------------

void text() {

  wrench = processCanData()[17];

  // Draw horisontal lines
  u8g2.drawHLine(0, 7, 128);
  u8g2.drawHLine(0, 37, 62);
  u8g2.drawHLine(0, 46, 128);
  u8g2.drawHLine(62, 55, 128);
  u8g2.drawHLine(97, 16, 128);
  u8g2.drawHLine(97, 26, 128);
  u8g2.drawHLine(97, 37, 128);

  // Draw vertical lines
  u8g2.drawVLine(62, 0, 64);
  u8g2.drawVLine(97, 0, 64);
  
  // Draw relay status
  u8g2.drawStr(0, 5, "Relay Status");
  u8g2.drawStr(0, 16, "Discharge");
  u8g2.setFont(u8g2_font_open_iconic_check_1x_t);
  if ((processCanData()[16] & 0x01) == 0x01) {
    u8g2.drawGlyph(52, 18, 64);
  }
  else {
    u8g2.drawGlyph(52, 18, 68);
  }
  u8g2.setFont(u8g2_font_chikita_tf);
  u8g2.drawStr(0, 25, "Charge");
  u8g2.setFont(u8g2_font_open_iconic_check_1x_t);
  if ((processCanData()[16] & 0x02) == 0x02) {
    u8g2.drawGlyph(52, 27, 64);
  }
  else {
    u8g2.drawGlyph(52, 27, 68);
  }
  u8g2.setFont(u8g2_font_chikita_tf);
  u8g2.drawStr(0, 34, "Chg Safety");
  u8g2.setFont(u8g2_font_open_iconic_check_1x_t);
  if ((processCanData()[16] & 0x04) == 0x04) {
    u8g2.drawGlyph(52, 36, 64);
  }
  else {
    u8g2.drawGlyph(52, 36, 68);
  }

  // Current limit 
  u8g2.setFont(u8g2_font_chikita_tf); 
  u8g2.drawStr(0, 44, "Current Limit"); 
  // Discharge current limit 
  u8g2.drawStr(0, 55, "DCH"); 
  if (processCanData()[8] >= 0 && processCanData()[8] < 10) { 
    u8g2.setCursor(47, 55); 
  }
  else if (processCanData()[8] >= 10 && processCanData()[8] < 20) {
    u8g2.setCursor(42, 64);
  }
  else if (processCanData()[8] >= 20 && processCanData()[8] < 100) { 
    u8g2.setCursor(40, 55); 
  } 
  else if (processCanData()[8] >= 100 && processCanData()[8] < 200) { 
    u8g2.setCursor(36, 55); 
  }
  else {
    u8g2.setCursor(34, 55);
  }
  u8g2.print(processCanData()[8]); u8g2.print(" A");
     
  // Charge current limit
  u8g2.drawStr(0, 64, "CHG"); 
  if (processCanData()[7] >= 0 && processCanData()[7] < 10) { 
    u8g2.setCursor(47, 64); 
  } 
  else if (processCanData()[7] >= 10 && processCanData()[7] < 20) { 
    u8g2.setCursor(42, 64); 
  } 
  else if (processCanData()[7] >= 20 && processCanData()[7] < 100) { 
    u8g2.setCursor(40, 64); 
  }
  else {
    u8g2.setCursor(36, 64);
  }
  u8g2.print(processCanData()[7]); u8g2.print(" A"); 
  
  // Draw fault and bms status flags
  byte x = 66;         // x position for flags
  byte y = 0;          // variable y position for flags
  u8g2.drawStr(69, 5, "Flags");

  // Flag internal communication fault
  if (((processCanData()[11] & 0x0100) == 0x0100) && y <= 28) {
    u8g2.drawStr(x, 16+y, "intCom");
    y += 7;
  }
  // Flag internal convertions fault
  if (((processCanData()[11] & 0x0200) == 0x0200) && y <= 28) {
    u8g2.drawStr(x-2, 16+y, "intConv");
    y += 7;
  }
  // Flag weak cell fault
  if (((processCanData()[11] & 0x0400) == 0x0400) && y <= 28) {
    // set cursor position for variable
    u8g2.setCursor(x + 6, 16 + y);
    if ( processCanData()[1] < 0 ) {
      u8g2.print(processCanData()[18]);
    }
    else {
      u8g2.print(processCanData()[19]);
    }
    u8g2.drawStr(x, 16 + y, "wkCl");
    y += 7;
  }
  // Flag low cell fault
  if (((processCanData()[11] & 0x0800) == 0x0800) && y <= 28) {
    u8g2.drawStr(x, 16+y, "lowCell");
    y += 7;
  }
  // Flag open wire fault
  if (((processCanData()[11] & 0x1000) == 0x1000) && y <= 28) {
    u8g2.drawStr(x-2, 16+y, "opnWire");
    y += 7;
  }
  // Flag current sense fault
  if (((processCanData()[11] & 0x2000) == 0x2000) && y <= 28) {
    u8g2.drawStr(x-1, 16+y, "crrSns");
    y += 7;
  }
  // Flag volt sense fault
  if (((processCanData()[11] & 0x4000) == 0x4000) && y <= 28) {
    u8g2.drawStr(x, 16+y, "vltSns");
    y += 7;
  }
  // Flag volt redundancy fault
  if (((processCanData()[11] & 0x8000) == 0x8000) && y <= 28) {
    u8g2.drawStr(x-2, 16+y, "vltRdcy");
    y += 7;
  }
  // Flag weak pack fault
  if (((processCanData()[11] & 0x0001) == 0x0001) && y <= 28) {
    u8g2.drawStr(x-2, 16+y, "wkPack");
    y += 7;
  }
  // Flag thermistor fault
  if (((processCanData()[11] & 0x0002) == 0x0002) && y <= 28) {
    u8g2.drawStr(x, 16+y, "xThrm");
    y += 7;
  }
  // Flag charge limit enforcement fault
  if (((processCanData()[11] & 0x0004) == 0x0004) && y <= 28) {
    u8g2.drawStr(x, 16+y, "chgRly");
    y += 7;
  }
  // Flag discharge limit enforcement fault
  if (((processCanData()[11] & 0x0008) == 0x0008) && y <= 28) {
    u8g2.drawStr(x, 16+y, "dchRly");
    y += 7;
  }
  // Flag charge safety relay fault
  if (((processCanData()[11] & 0x0010) == 0x0010) && y <= 28) {
    u8g2.drawStr(x-2, 16+y, "sftyRly");
    y += 7;
  }
  // Flag internal memory fault
  if (((processCanData()[11] & 0x0020) == 0x0020) && y <= 28) {
    u8g2.drawStr(x, 16+y, "intMem");
    y += 7;
  }
  // Flag internal thermistor fault
  if (((processCanData()[11] & 0x0040) == 0x0040) && y <= 28) {
    u8g2.drawStr(x, 16+y, "intThm");
    y += 7;
  }
  // Flag internal logic fault
  if (((processCanData()[11] & 0x0080) == 0x0080) && y <= 28) {
    u8g2.drawStr(x, 16+y, "intLog");
    y += 7;
  }

  // Flag BMS status
  if (((processCanData()[15] & 0x01) == 0x01) && y <= 28){
    u8g2.drawStr(x, 16+y, "VoltFS");
    y += 7;
  }
  if (((processCanData()[15] & 0x02) == 0x02) && y <= 28) {
    u8g2.drawStr(x-2, 16+y, "CurrFS");
    y += 7;
  }
  if (((processCanData()[15] & 0x04) == 0x04) && y <= 28) {
    u8g2.drawStr(x-1, 16+y, "RelyFS");
    y += 7;
  }
  if (((processCanData()[15] & 0x08) == 0x08) && y <= 28) {
    u8g2.drawStr(x-2, 16+y, "CellBlcg");
    y += 7;
  }

  // Draw count
  if (processCanData()[14] < 10) {
    u8g2.setCursor(111, 5);
  }
  else if (processCanData()[14] >= 10 && processCanData()[14] < 100) {
    u8g2.setCursor(109, 5);
  }
  else if (processCanData()[14] >= 100 && processCanData()[14] < 120) {
    u8g2.setCursor(107, 5);
  }
  else if (processCanData()[14] >= 120 && processCanData()[14] < 200) {
    u8g2.setCursor(106, 5);
  }
  else {
    u8g2.setCursor(105, 5);
  }
  u8g2.print(processCanData()[14]);

  // Draw total pack cycles
  u8g2.drawStr(100, 14, "Cycles");
  if (processCanData()[6] < 10) {
    u8g2.setCursor(111, 24);
  }
  else if (processCanData()[6] >= 10 && processCanData()[6] < 100) {
    u8g2.setCursor(109, 24);
  }
  else if (processCanData()[6] >= 100 && processCanData()[6] < 120) {
    u8g2.setCursor(107, 24);
  }
  else if (processCanData()[6] >= 120 && processCanData()[6] < 200) {
    u8g2.setCursor(106, 24);
  }
  else {
    u8g2.setCursor(105, 24);
  }
  u8g2.print(processCanData()[6]);

  // Draw Ah 
  u8g2.drawStr(109, 35, "Ah");
  if (processCanData()[9] < 1000) {
    u8g2.setCursor(108, 44);
  }
  if (processCanData()[9] >= 1000 && processCanData()[9] < 10000) { 
    u8g2.setCursor(105, 44); 
  } 
  else if (processCanData()[9] > 11094 && processCanData()[9] < 11195) { 
    u8g2.setCursor(106,  44); 
  } 
  else { 
    u8g2.setCursor(103,44); 
  } 
  u8g2.print(processCanData()[9]/100.0, 1); 

  // Draws pack temp
  u8g2.drawStr(66, 53, "TempH"); 
  u8g2.drawStr(100, 53, "TempL"); 
  // Highest temperature
  if (processCanData()[12] >= 0 && processCanData()[12] < 10) { 
    u8g2.setCursor(78, 64); 
  } 
  else if (processCanData()[12] >= 10) { 
    u8g2.setCursor(75, 64); 
  } 
  else if (processCanData()[12] < 0 && processCanData()[12] > -10) { 
    u8g2.setCursor(71, 64); 
  } 
  else { 
    u8g2.setCursor(68, 64); 
  } 
  u8g2.print(processCanData()[12]); 
  // Lowest temperature
  if (processCanData()[13] >= 0 && processCanData()[13] < 10) { 
    u8g2.setCursor(112, 64); 
  } 
  else if (processCanData()[13] >= 10) { 
    u8g2.setCursor(109, 64); 
  } 
  else if (processCanData()[13] < 0 && processCanData()[13] > -10) { 
    u8g2.setCursor(105, 64); 
  } 
  else { 
    u8g2.setCursor(102, 64); 
  } 
  u8g2.print(processCanData()[13]); 
} 
// -------------------------- loop -------------------------

void loop() {

  // Read MCP2515
  if(!digitalRead(CAN0_INT)) {
    CAN0.readMsgBuf(&rxId, &len, rxBuf);
  }

  // As a sine wave is fairly linear between 135deg and 225deg we use this for computing x and y positions for needle tip
  // needle tip x position relative to gauge centre, starts at minus moved to 0 and ends at positive e.g -60 -> 0 -> +60
  // needle tip y position relative to gauge centre, starts at 0 moves to max in middle before going back to 0 at full scale

  // needle position calculations for amperage and voltage (using 50 degree resolution for 100 degree movement)
  // 155 = start position, 180 = just before middle, 0 = middle, 25 = end position
  va_angle = m;
  // position correction
  if (va_angle < 25){
    va_angle += 155;
  }
  else {
    va_angle -= 25;
  }
  
  // needle position calculations watt gauge (using 90 degree resolution for 180 degree movement, as a sine wave doesn't have 180 degree linear movement)
  // 135 = start position, 180 = just before middle, 0 = middle, 45 = end position
  p_angle = m;
  // position correction
  if (p_angle < 45){
    p_angle += 135;
  }
  else {
    p_angle -= 45;
  }

  // Check the status of the button
  checkButton();

  // Display voltage page
  if (hits == 1) {
    u8g2.firstPage(); 
    do {             
      voltage(va_angle);
    }
    while(u8g2.nextPage());
  }
  
  // Display amperage page
  else if (hits == 2) {
    u8g2.firstPage(); 
    do {             
      amperage(va_angle);
    }
    while(u8g2.nextPage());
  }

  // Display bars page
  else if (hits == 4) {
    u8g2.firstPage();
    do {
      bars();
    }
    while(u8g2.nextPage());
  }

  // Display text page
  else if (hits == 5) {
    u8g2.firstPage();
    do {
      text();
    }
    while(u8g2.nextPage());
  }

  // Display power page as default or if hit == 3
  else {
    u8g2.firstPage();
    do {
      power(p_angle);
    }
    while(u8g2.nextPage());
  }
}
