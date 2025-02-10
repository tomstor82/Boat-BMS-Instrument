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
//  24/06/19  Edited CANBus data from Orion Jr as follows: removed BMS calculations for instU, DCL, CCL; set Big Endian byte order on BMS status. Changed to 1 byte hex (0x01) instead of 2 byte (0x0001) in BMS status on text page.
//  27/06/19  Added delayed amperage reading for more stability in watt and time indications and changed instI from float to int.
//  30/06/19  Added rxBuf for "ry" in gauge for displaying "sun" only when relay enabled; added bms status to new variable "fs" for "wrench" icon display; adjusted "0A" DCH & CHG limits.
//  09/07/19  Replaced counter with total pack cycles
//  27/07/19  Added counter and adjusted cycles on text page; Adjusted Ah digit position
//  30/08/19  Removed constraints on values and assigned minimum values to variables to avoid out of scale indications when power up; Changed low SOC warning to display below 20% SOC as long as charge safety relay in open.
//  05/09/19  Returned limits of bars and text to previous values. **Never change things that work**; Improved Amp bar code; Refined when sun charge icon should appear.
//  11/09/19  Changed time and watt calculations to use average current. Changed charging symbols to use average current.
//  21/09/19  Removed the absolute and average current algorithm as I detected no values from CANBUS.
//  22/09/19  Edited CANBUS data to fit Avg and Abs current as I suspect the rxId int is unable to fit a 5th Id. AvgI used for watt and clock computations; **** modified clock calculations. ****
//  23/09/19  Adjusted clock position two pixels left when hours exceed 99.
//  24/09/19  Replaced avgI with instI for displaying lightning symbol whilst charging.
//  25/09/19  Added button press to swap between average and instant Watt reading. But had no success changing between the values. **** Deleted ****
//  29/09/19  Added button press to change contrast level. ** Replaced "if" in void loop button pressing for "else if & else" same with "hits". Saves 20 bytes.
//  01/10/19  Changed abbreviatons on BMS status messages to easier understand their meaning.
//  02/01/25  Removed code repeats in clock statements. Added "hrs" plural condition and associated string, and arithmetic expression for hrs above 120 displayed as days.
//  03/02/25  Added data structures to allow instant CAN and function variables globally without redeclaration. Also added Macros which are not consuming storage, for easier code writing. Saved 3290 bytes
//  06/02/25  Forgot to add sort_can() to loop and this added +4,5 kbytes to sketch size :o also added high temp warning in form of warning sign as well as status message and weak cell identifier
//  08/02/25  Disabled u8g2 features to save memory. shaved off 1420 bytes
//  10/02/25  Crazy update with unified LUT (look-up table) for angle computation, as this is resource hungry. Also simplified the amperage mapping as standalone function as the resolution computations can look wild
//
//  Sketch 26492 bytes
//
//  HARDWARE:
//  Arduino Uno clone
//  SH1106 128x64 OLED
//  MCP2515 TJ1A050 CANBus Transeiver
//  Pushbutton and 10kOhm resistor (pull down)

#include <U8g2lib.h>
#include <mcp_can.h>  // CAN library
#include <SPI.h>      // SPI library CAN shield
#include <Wire.h>     // I2C library OLED

//  OLED library driver
U8G2_SH1106_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

//  CANBUS Shield pins
#define CAN0_INT 9                              // Set INT to pin 9
MCP_CAN CAN0(10);                               // Set CS to pin 10

//  CANBUS data Identifier List
//  ID 0x03B BYT0+1:INST_VOLT BYT2+3:INST_AMP BYT4+5:ABS_AMP BYT6:SOC **** ABS_AMP from OrionJr errendous ****
//  ID 0x6B2 BYT0+1:LOW_CELL BYT2+3:HIGH_CELL BYT4:HEALth BYT5+6:CYCLES
//  ID 0x0A9 BYT0:RELAYS_STATE BYT1:CCL BYT2:DCL BYT3+4:PACK_AH BYT5+6:AVG_AMP
//  ID 0x0BD BYT0+1:CUSTOM_FLAGS BYT2:HI_TMP BYT3:LO_TMP BYT4:FREE BYT5:BMS_STATUS
//  ID 0x0BE BYT0:HI_CELL_ID BYT1:LO_CELL_ID BYT2:COUNTER BYT3:HEAT_SINK

//  MCP_CAN DATA
struct CanMsgData {
  long unsigned int rxId;     // Stores 4 bytes 32 bits
  unsigned char len = 0;      // Stores at least 1 byte
  unsigned char rxBuf[8];     // Stores 8 bytes, 1 character  = 1 byte
};

struct CanData {

  int p = 0;                  // watt calculated by script
  int cc = 0;                 // Total pack cycles (int to avoid overrun issues with uint16_t)

  float instU = 0;            // Voltage - multiplied by 10
  float instI = 0;            // Current - multiplied by 10 - negative value indicates charge
  float avgI = 0;             // Average current for clock and sun symbol calculations
  float ah = 0;               // Amp hours
  float hC = 0;               // High Cell Voltage in 0,0001V
  float lC = 0;               // Low Cell Voltage in 0,0001V
  //float minC = 0;             // Minimum Allowed cell voltage
  //float maxC = 0;             // Maximum Allowed cell voltage

  byte soc = 0;               // State of charge - multiplied by 2
  byte hT = 0;                // Highest cell temperature
  byte lT = 0;                // Lowest cell temperature
  byte ry = 0;                // Relay status
  byte dcl = 0;               // Discharge current limit
  byte ccl = 0;               // Charge current limit
  byte h = 0;                 // Health
  byte hCid = 0;              // High Cell ID
  byte lCid = 0;              // Low Cell ID

  uint16_t fu = 0;            // BMS Faults
  uint16_t st = 0;            // BMS Status

  byte ct = 0;                // Counter

  byte hs = 0;                // Internal Heatsink
  //byte cu = 0;                // BMS custom flag NOT IN USE
};

typedef struct {
  byte xmax = 128;
  byte ymax = 64;
  byte xcenter = xmax/2;
  byte ycenter = 0;  
  byte arc = 0;
  uint32_t wrench = 0;          // Wrench icon variable
  float x = 0;
  float y = 0;
} display_data_t;

//  Variables
int mappedValue = 10;                   // Mapped values to fit needle
u8g2_uint_t needleAngle = 0;           // 8 bit unsigned int amperage and voltage needle angle
u8g2_uint_t wattNeedleAngle = 0;           // 8 bit unsigned int watt needle angle
uint8_t c = 180;              // 8 bit unsigned integer range from 0-255 (low - high contrast)

// Short-hand macros cost free
#define VOLT        canData.instU
#define AMPS        canData.instI
#define AVG_AMPS    canData.avgI
#define WATTS       canData.p
#define SOC         canData.soc
#define AH          canData.ah
#define RELAYS      canData.ry
#define CCL         canData.cc
#define DCL         canData.dcl
#define CYCLES      canData.cc
#define COUNT       canData.ct
#define FAULTS      canData.fu
#define STATUS      canData.st
#define HI_TEMP     canData.hT
#define LO_TEMP     canData.lT
#define HI_CELL_V   canData.hC
#define LO_CELL_V   canData.lC
#define HI_CELL_ID  canData.hCid
#define LO_CELL_ID  canData.lCid
#define HEALTH      canData.h
#define HEAT_SINK   canData.hs

#define X_CTR       displayData.xcenter
#define Y_CTR       displayData.ycenter
#define X_MX        displayData.xmax
#define Y_MX        displayData.ymax
#define ARC         displayData.arc
#define X           displayData.x
#define Y           displayData.y

#define CAN_RX_ID   canMsgData.rxId
#define CAN_RX_BUF  canMsgData.rxBuf

#define BUTTON_PIN  2
#define CHOICE      1             // SET 1 FOR CABIN OR 3 FOR HELM POSITION

//  Button settings
uint32_t millis_held = 0;         // 4 byte variable for storing duration button is held down
uint32_t firstTime = 0;           // 4 byte variable for storing time button is first pushed
byte previous = HIGH;             // Pin state before pushing or releasing button
byte hits = 0;                    // Variable for how many times button has bin pushed
bool buttonState = false;         // Variable for button pushed or not

static CanMsgData canMsgData;
static CanData canData;
static display_data_t displayData;

const uint8_t NUM_ANGLES = 181; // 0 to 180 degrees
uint8_t sinLUT[NUM_ANGLES];
uint8_t cosLUT[NUM_ANGLES];

// ------------------------ setup ------------------------------

void setup() {
  // Start serial monitor communication
  if (Serial) {
    Serial.begin(115200);
  }

   // Initialise MCP2515 running at 8MHz and baudrate 250kb/s
  CAN0.begin(MCP_ANY, CAN_250KBPS, MCP_8MHZ);

  CAN0.setMode(MCP_NORMAL);
  
  // Configure pin for INT input
  pinMode(CAN0_INT, INPUT);

  // Starts display
  u8g2.begin();

  // Set standard font
  u8g2.setFont(u8g2_font_chikita_tf);

  // Fill the lookup tables
  for (uint8_t i = 0; i < NUM_ANGLES; i++) {
    float angleRad = i * PI / 180.0; // Convert degrees to radians
    sinLUT[i] = (uint8_t)(sin(angleRad) * 255);
    cosLUT[i] = (uint8_t)(cos(angleRad) * 255);
  }
}

// -------------------- set contrast -----------------------------

void contrast(uint8_t c) {
  
  u8g2.setContrast(c);
}

// --------------------- sort can data into struct -------------------------

// Unsigned to Signed value function
int16_t signValue(uint16_t canValue) {
    int16_t signedValue = (canValue > 32767) ? canValue - 65536 : canValue;
    return signedValue;
}

// Sort CAN bus data
void sort_can() {

    if (CAN_RX_ID == 0x3B) {
        VOLT = ((CAN_RX_BUF[0] << 8) + CAN_RX_BUF[1]) / 10.0;
        AMPS = (signValue((CAN_RX_BUF[2] << 8) + CAN_RX_BUF[3])) / 10.0; // orion2jr issue: unsigned value despite ticket as signed
        //CAPACITY = ((CAN_RX_BUF[4] << 8) + CAN_RX_BUF[5]) / 10.0; // not available on orionJr
        SOC = CAN_RX_BUF[6] / 2;
    }
    if (CAN_RX_ID == 0x6B2) {
        LO_CELL_V = ((CAN_RX_BUF[0] << 8) + CAN_RX_BUF[1]) / 10000.00;
        HI_CELL_V = ((CAN_RX_BUF[2] << 8) + CAN_RX_BUF[3]) / 10000.00;
        HEALTH = CAN_RX_BUF[4];
        CYCLES = (CAN_RX_BUF[5] << 8) + CAN_RX_BUF[6];
    }
    if (CAN_RX_ID == 0x0A9) {
        RELAYS = CAN_RX_BUF[0];
        CCL = CAN_RX_BUF[1];
        DCL = CAN_RX_BUF[2];
        AH = ((CAN_RX_BUF[3] << 8) + CAN_RX_BUF[4]) / 10.0; //******************************************* MIGHT NEED TO BE DIVIDED BY 100.0 FOR ORION JR
        AVG_AMPS = (signValue((CAN_RX_BUF[5] << 8) + CAN_RX_BUF[6])) / 10.0; // orion2jr issue: unsigned value despite ticket as signed
    }
    if (CAN_RX_ID == 0x0BD) {
        FAULTS = (CAN_RX_BUF[0] << 8) + CAN_RX_BUF[1];
        HI_TEMP = CAN_RX_BUF[2];
        LO_TEMP = CAN_RX_BUF[3];
        //canData.cu = CAN_RX_BUF[4];
        STATUS = (CAN_RX_BUF[5] << 8) + CAN_RX_BUF[6];
    }
    if (CAN_RX_ID == 0x0BE) {
        HI_CELL_ID = CAN_RX_BUF[0];
        LO_CELL_ID = CAN_RX_BUF[1];
        HEAT_SINK = CAN_RX_BUF[2];
        COUNT = CAN_RX_BUF[4];
    }
    WATTS = abs(AVG_AMPS) * VOLT; // absolute amps used for boat
}

// -------------------- needle angle function -----------------------------------------

uint8_t getSin(uint8_t angle) {
  if (angle <= 90) return sinLUT[angle];
  if (angle <= 180) return sinLUT[180 - angle];
  if (angle <= 270) return 255 - sinLUT[angle - 180];
  return 255 - sinLUT[360 - angle];
}

uint8_t getCos(uint8_t angle) {
  return getSin(angle + 90);
}

void drawNeedle(uint8_t angle, uint8_t centerX, uint8_t centerY, uint8_t radius) {
  uint8_t idx = angle % 360;
  int16_t x = centerX + (radius * getSin(idx)) / 255;
  int16_t y = centerY - (radius * getCos(idx)) / 255;
  u8g2.drawLine(centerX, centerY, x, y);
}

// -------------------- mapping function ----------------------------------------------

int mapAmperage(float amps) {

  if (amps >= 100 || amps <= -100) return map(amps, -250, 250, 50, 0);
  if (amps >= 50 || amps <= -50) return map(amps, -100, 100, 50, 0);
  if (amps >= 10 || amps <= -10) return map(amps, -50, 50, 50, 0);
  if (amps >= 5 || amps <= -5) return map(amps, -10, 10, 50, 0);

  return map(amps, -5, 5, 50, 0);
}

// -------------------- amperage display * 2 bytes from rxBuf -------------------------

void amperage(uint8_t angle, display_data_t *data) {

  Y_CTR = 80;
  ARC = 64;

  // Map various amp readings to between 0-50. 0 = discharge 50 = charge
  mappedValue = mapAmperage(AMPS);

  // Draw arc and scale lines
  u8g2.drawCircle(X_CTR, Y_CTR + 4, ARC + 8);
  u8g2.drawCircle(X_CTR, Y_CTR + 4, ARC + 12);
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
  drawNeedle(angle, X_CTR, Y_CTR, ARC);
  //u8g2.drawLine(X_CTR, Y_CTR, X_CTR + ARC * NEEDLE_X, Y_CTR - ARC * NEEDLE_Y);
  u8g2.drawDisc(X_CTR, Y_MX + 10, 20, U8G2_DRAW_UPPER_LEFT);
  u8g2.drawDisc(X_CTR, Y_MX + 10, 20, U8G2_DRAW_UPPER_RIGHT);

  // Draw 3 different scale labels
  u8g2.setFont(u8g2_font_chikita_tf);
  // Scale from -250 till 250
  if (AMPS > 100 || AMPS < -100) {
    u8g2.drawStr(0, 22, "250");                   
    u8g2.drawStr(106, 22, "-250");
  }
  // Scale from -100 till 100
  else if (AMPS > 50 || AMPS < -50) {
    u8g2.drawStr(0, 24, "100");                   
    u8g2.drawStr(110, 24, "-100");
  }
  // Scale from -50 till 50
  else if (AMPS > 10 || AMPS <-10) {
    u8g2.drawStr(0, 26, "50");                   
    u8g2.drawStr(112, 26, "-50");
  }
  // Scale from -10 till 10
  else if (AMPS > 5 || AMPS < -5) {
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

// --------------------- volt display * 2 bytes from rxBuf-----------------------

void voltage(uint8_t angle, display_data_t *data) {

  // Map voltage from 44,0V - 64,0V between 0 - 50
  mappedValue = map(VOLT, 44.0, 64.0, 0, 50);

  // Draw arc and scale lines
  u8g2.drawCircle(X_CTR, Y_CTR + 4, ARC + 8);
  u8g2.drawCircle(X_CTR, Y_CTR + 4, ARC + 12);
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
  drawNeedle(angle, X_CTR, Y_CTR, ARC); //u8g2.drawLine(X_CTR, Y_CTR, X_CTR + ARC * NEEDLE_X, Y_CTR - ARC * NEEDLE_Y);
  u8g2.drawDisc(X_CTR, Y_MX + 10, 20, U8G2_DRAW_UPPER_LEFT);
  u8g2.drawDisc(X_CTR, Y_MX + 10, 20, U8G2_DRAW_UPPER_RIGHT);
  // Draw scale labels
  u8g2.drawStr(0, 29, "44"); 
  u8g2.drawStr(24, 11, "49");                  
  u8g2.drawStr(59, 5, "54");
  u8g2.drawStr(92, 10, "59");
  u8g2.drawStr(118, 29, "64");
  
  // Draw unit
  u8g2.drawStr(48, 38, "Voltage");

}

// --------------------- gauge display * 11 bytes from rxBuf ----------------------

void gauge(uint8_t angle, display_data_t *data) {

  // Map watt readings 0-10000 to between 0-90
  mappedValue = map(WATTS, 0, 10000, 0, 90);

  // Display dimensions
  Y_CTR = Y_MX / 2 + 10;
  ARC = Y_MX / 2;

  // Draw border of the gauge
  u8g2.drawCircle(X_CTR, Y_CTR, ARC + 6, U8G2_DRAW_UPPER_RIGHT);
  u8g2.drawCircle(X_CTR, Y_CTR, ARC + 4, U8G2_DRAW_UPPER_RIGHT);
  u8g2.drawCircle(X_CTR, Y_CTR, ARC + 6, U8G2_DRAW_UPPER_LEFT);
  u8g2.drawCircle(X_CTR, Y_CTR, ARC + 4, U8G2_DRAW_UPPER_LEFT);

  // Draw the needle
  drawNeedle(angle, X_CTR, Y_CTR, ARC); //u8g2.drawLine(X_CTR, Y_CTR, X_CTR + ARC * NEEDLE_X, Y_CTR - ARC * NEEDLE_Y);
  u8g2.drawDisc(X_CTR, Y_CTR, 5, U8G2_DRAW_UPPER_LEFT);
  u8g2.drawDisc(X_CTR, Y_CTR, 5, U8G2_DRAW_UPPER_RIGHT);
 
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
  if (WATTS < 10){
    u8g2.print("0");
  }
  // Shift position of values above 99W
  if (WATTS > 99 && WATTS <= 999) {
    u8g2.setCursor(53, 60);
  }
  // Shift position of values above 999W
  else if (WATTS > 999 && WATTS <= 9999) {
    u8g2.setCursor(41, 60);
  }
  // Shift position of values above 9999W
  else if (WATTS > 9999) {
    u8g2.setCursor(29,60);
  }
  u8g2.print(WATTS);
  
  // Draw lightening bolt when charge current above 20A and charge safety relay is closed
  if (AMPS < -20 && (RELAYS & 0b00000100) == 0b00000100) {
    u8g2.setFont(u8g2_font_open_iconic_embedded_2x_t);
    u8g2.drawGlyph(4, 40, 67);
  }
  // Draw sun when charge current is above 0A and charge relay is closed and charge safety relay is open or above 30A charge and charge safety relay closed
  if (AVG_AMPS < 0 && (RELAYS & 0b00000010) == 0b00000010 && (RELAYS & 0b00000100) != 0b00000100 || AVG_AMPS < -30 && (RELAYS & 0b00000010) == 0b00000010 && (RELAYS & 0b00000100) == 0b00000100) {
    u8g2.setFont(u8g2_font_open_iconic_weather_2x_t);
    u8g2.drawGlyph(4, 39, 69);
  }
  // Draw warning symbol at and below 20% SOC if discharge or battery high temperature
  if (SOC <= 20 && AVG_AMPS > 0 || HI_TEMP > 35) {
    u8g2.setFont(u8g2_font_open_iconic_embedded_2x_t);
    u8g2.drawGlyph(4, 39, 71);
  }
  // Draw wrench icon if BMS flags have not been seen
  if ( (FAULTS + STATUS) != data->wrench) {
    u8g2.setFont(u8g2_font_open_iconic_embedded_2x_t);
    u8g2.drawGlyph(3, 62, 72);
  }

  // Draw battery icon
  u8g2.drawFrame( 1, 0, 22, 9);
  u8g2.drawFrame( 23, 2, 2, 5);
  u8g2.drawBox( 3, 2, SOC * 0.18, 5);

  // Draw state of charge
  u8g2.setCursor(4,16);
  u8g2.setFont(u8g2_font_chikita_tf);
  u8g2.print(SOC);
  u8g2.print('%');
  
  // Draw clock
  uint16_t h;
  uint8_t m;
  char t[11];
  char c[4] = {"hrs"};

  h = AH / abs(AVG_AMPS);
  m = AH / (abs(AVG_AMPS) - h) * 60;

  // Adjust x - positon
  if (h > 99 && h <= 120 || h >= 240) { // AND has higher precedence than OR so essentially this is "(h>99 && h<=120) || h>=240"
    u8g2.setCursor(84, 5);
  }
  else {
    u8g2.setCursor(88, 5);
  }
  // Set days if above 120 hrs
  if (h > 120) {
    uint8_t d = 24 / h;
    sprintf(t, "+%d days", d);
  }

  else {
    // Plural if not 1
    if (h == 1) {
      strcpy(c, "hr");
    }
    sprintf(t, "%02d:%02d%s", h, m, c);
  }
  // Print
  u8g2.print(t);
}
 
// ------------------------ bars gauge * 7 bytes from rxBuf ------------------------

void bars(display_data_t *data) {
  
  // Draw pack volt bar
  int pV = ( (VOLT - 43.2) * 2.083 ); // Box length 35/16.8 (volt difference max to min)
  u8g2.setCursor(2, 5);
  u8g2.print(VOLT, 1); // One decimal
  u8g2.drawStr(1, 56, "Pack");
  u8g2.drawStr(2, 63, "Volt");
  u8g2.drawFrame(5, 8, 11, 38);
  u8g2.drawBox(7, 44 - pV, 7, pV);
  
  // Draw Min and Max cell voltage bars
  int hCb = ( HI_CELL_V - 2.7) * 26.9;
  int lCb = ( LO_CELL_V - 2.7) * 26.9;
  u8g2.setCursor(28, 5);
  u8g2.print(HI_CELL_V, 2); // Two decimals
  u8g2.drawStr(28, 56, "High");
  u8g2.drawStr(29, 63, "Cell");
  u8g2.drawFrame(31, 8, 11, 38);
  if (hCb <= 35 && hCb > 0) {
    u8g2.drawBox(33, 44 - hCb, 7, hCb);
  }
  u8g2.setCursor(54, 5);
  u8g2.print(LO_CELL_V, 2);  // Two decimals
  u8g2.drawStr(54, 56, "Low");
  u8g2.drawStr(55, 63, "Cell");
  u8g2.drawFrame(57, 8, 11, 38);
  if (lCb <= 35 && lCb > 0) {
    u8g2.drawBox(59, 44 - lCb, 7, lCb);
  }
  
  // Draw health bar
  
  int hBar = HEALTH * 0.34;
  if (HEALTH >= 0 && HEALTH <= 9) {
    u8g2.setCursor(88, 5);  // Shift position at and above 0%
  }
  else {                    // Shift position above 9%
    u8g2.setCursor(84, 5);
  }
  u8g2.print(HEALTH);
  u8g2.drawStr(76, 56, "Health");
  u8g2.drawStr(86, 63, "%");
  u8g2.drawFrame(84, 8, 11, 38);
  if (HEALTH <= 100 && HEALTH >= 0) {
    u8g2.drawBox(86, 44 - hBar, 7, hBar);
  }
  
  // Draw ampere bar

  float aBar = abs(AMPS) * 0.137;

  // DISCHARGE CURSOR POSITION ADJUSTMENTS
  if (AMPS >= 200) {                     // Shift position above 199A
    u8g2.setCursor(108, 5);
  }
  else if (AMPS >= 100) {                // Shift position above 99A
    u8g2.setCursor(110, 5);
  }
  else if (AMPS >= 20) {                 // Shift position above 19A
    u8g2.setCursor(111, 5);
  }
  else if (AMPS >= 10) {                 // Shift position above 9A
    u8g2.setCursor(113, 5);
  }
  else if (AMPS >= 0) {                  // Shift position at and above 0A
    u8g2.setCursor(111, 5);
  }

  // CHARGE CURSOR POSITION ADJUSTMENTS
  else if (AMPS <= -100) {               // Shift position below -99A
    u8g2.setCursor(104, 5);
  }
  else if (AMPS <= -20) {                // Shift position below -19A
    u8g2.setCursor(106, 5);
  }
  else if (AMPS <= -10) {                // Shift position below -9A
    u8g2.setCursor(107, 5);
  }
  else if (AMPS < 0) {                   // Shift position below 0A
    u8g2.setCursor(105, 5);
  }

  if (AMPS < 10 && AMPS > -10) {
    u8g2.print(AMPS, 1);                 // Prints single decimal above -10A and below 10A
  }
  else {
    u8g2.print(AMPS, 0);                  // Prints amp without decimal at and above 10A and at and below -10A
  }
  u8g2.drawStr(108, 56, "Amp");
  u8g2.drawFrame(111, 8, 11, 38);

  // No bar displayed below 7,3A
  if (abs(AMPS) > 7.2 && abs(AMPS) <= 25) {
    u8g2.drawBox(113, 45 - aBar, 7, aBar);
  }
  // Full bar at 250A
  else if (abs(AMPS) > 250) {
    u8g2.drawBox(113, 10, 7, 34);
  }
}
// ------------------------ text display * 13 bytes from rxBuf ---------------------

void text(display_data_t *data) {

  // store current bms status to wrench var for comparison if fs changes
  data->wrench =  FAULTS + STATUS;

  // Draw horisontal lines
  u8g2.drawHLine(0, 7, 128);
  u8g2.drawHLine(0, 37, 62);
  u8g2.drawHLine(0, 46, 128);
  u8g2.drawHLine(62, 55, 128);
  u8g2.drawHLine(97, 16, 128);
  u8g2.drawHLine(97, 26, 128);
  u8g2.drawHLine(97, 37, 128);

  // Draw vertical lines
  u8g2.drawVLine(62, 0, 64); u8g2.drawVLine(97, 0, 64);
    
  // Draw relay status
  u8g2.drawStr(0, 5, "Relay Status");
  u8g2.drawStr(0, 16, "Discharge");
  u8g2.setFont(u8g2_font_open_iconic_check_1x_t);
  if ((RELAYS & 0b00000001) == 0b00000001) {
    u8g2.drawGlyph(52, 18, 64);
  }
  else {
    u8g2.drawGlyph(52, 18, 68);
  }
  u8g2.setFont(u8g2_font_chikita_tf);
  u8g2.drawStr(0, 25, "Charge");
  u8g2.setFont(u8g2_font_open_iconic_check_1x_t);
  if ((RELAYS & 0b00000010) == 0b00000010) {
    u8g2.drawGlyph(52, 27, 64);
  }
  else {
    u8g2.drawGlyph(52, 27, 68);
  }
  u8g2.setFont(u8g2_font_chikita_tf);
  u8g2.drawStr(0, 34, "Chg Safety");
  u8g2.setFont(u8g2_font_open_iconic_check_1x_t);
  if ((RELAYS & 0b00000100) == 0b00000100) {
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

  if (DCL >= 200 ) {
    u8g2.setCursor(34, 55);
  }
  else if (DCL >= 100) { 
    u8g2.setCursor(36, 55); 
  }
  else if (DCL >= 20) { 
    u8g2.setCursor(40, 55); 
  }
  else if (DCL >= 10) {
    u8g2.setCursor(42, 64);
  }
  else { 
    u8g2.setCursor(47, 55); 
  }
  // print label
  u8g2.print(DCL);
  u8g2.print(" A");
     
  // Charge current limit
  u8g2.drawStr(0, 64, "CHG"); 

  if (CCL >= 200) {
    u8g2.setCursor(36, 64);
  }
  else if (CCL >= 20) { 
    u8g2.setCursor(40, 64); 
  }
  else if (CCL >= 10) { 
    u8g2.setCursor(42, 64); 
  }
  else { 
    u8g2.setCursor(47, 64); 
  } 
  // print label
  u8g2.print(CCL);
  u8g2.print(" A"); 
  
  // Draw fault and bms status flags
  uint8_t x = 66;         // x position for flags
  uint8_t y = 0;          // variable y position for flags
  u8g2.drawStr(69, 5, "Flags");

  // Flag internal communication fault
  if (((FAULTS & 0x0100) == 0x0100) && y <= 28) {
    u8g2.drawStr(x, 16 + y, "intCom");
    y += 7;
  }
  // Flag internal convertions fault
  if (((FAULTS & 0x0200) == 0x0200) && y <= 28) {
    u8g2.drawStr(x - 2, 16 + y, "intConv");
    y += 7;
  }
  // Flag weak cell fault
  if (((FAULTS & 0x0400) == 0x0400) && y <= 28) {
    // Create buffer to add weak cell id to string
    char buf[7];
    snprintf(buf, sizeof(buf), "#%dweak", LO_CELL_ID);
    u8g2.drawStr(x, 16 + y, buf);
    y += 7;
  }
  // Flag low cell fault
  if (((FAULTS & 0x0800) == 0x0800) && y <= 28) {
    u8g2.drawStr(x, 16 + y, "lowCell");
    y += 7;
  }
  // Flag open wire fault
  if (((FAULTS & 0x1000) == 0x1000) && y <= 28) {
    u8g2.drawStr(x - 2, 16 + y, "opnWire");
    y += 7;
  }
  // Flag current sense fault
  if (((FAULTS & 0x2000) == 0x2000) && y <= 28) {
    u8g2.drawStr(x - 1, 16 + y, "crrSns");
    y += 7;
  }
  // Flag volt sense fault
  if (((FAULTS & 0x4000) == 0x4000) && y <= 28) {
    u8g2.drawStr(x, 16 + y, "vltSns");
    y += 7;
  }
  // Flag volt redundancy fault
  if (((FAULTS & 0x8000) == 0x8000) && y <= 28) {
    u8g2.drawStr(x - 2, 16 + y, "vltRdcy");
    y += 7;
  }
  // Flag weak pack fault
  if (((FAULTS & 0x0001) == 0x0001) && y <= 28) {
    u8g2.drawStr(x - 2, 16 + y, "wkPack");
    y += 7;
  }
  // Flag thermistor fault
  if (((FAULTS & 0x0002) == 0x0002) && y <= 28) {
    u8g2.drawStr(x, 16 + y, "xThrm");
    y += 7;
  }
  // Flag charge limit enforcement fault
  if (((FAULTS & 0x0004) == 0x0004) && y <= 28) {
    u8g2.drawStr(x - 2, 16 + y, "chgRly");
    y += 7;
  }
  // Flag discharge limit enforcement fault
  if (((FAULTS & 0x0008) == 0x0008) && y <= 28) {
    u8g2.drawStr(x, 16 + y, "dchRly");
    y += 7;
  }
  // Flag charge safety relay fault
  if (((FAULTS & 0x0010) == 0x0010) && y <= 28) {
    u8g2.drawStr(x - 2, 16 + y, "sftyRly");
    y += 7;
  }
  // Flag internal memory fault
  if (((FAULTS & 0x0020) == 0x0020) && y <= 28) {
    u8g2.drawStr(x, 16 + y, "intMem");
    y += 7;
  }
  // Flag internal thermistor fault
  if (((FAULTS & 0x0040) == 0x0040) && y <= 28) {
    u8g2.drawStr(x, 16 + y, "intThm");
    y += 7;
  }
  // Flag internal logic fault
  if (((FAULTS & 0x0080) == 0x0080) && y <= 28) {
    u8g2.drawStr(x, 16 + y, "inlTog");
    y += 7;
  }

  // Flag BMS status
  if (((STATUS & 0x01) == 0x01) && y <= 28){
    u8g2.drawStr(x, 16 + y, "VoltFS");
    y += 7;
  }
  if (((STATUS & 0x02) == 0x02) && y <= 28) {
    u8g2.drawStr(x - 2, 16 + y, "CurrFS");
    y += 7;
  }
  if (((STATUS & 0x04) == 0x04) && y <= 28) {
    u8g2.drawStr(x - 1, 16 + y, "RelyFS");
    y += 7;
  }
  if (((STATUS & 0x08) == 0x08) && y <= 28) {
    u8g2.drawStr(x - 2, 16 + y, "CellBlcg");
    y += 7;
  }
  if ( HI_TEMP > 35 ) {
    u8g2.drawStr(x - 2, 16 + y, "HighTemp");
    y += 7;
  }

  // Draw count
  if (COUNT >= 200) {
    u8g2.setCursor(105, 5);
  }
  else if (COUNT >= 120) {
    u8g2.setCursor(106, 5);
  }
  else if (COUNT >= 100) {
    u8g2.setCursor(107, 5);
  }
  else if (COUNT >= 10) {
    u8g2.setCursor(109, 5);
  }
  else {
    u8g2.setCursor(111, 5);
  }
  // print label
  u8g2.print(COUNT);

  // Draw total pack cycles
  u8g2.drawStr(100, 14, "Cycles");
  // adjust cursor position
  if (CYCLES > 200) {
    u8g2.setCursor(105, 24);
  }
  else if (CYCLES >= 120) {
    u8g2.setCursor(106, 24);
  }
  else if (CYCLES >= 100) {
    u8g2.setCursor(107, 24);
  }
  else if (CYCLES >= 10) {
    u8g2.setCursor(109, 24);
  }
  else {
    u8g2.setCursor(111, 24);
  }
  // print label
  u8g2.print(CYCLES);

  // Draw Ah 
  u8g2.drawStr(109, 35, "Ah");
  // adjust cursor
  if (AH >= 120) { 
    u8g2.setCursor(103,44); 
  }
  else if (AH >= 110) { 
    u8g2.setCursor(106,  44); 
  }
  else if (AH >= 10) { 
    u8g2.setCursor(105, 44); 
  }
  else {
    u8g2.setCursor(108, 44);
  }

  // print label with one decimal
  u8g2.print(AH, 1); 

  // Draws pack temp
  u8g2.drawStr(66, 53, "TempH"); 
  u8g2.drawStr(100, 53, "TempL"); 
  // Highest temperature
  if (HI_TEMP >= 0 && HI_TEMP < 10) { 
    u8g2.setCursor(78, 64); 
  } 
  else if (HI_TEMP >= 10) { 
    u8g2.setCursor(75, 64); 
  } 
  else if (HI_TEMP < 0 && HI_TEMP > -10) { 
    u8g2.setCursor(71, 64); 
  } 
  else { 
    u8g2.setCursor(68, 64); 
  } 
  u8g2.print(HI_TEMP); 
  // Lowest temperature
  if (LO_TEMP >= 0 && LO_TEMP < 10) { 
    u8g2.setCursor(112, 64); 
  } 
  else if (LO_TEMP >= 10) { 
    u8g2.setCursor(109, 64); 
  } 
  else if (LO_TEMP < 0 && LO_TEMP > -10) { 
    u8g2.setCursor(105, 64); 
  } 
  else { 
    u8g2.setCursor(102, 64); 
  } 
  u8g2.print(LO_TEMP); 
} 
// -------------------------- loop -------------------------

void loop() {

  do {
    contrast(c);
  }
  while(u8g2.nextPage());
  
  // Read MCP2515
  if(!digitalRead(CAN0_INT)) {
    CAN0.readMsgBuf(&CAN_RX_ID, &canMsgData.len, CAN_RX_BUF);
    sort_can();
  }

  // needle position calculations for amperage and voltage
  // 155 = zero position, 180 = just before middle, 0 = middle, 25 = max
  needleAngle = mappedValue;
  // position correction
  if (needleAngle < 25){
    needleAngle += 155;
  }
  else {
    needleAngle -= 25;
  }
  
  // needle position calculations watt gauge
  // 135 = zero position, 180 = just before middle, 0 = middle, 45 = max
  wattNeedleAngle = mappedValue;
  // position correction
  if (wattNeedleAngle < 45){
    wattNeedleAngle += 135;
  }
  else {
    wattNeedleAngle -= 45;
  }

  // Check the status of the button
  buttonState = digitalRead(BUTTON_PIN);

  // How long is the button held down
  if (buttonState == HIGH && previous == LOW) {
    firstTime = millis();
  }
  if (buttonState == LOW && previous == HIGH) {
    millis_held = millis() - firstTime;
  }

  // Require more Than 200ms push to qualify as a button "hit"
  if (millis_held > 200) {
    if (buttonState == LOW && previous == HIGH) {

      // Long push for 0,5 sec changes contrast
      if (millis_held >= 500) {
        if (c == 255) {
          c = 100;
        }
        else if (c == 100) {
          c = 180;
        }
        else {
          c = 255;
        }
      }
      
      // Short button press changes between pages
      else {
        if (hits < 5) {
          hits += 1;  // adds 1 to hits
        }
        else {
          hits = CHOICE;
        }
      }
    }
    // Save button state
    previous = buttonState;
  }

  // Display voltage page
  if (hits == 1) {
    u8g2.firstPage(); 
    do {             
      voltage(needleAngle, &displayData);
    }
    while(u8g2.nextPage());
  }
  
  // Display amperage page
  else if (hits == 2) {
    u8g2.firstPage(); 
    do {             
      amperage(needleAngle, &displayData);
    }
    while(u8g2.nextPage());
  }
  
  // Display gauge page
  else if (hits == 3) {
    u8g2.firstPage(); 
    do {             
      gauge(wattNeedleAngle, &displayData);
    }
    while(u8g2.nextPage());
  }

  // Display bars page
  else if (hits == 4) {
    u8g2.firstPage();
    do {
      bars(&displayData);
    }
    while(u8g2.nextPage());
  }

  // Display text page
  else {
    u8g2.firstPage();
    do {
      text(&displayData);
    }
    while(u8g2.nextPage());
  }
  
}
