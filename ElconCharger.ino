#include <U8g2lib.h>
#include <Wire.h>
#include <ACAN2515.h>
#include "icons.h"

#ifndef ARDUINO_ARCH_ESP32
  #error "Select an ESP32 board" 
#endif

#define charger_millivolt_setpoint 196000
#define charger_milliamp_setpoint 8000

// Display Driver
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R2, /* reset=*/ 16, /* clock=*/ 15, /* data=*/ 4); // 1024 byte frame buffe


// CAN driver
static const byte MCP2515_SCK  = 18 ; // 26 SCK input of MCP2517 
static const byte MCP2515_MOSI = 23 ; // 19 SDI input of MCP2517  
static const byte MCP2515_MISO = 19 ; // 18 SDO output of MCP2517 
static const byte MCP2515_CS  = 22 ; // CS input of MCP2515 (adapt to your design) 
static const byte MCP2515_INT =  5 ; // INT output of MCP2515 (adapt to your design)
ACAN2515 can (MCP2515_CS, SPI, MCP2515_INT) ;

// Receive functions
static void receive0 (const CANMessage & inMessage) {
  Serial.println ("Receive 0") ;
}

static void receive1 (const CANMessage & inMessage) {
  Serial.println ("Receive 1") ;
}

static void receive2 (const CANMessage & inMessage) {
  Serial.println ("Receive 2") ;
}

//——————————————————————————————————————————————————————————————————————————————
//  MCP2515 Quartz: adapt to your design
//——————————————————————————————————————————————————————————————————————————————

static const uint32_t QUARTZ_FREQUENCY = 8UL * 1000UL * 1000UL ; // 8 MHz

//——————————————————————————————————————————————————————————————————————————————
//   SETUP
//——————————————————————————————————————————————————————————————————————————————

void setup () {
//--- Switch on builtin led
  pinMode (LED_BUILTIN, OUTPUT) ;
  digitalWrite (LED_BUILTIN, HIGH) ;


  
//--- Start serial
  Serial.begin (115200) ;
//--- Wait for serial (blink led at 10 Hz during waiting)
  while (!Serial) {
    delay (50) ;
    digitalWrite (LED_BUILTIN, !digitalRead (LED_BUILTIN)) ;
  }

  //--- Switch on OLED module
  u8g2.begin();
  
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_open_iconic_embedded_4x_t);
  u8g2.drawGlyph(48, 32, 78); // power icon
  u8g2.setFont(u8g2_font_helvB10_tr); // top line
  u8g2.drawStr(0,42, "Starting Up!"); 
  u8g2.sendBuffer();
  
//--- Begin SPI
  SPI.begin (MCP2515_SCK, MCP2515_MISO, MCP2515_MOSI) ;
//--- Configure ACAN2515
  Serial.println ("Configure ACAN2515") ;
  ACAN2515Settings settings (QUARTZ_FREQUENCY, 250L * 1000UL) ; // CAN bit rate 250 kb/s
  //settings.mRequestedMode = ACAN2515Settings::LoopBackMode ; // Select loopback mode
  const ACAN2515Mask rxm0 = extended2515Mask (0x1FFFFFFF) ; // For filter #0 and #1
  const ACAN2515Mask rxm1 = standard2515Mask (0xFF, 0xFF, 0) ; // For filter #2 to #5
  const ACAN2515AcceptanceFilter filters [] = {
    {extended2515Filter (0x18FF50E5), receive0},
    {extended2515Filter (0x1806E680), receive1}
    //{standard2515Filter (0x560, 0x55, 0), receive2}
  } ;
  uint32_t errorCode = 0;//can.begin (settings, [] { can.isr () ; } );//, rxm0, filters, 2) ;
  do
  {
    Serial.println ("waiting for can...");
    delay(500);
    errorCode = can.begin (settings, [] { can.isr () ; } );//, rxm0, filters, 2) ;
  } while(errorCode != 0);
  
  if (errorCode == 0) {
    Serial.print ("Bit Rate prescaler: ") ;
    Serial.println (settings.mBitRatePrescaler) ;
    Serial.print ("Propagation Segment: ") ;
    Serial.println (settings.mPropagationSegment) ;
    Serial.print ("Phase segment 1: ") ;
    Serial.println (settings.mPhaseSegment1) ;
    Serial.print ("Phase segment 2: ") ;
    Serial.println (settings.mPhaseSegment2) ;
    Serial.print ("SJW: ") ;
    Serial.println (settings.mSJW) ;
    Serial.print ("Triple Sampling: ") ;
    Serial.println (settings.mTripleSampling ? "yes" : "no") ;
    Serial.print ("Actual bit rate: ") ;
    Serial.print (settings.actualBitRate ()) ;
    Serial.println (" bit/s") ;
    Serial.print ("Exact bit rate ? ") ;
    Serial.println (settings.exactBitRate () ? "yes" : "no") ;
    Serial.print ("Sample point: ") ;
    Serial.print (settings.samplePointFromBitStart ()) ;
    Serial.println ("%") ;
  }else{
    Serial.print ("Configuration error 0x") ;
    Serial.println (errorCode, HEX) ;
  }
}

//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

static uint32_t gBlinkLedDate = 0 ;
static uint32_t gReceivedFrameCount = 0 ;
static uint32_t gSentFrameCount = 0 ;
static uint32_t displayFrameDate = 0;
static uint32_t lastCANMessageDate = 0;
static int32_t lastCANMilliVolt = 0;
static int32_t lastCANMilliAmp = 0;
static int8_t lastCANStatus= -1;

void update_display() 
{
  char sbuf[64];
  uint32_t millisNow = millis();
  int8_t canStatus = lastCANStatus;
  //canStatus = 0x00;
  //lastCANMilliVolt = 238450;
  //lastCANMilliAmp = 12345;
  if((millisNow - displayFrameDate) > 500)
  {
    u8g2.clearBuffer();
    
    if(canStatus == 0) // charging
    {
      u8g2.setFont(u8g2_font_open_iconic_embedded_4x_t);
      u8g2.drawGlyph(48, 32, 64); // battery empty  
      
      u8g2.setFont(u8g2_font_helvB10_tr); // top line
      u8g2.drawStr(0,42, "Volts");
      u8g2.drawStr(80,42, "Amps");
      u8g2.setFont(u8g2_font_helvB18_tr); // bottom line
      sprintf(sbuf, "%03.1f", (lastCANMilliVolt / 1000.0));
      u8g2.drawStr(0,64, sbuf);
      sprintf(sbuf, "%01.1f", (lastCANMilliAmp / 1000.0));
      u8g2.drawStr(80,64, sbuf);

      if((millisNow % 1000) > 500) 
      {
        u8g2.drawBox(48,8,15,16);
      }
    }
    else if(canStatus == 0x08) // battery charged
    {
      u8g2.setFont(u8g2_font_open_iconic_embedded_4x_t);
      u8g2.drawGlyph(48, 32, 73); // battery full
      
      u8g2.setFont(u8g2_font_helvB10_tr); // top line
      u8g2.drawStr(0,42, "Battery Charged");
      u8g2.setFont(u8g2_font_helvB18_tr); // bottom line
      sprintf(sbuf, "%03.1fV", (lastCANMilliVolt / 1000.0));
      u8g2.drawStr(0,64, sbuf);
    }
    else if (canStatus > 0) // charger error
    {
      u8g2.setFont(u8g2_font_open_iconic_embedded_4x_t);
      u8g2.drawGlyph(48, 32, 71); // error

      u8g2.setFont(u8g2_font_helvB10_tr); // top line
      switch(canStatus)
      {
        case 0x04:
          u8g2.drawStr(0,42, "Charger Mains Lost");
          break;
        default:
          u8g2.drawStr(0,42, "Charger Error");
          break;
      }
      u8g2.setFont(u8g2_font_helvB18_tr); // bottom line
      sprintf(sbuf, "Code: %02hhX", canStatus);
      u8g2.drawStr(0,64, sbuf);
    }
    else // comms error
    {
      u8g2.setFont(u8g2_font_open_iconic_check_4x_t);
      u8g2.drawGlyph(48, 32, 68);

      u8g2.setFont(u8g2_font_helvB10_tr); // top line
      u8g2.drawStr(0,42, "Comms Error");
      u8g2.setFont(u8g2_font_helvB18_tr); // bottom line
      sprintf(sbuf, "Code: %02hhX", canStatus);
      u8g2.drawStr(0,64, sbuf);
    }
    //u8g2.setFont(u8g2_font_profont22_tr );
    //u8g2.drawStr(5,60,"www.globaloffice.co.nz");
    //u8g2.setFont(u8g2_font_open_iconic_embedded_6x_t);
    //u8g2.drawGlyph(10, 0, 67);  
    //u8g2.drawXBM(0, 2, power_ac_width,  power_ac_height, power_ac_bits);
    //u8g2.drawXBM(20, 2, power_bat2_width,  power_bat2_height, power_bat2_bits);
    //u8g2.drawXBM(40, 2, battery_width,  battery_height, battery_bits);
    //u8g2.drawXBM(60, 2, load_width,  load_height, load_bits);

    u8g2.sendBuffer();
    displayFrameDate = millis();
  }
}

void print_CanMessage(CANMessage * frame)
{
  Serial.print ("Received: ") ;
  Serial.print(gReceivedFrameCount);
  Serial.print (" id: ") ;
  Serial.print(frame->id, HEX);
  Serial.print (" ext: ") ;
  Serial.print(frame->ext, HEX);
  Serial.print (" rtr: ") ;
  Serial.print(frame->rtr, HEX);
  Serial.print (" idx: ") ;
  Serial.print(frame->idx, HEX);
  Serial.print (" len: ") ;
  Serial.print(frame->len, HEX);

  int i = 0;
  while(i < frame->len)
  {
   Serial.print (" ") ;
   Serial.print(frame->data[i++], HEX);
  }  
  Serial.println () ;
}


void process_ChargerCanMessage(CANMessage * frame)
{
  int32_t millivoltSetPoint = charger_millivolt_setpoint;
  int32_t milliampSetPoint = charger_milliamp_setpoint;
  int32_t millivolt = 100 * ( (frame->data[0] << 8) | (frame->data[1] << 0));
  int32_t milliamp = 100 * ( (frame->data[2] << 8) | (frame->data[3] << 0));
  int8_t statusFlags = (frame->data[4] << 0);

  Serial.print ("Charger said: ") ;
  Serial.print(millivolt);
  Serial.print ("mV  ") ;
  Serial.print(milliamp);
  Serial.print ("mA  ") ;
  Serial.print ("status: 0x") ;
  Serial.println(statusFlags, HEX);
  
  CANMessage responseFrame;
  responseFrame.id = 0x1806E5F4; // id of BMS system
  responseFrame.ext = true;
  responseFrame.rtr = false;
  responseFrame.idx = 0;
  responseFrame.len = 8;

  responseFrame.data[0] = (millivoltSetPoint / 100) >> 8;
  responseFrame.data[1] = (millivoltSetPoint / 100) >> 0;
  responseFrame.data[2] = (milliampSetPoint / 100) >> 8;
  responseFrame.data[3] = (milliampSetPoint / 100) >> 0;
  responseFrame.data[4] = 0; // Set status to 0 to turn on charger
  // clear remaining data
  responseFrame.data[5] = 0;
  responseFrame.data[6] = 0;
  responseFrame.data[7] = 0;
  const bool ok = can.tryToSend (responseFrame) ;
  if (ok) {
    Serial.println ("Sent charger response");
  }else{
    Serial.println ("Send charger response failure") ;
  }

  lastCANMilliVolt = millivolt;
  lastCANMilliAmp = milliamp;
  lastCANStatus = statusFlags;
  lastCANMessageDate = millis();
}

//——————————————————————————————————————————————————————————————————————————————

void loop () {
  //can.dispatchReceivedMessage ();
  CANMessage frame;
  if (gBlinkLedDate < millis ()) {
    gBlinkLedDate += 2000 ;
    digitalWrite (LED_BUILTIN, !digitalRead (LED_BUILTIN)) ;
  }

  if((millis() - lastCANMessageDate) > 15000) 
  {
    lastCANStatus = -1;
  }
  
  if (can.available ()) {
    can.receive (frame);
    switch(frame.id)
    {
      case 0x18FF50E5: // Charger address
        print_CanMessage(&frame);
        process_ChargerCanMessage(&frame);
        break;
      default:
        print_CanMessage(&frame);
        break;
    }
  }

  update_display();
  //process_ChargerCanMessage(&frame);
}

//——————————————————————————————————————————————————————————————————————————————
