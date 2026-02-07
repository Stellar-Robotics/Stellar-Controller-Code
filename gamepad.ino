#include <Rotary.h>

/*********************************************************************
 Adafruit invests time and resources providing this open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!

 MIT license, check LICENSE for more information
 Copyright (c) 2021 NeKuNeKo for Adafruit Industries
 All text above, and the splash screen below must be included in
 any redistribution
*********************************************************************/

#include "Adafruit_TinyUSB.h"

int R1 = 14;
int R2 = 9;
int R4 = 12;
int L1A = 19;
int L1B = 22;
int L2 = 20;
int L3 = 17;
int L4 = 23;
int SE1 = 3;
int SE2 = 21;
int ST1 = 2;
int ST2 = 11;
int H1 = 18;
int C = 15;
int A = 8;
int B = 10;

int xMin = 176;
int xMax = 910;

int yMin = 174;
int yMax = 908;

// HID report descriptor using TinyUSB's template
// Single Report (no ID) descriptor
uint8_t const desc_hid_report[] = {
HID_USAGE_PAGE ( HID_USAGE_PAGE_DESKTOP     )                 ,\
  HID_USAGE      ( HID_USAGE_DESKTOP_GAMEPAD  )                 ,\
  HID_COLLECTION ( HID_COLLECTION_APPLICATION )                 ,\
    /* 8 bit X, Y, Z, Rz, Rx, Ry (min -127, max 127 ) */ \
    HID_USAGE_PAGE     ( HID_USAGE_PAGE_DESKTOP                 ) ,\
    HID_USAGE          ( HID_USAGE_DESKTOP_X                    ) ,\
    HID_USAGE          ( HID_USAGE_DESKTOP_Y                    ) ,\
    HID_USAGE          ( HID_USAGE_DESKTOP_Z                    ) ,\
    HID_USAGE          ( HID_USAGE_DESKTOP_RZ                   ) ,\
    HID_USAGE          ( HID_USAGE_DESKTOP_RX                   ) ,\
    HID_USAGE          ( HID_USAGE_DESKTOP_RY                   ) ,\
    HID_LOGICAL_MIN    ( 0x81                                   ) ,\
    HID_LOGICAL_MAX    ( 0x7f                                   ) ,\
    HID_REPORT_COUNT   ( 6                                      ) ,\
    HID_REPORT_SIZE    ( 8                                      ) ,\
    HID_INPUT          ( HID_DATA | HID_VARIABLE | HID_ABSOLUTE ) ,\
    /* 8 bit DPad/Hat Button Map  */ \
    HID_USAGE_PAGE     ( HID_USAGE_PAGE_DESKTOP                 ) ,\
    HID_USAGE          ( HID_USAGE_DESKTOP_HAT_SWITCH           ) ,\
    HID_LOGICAL_MIN    ( 1                                      ) ,\
    HID_LOGICAL_MAX    ( 8                                      ) ,\
    HID_PHYSICAL_MIN   ( 0                                      ) ,\
    HID_PHYSICAL_MAX_N ( 315, 2                                 ) ,\
    HID_REPORT_COUNT   ( 1                                      ) ,\
    HID_REPORT_SIZE    ( 8                                      ) ,\
    HID_INPUT          ( HID_DATA | HID_VARIABLE | HID_ABSOLUTE ) ,\
    /* 15 bit Button Map */ \
    HID_USAGE_PAGE     ( HID_USAGE_PAGE_BUTTON                  ) ,\
    HID_USAGE_MIN      ( 1                                      ) ,\
    HID_USAGE_MAX      ( 15                                     ) ,\
    HID_LOGICAL_MIN    ( 0                                      ) ,\
    HID_LOGICAL_MAX    ( 1                                      ) ,\
    HID_REPORT_COUNT   ( 15                                     ) ,\
    HID_REPORT_SIZE    ( 1                                      ) ,\
    HID_INPUT          ( HID_DATA | HID_VARIABLE | HID_ABSOLUTE ) ,\
    HID_REPORT_COUNT(17),\
    HID_INPUT(HID_CONSTANT | HID_VARIABLE | HID_ABSOLUTE),\
    HID_USAGE_MIN(1),\
    HID_USAGE_MAX(24),\
    HID_REPORT_COUNT(24),\
    HID_OUTPUT(HID_DATA | HID_VARIABLE | HID_ABSOLUTE),\
  HID_COLLECTION_END \
};

// USB HID object
Adafruit_USBD_HID usb_hid;

// Report payload defined in src/class/hid/hid.h
// - For Gamepad Button Bit Mask see  hid_gamepad_button_bm_t
// - For Gamepad Hat    Bit Mask see  hid_gamepad_hat_t
hid_gamepad_report_t    gp;

Rotary r = Rotary(A, B);

void setup() 
{
  pinMode(R1, INPUT_PULLUP);
  pinMode(R2, INPUT_PULLUP);
  pinMode(R4, INPUT_PULLUP);
  pinMode(L1A, INPUT_PULLUP);
  pinMode(L1B, INPUT_PULLUP);
  pinMode(L2, INPUT_PULLUP);
  pinMode(L3, INPUT_PULLUP);
  pinMode(L4, INPUT_PULLUP);
  pinMode(SE1, INPUT_PULLUP);
  pinMode(SE2, INPUT_PULLUP);
  pinMode(ST1, INPUT_PULLUP);
  pinMode(ST2, INPUT_PULLUP);
  pinMode(H1, INPUT_PULLUP);
  pinMode(C, INPUT_PULLUP);

#if defined(ARDUINO_ARCH_MBED) && defined(ARDUINO_ARCH_RP2040)
  // Manual begin() is required on core without built-in support for TinyUSB such as mbed rp2040
  TinyUSB_Device_Init(0);
#endif

  Serial.begin(115200);

  r.setChangedHandler(rotate);
  r.setLeftRotationHandler(showDirection);
  r.setRightRotationHandler(showDirection);

  //r.setUpperBound(LONG_MAX);
  //r.setLowerBound(LONG_MAX);

  //usb_hid.enableOutEndpoint(true);
  // Setup HID
  usb_hid.setPollInterval(2);
  usb_hid.setReportDescriptor(desc_hid_report, sizeof(desc_hid_report));

  usb_hid.setStringDescriptor("Stellar Controller");

  usb_hid.setReportCallback(get_report_callback, hid_report_callback);

  usb_hid.begin();

  // wait until device mounted
  while( !TinyUSBDevice.mounted() ) delay(1);
  
  Serial.println("Adafruit TinyUSB HID Gamepad example");
  gp.z = -127;
}
void loop() 
{
  r.loop();
  if ( !usb_hid.ready() ) return;
  int x = analogRead(A1);
  if (x < xMin) xMin = x;
  if (x > xMax) xMax = x;
  //Serial.println(xMin);
  //Serial.println(xMax);
  gp.x = map(x, xMin, xMax, -127, 127);
  int y = analogRead(A0);
  if (y < yMin) yMin = y;
  if (y > yMax) yMax = y;
  //Serial.println(yMin);
  //Serial.println(yMax);
  gp.y = map(y, yMin, yMax, -127, 127);
  
  // Test buttons (up to 32 buttons)
  gp.buttons = 0;
  gp.buttons |= !digitalRead(R1) << 0;
  gp.buttons |= !digitalRead(R2) << 1;
  gp.buttons |= !digitalRead(R4) << 2;
  gp.buttons |= !digitalRead(L1A) << 3;
  gp.buttons |= (digitalRead(L1A) & digitalRead(L1B)) << 4;
  gp.buttons |= !digitalRead(L1B) << 5;
  gp.buttons |= !digitalRead(L2) << 6;
  gp.buttons |= !digitalRead(L3) << 7;
  gp.buttons |= !digitalRead(L4) << 8;
  gp.buttons |= !digitalRead(SE1) << 9;
  gp.buttons |= !digitalRead(SE2) << 10;
  gp.buttons |= !digitalRead(ST1) << 11;
  gp.buttons |= !digitalRead(ST2) << 12;
  gp.buttons |= !digitalRead(H1) << 13;
  gp.buttons |= !digitalRead(C) << 14;
  usb_hid.sendReport(0, &gp, sizeof(gp));
}

void printBin(byte aByte) {
  for (int8_t aBit = 7; aBit >= 0; aBit--)
    Serial.write(bitRead(aByte, aBit) ? '1' : '0');
}

uint16_t get_report_callback (uint8_t report_id, hid_report_type_t report_type, uint8_t* buffer, uint16_t reqlen)
{
  // not used in this example
  (void) report_id;
  (void) report_type;
  (void) buffer;
  (void) reqlen;
  return 0;
}

// Invoked when received SET_REPORT control request or
// received data on OUT endpoint ( Report ID = 0, Type = 0 )
void hid_report_callback(uint8_t report_id, hid_report_type_t report_type, uint8_t const* buffer, uint16_t bufsize)
{
  // This example doesn't use multiple report and report ID
  (void) report_id;
  (void) report_type;
  printBin(buffer[2]);
  printBin(buffer[1]);
  printBin(buffer[0]);
  Serial.println( );
  if ((buffer[0] + buffer[1] +buffer[2]) > 0 ){
    
  } else {
    
  };

  // echo back anything we received from host
  usb_hid.sendReport(0, buffer, bufsize);
}

void rotate(Rotary& r) {
  long Rotation = r.getPosition();
  if (Rotation >=0) {
    gp.z = map(Rotation%2048, 0, 2048, 127, -127);
    Serial.println(gp.z);
  } else {
    gp.z = map(2048+(Rotation%2048), 0, 2048, 127, -127);
    Serial.println(gp.z);
  }
  /*
   // Analog Trigger 2 UP
  //Serial.println("Analog Trigger 2 UP");
  gp.rz = 127;
  usb_hid.sendReport(0, &gp, sizeof(gp));
  delay(2000);
  
  // Analog Trigger 2 DOWN
  //Serial.println("Analog Trigger 2 DOWN");
  gp.rz = -127;
  usb_hid.sendReport(0, &gp, sizeof(gp));
  delay(2000);

  // Analog Trigger 2 CENTER
  //Serial.println("Analog Trigger 2 CENTER");
  gp.rz = 0;
  usb_hid.sendReport(0, &gp, sizeof(gp));
  delay(2000);
  */
   Serial.println(r.getPosition());
   Serial.println(r.getUpperBound());
   //usb_hid.sendReport(0, &gp, sizeof(gp));
}

// on left or right rotattion
void showDirection(Rotary& r) {
  Serial.println(r.directionToString(r.getDirection()));
}
