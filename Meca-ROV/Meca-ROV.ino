#include <Arduino.h>

// #define BLYNK_DEBUG
// #define BLYNK_PRINT Serial
// #define DEBUG
#define BLYNK_USE_DIRECT_CONNECT
#define M_PI 3.14159265358979323846264338327950288

/*
  Mecanum wheeled rover control using Arduino 101, Adafruit Motor Shield v2.3,
  and Blynk.
  Displays orientation/contol info on a Seeed Grove RGB LCD.

  https://www.arduino.cc/en/Main/ArduinoBoard101
  http://www.adafruit.com/products/1438
  http://docs.blynk.cc/#blynk-server
  http://wiki.seeed.cc/Grove-LCD_RGB_Backlight/

  General idea behind control for Mecanum wheels.
  This is for a "freeform" control system.
  Wheels should form an "X". (Each toed in at a 45 degree angle.)

    1      3
    LF    RF
    \\    //
    \\    //
      |  |
    //    \\
    //    \\
    LR    RR
    2      4

    (1) Left Front Wheel  = Left Joystick Y-Axis + Left Joystick X-Axis + Right
  Joystick X-Axis
    (2) Left Rear Wheel   = Left Joystick Y-Axis - Left Joystick X-Axis + Right
  Joystick X-Axis
    (3) Right Front Wheel = Left Joystick Y-Axis - Left Joystick X-Axis - Right
  Joystick X-Axis
    (4) Right Rear Wheel  = Left Joystick Y-Axis + Left Joystick X-Axis - Right
  Joystick X-Axis

  Madgwick's filter algorithm is used to calculate quaternions from the 6-axis
  accelerometer/gyroscope.
  https://en.wikipedia.org/wiki/Quaternion
  The quarternions are then used to calculate Euler angles Yaw, Pitch, and Roll.
*/

// Uncomment to use the Grove Seeed RGB LCD display.
#define LCD

#include "CurieIMU.h"
#include "mapGeneric.h"
#include <Adafruit_MotorShield.h>
#include <BlynkSimpleCurieBLE.h>
#include <CurieBLE.h>
#include <MadgwickAHRS.h>
#include <SimpleTimer.h>

#ifdef LCD
#include <rgb_lcd.h>
#endif

// Blynk Auth Token.
// Local Blynk Server.
char auth[] = "5dca16ba842a4450b6c13c83742c2b7d";
// Cloud Blynk Server.
// char auth[] = "10272ad72bc8481497e4274ad9743c82";

// Command received from BLE.
// String cmdInput;
// String lastCmd = "";

BLEPeripheral blePeripheral;

Madgwick filter;

// BLEService uartService = BLEService(auth);
// Create characteristics.
// BLECharacteristic rxCharacteristic = BLECharacteristic(auth,
// BLEWriteWithoutResponse, 20);
// BLECharacteristic txCharacteristic = BLECharacteristic(auth, BLENotify , 20);

bool connectionStatus = false;

#ifdef DEBUG
#define XDEBUG_PRINT(str)                                                      \
  Serial.print(millis());                                                      \
  Serial.print(": ");                                                          \
  Serial.print(__PRETTY_FUNCTION__);                                           \
  Serial.print(' ');                                                           \
  Serial.print(__FILE__);                                                      \
  Serial.print(':');                                                           \
  Serial.print(__LINE__);                                                      \
  Serial.print(' ');                                                           \
  Serial.println(str);
#define DEBUG_PRINT(str) Serial.print(str);
#define DEBUG_PRINTLN(str) Serial.println(str);
#else
#define DEBUG_PRINT(str)
#define DEBUG_PRINTLN(str)
#endif

SimpleTimer timer;

// Joystick control values ("Sticks").
float x, y, rotation = 128;
float xMapped, yMapped, rotMapped;

// Is MecaROV active?
bool roverActive = 1;

// Is first time connected?
bool isFirstConnect = true;

// Determine whether IMU calibration takes place or not.
int calibrateOffsets = 1;

// Track if we have a serial connection.
bool serialConnected = 0;

// Variable by which to divide gyroscope values. Used to control sensitivity.
int gFactor = 1;

unsigned long microsPerReading, microsPrevious;
unsigned long microsNow;

unsigned long previousMillis = 0;
boolean state = 1;

float accelScale, gyroScale;
float hpr[3]; // heading, pitch, roll

// Define a "Deadzone" for small values of x,y (Sticks).
int threshold = 10;

// Individual wheel values.
int leftFront, leftRear, rightFront, rightRear = 0;
int driveSpeed = 255;
float rotateSpeed = 0.6f;
double maxPower = 255;

// Create the motor shield object with the default I2C address.
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *motorLF = AFMS.getMotor(1); // Left Front
Adafruit_DCMotor *motorLR = AFMS.getMotor(2); // Left Rear
Adafruit_DCMotor *motorRF = AFMS.getMotor(3); // Right Front
Adafruit_DCMotor *motorRR = AFMS.getMotor(4); // Right Rear

#ifdef LCD
rgb_lcd lcd;
unsigned char BLRed[] = {0xFF, 0x00, 0x00};   // Red
unsigned char BLGreen[] = {0x00, 0xFF, 0x00}; // Green
unsigned char BLBlue[] = {0x00, 0x55, 0xFF};  // Blue
const byte lcdNumRows = 2;
const byte lcdNumCols = 16;
#endif

int aix, aiy, aiz = 0;
int gix, giy, giz = 0;
float ax, ay, az = 0;
float gx, gy, gz = 0;

/**
  Compare previous millis to current one and wait until the interval has been
  reached in a do loop
  Also set / reset waiting states so the loop keeps looping until the interval
  has been reached
*/
void wait(unsigned long interval) {
  do {
    if ((unsigned long)(millis() - previousMillis) >= interval) {
      previousMillis = millis();
      state = 0;
      break;
    }
  } while (state = 1);
}

/**
   Sends updated feedback every second to the Blynk app.
  (Make sure to set Blynk Widgets' reading frequency to PUSH.)
*/
void updateRemote() {
  // Don't send more than 10 events per second if you are using Blynk Cloud
  // or they will give you a pedazo. You can increase this on a local server.
  Blynk.virtualWrite(V12, hpr[0]); // Send yaw.
}

/**
   Translate accelerometer reading.
*/
float convertRawAcceleration(int aRaw) {
  // 2G range.
  // -2g maps to a raw value of -32768
  // +2g maps to a raw value of 32767
  float a = (aRaw * 2.0f) / 32768.0f;
  return a;
}

/**
   Translate gyro reading.
*/
float convertRawGyro(int gRaw) {
  // 250 degrees/second range.
  // -250 maps to a raw value of -32768
  // +250 maps to a raw value of 32767
  float g = (gRaw * 250.0f) / 32768.0f;
  return g;
}

// static void crashCallback(void)
//{
//  if (CurieIMU.getInterruptStatus(CURIE_IMU_SHOCK)) {
//    if (CurieIMU.shockDetected(X_AXIS, POSITIVE)) {
//      releaseMotors();
//      timer.disable(0);
//      timer.disable(1);
//      wait(100);
//      DEBUG_PRINTLN();
//      DEBUG_PRINT("Negative shock detected on X-axis");
//#ifdef LCD
//      lcd.setRGB(BLRed[0], BLRed[1], BLRed[2]);
//      lcd.clear();
//      sendToLCDDisplay("(-) X-axis Shock");
//#endif
//    }
//    if (CurieIMU.shockDetected(X_AXIS, NEGATIVE)) {
//      releaseMotors();
//      timer.disable(0);
//      timer.disable(1);
//      wait(100);
//      DEBUG_PRINTLN();
//      DEBUG_PRINT("Positive shock detected on X-axis");
//#ifdef LCD
//      lcd.setRGB(BLRed[0], BLRed[1], BLRed[2]);
//      lcd.clear();
//      sendToLCDDisplay("(+) X-axis Shock");
//#endif
//    }
//    if (CurieIMU.shockDetected(Y_AXIS, POSITIVE)) {
//      releaseMotors();
//      timer.disable(0);
//      timer.disable(1);
//      wait(100);
//      DEBUG_PRINTLN();
//      DEBUG_PRINT("Negative shock detected on Y-axis");
//#ifdef LCD
//      lcd.setRGB(BLRed[0], BLRed[1], BLRed[2]);
//      lcd.clear();
//      sendToLCDDisplay("(-) Y-axis Shock");
//#endif
//    }
//    if (CurieIMU.shockDetected(Y_AXIS, NEGATIVE)) {
//      releaseMotors();
//      timer.disable(0);
//      timer.disable(1);
//      wait(100);
//      DEBUG_PRINTLN();
//      DEBUG_PRINT("Positive shock detected on Y-axis");
//#ifdef LCD
//      lcd.setRGB(BLRed[0], BLRed[1], BLRed[2]);
//      lcd.clear();
//      sendToLCDDisplay("(+) Y-axis Shock");
//#endif
//    }
//    if (CurieIMU.shockDetected(Z_AXIS, POSITIVE)) {
//      releaseMotors();
//      timer.disable(0);
//      timer.disable(1);
//      wait(100);
//      DEBUG_PRINTLN();
//      DEBUG_PRINT("Negative shock detected on Z-axis");
//#ifdef LCD
//      lcd.setRGB(BLRed[0], BLRed[1], BLRed[2]);
//      lcd.clear();
//      sendToLCDDisplay("(-) Z-axis Shock");
//#endif
//    }
//    if (CurieIMU.shockDetected(Z_AXIS, NEGATIVE)) {
//      releaseMotors();
//      timer.disable(0);
//      timer.disable(1);
//      wait(100);
//      DEBUG_PRINTLN();
//      DEBUG_PRINT("Positive shock detected on Z-axis");
//#ifdef LCD
//      lcd.setRGB(BLRed[0], BLRed[1], BLRed[2]);
//      lcd.clear();
//      sendToLCDDisplay("(+) Z-axis Shock");
//#endif
//    }
//  }
//}

/**
   Setup.
*/
void setup() {
  Serial.begin(9600);
  wait(1000);

  // timer.setInterval(11000L, checkConnection); // check if still connected
  // every 11 seconds

  blePeripheral.setLocalName("Meca-ROV");
  blePeripheral.setDeviceName("Meca-ROV");
  blePeripheral.setAppearance(384);

  Blynk.begin(blePeripheral, auth);

  blePeripheral.begin();

  pinMode(LED_BUILTIN, OUTPUT);

#ifdef LCD
  // Set up the LCD's number of columns and rows.
  lcd.begin(lcdNumCols, lcdNumRows);
  lcd.setRGB(BLBlue[0], BLBlue[1], BLBlue[2]);
  lcd.clear();
  lcd.print("    Meca-ROV    ");
  lcd.setCursor(0, 1);
  lcd.print("connect monitor");
#endif

  // while (!Serial);

  //  // Wait for ten seconds or until data is available on serial,
  //  // whichever occurs first.
  //  while (Serial.available() == 0 && millis() < 10000);
  //
  //  // On timeout or availability of data, we come here.
  //  if (Serial.available() > 0)
  //  {
  //    // If data is available, we enter here.
  //    serialConnected = 1;
  //
  //    int test = Serial.read(); // We then clear the input buffer.
  //
  //    Serial.println("DEBUG MODE: Serial connected.");
  //#ifdef LCD
  //    lcd.setCursor(0, 1);
  //    lcd.print("monitor present");
  //    wait(2000);
  //#endif
  //  }
  //  else
  //  {
  //#ifdef LCD
  //    lcd.setCursor(0, 1);
  //    lcd.print("standalone mode");
  //    // wait(2000);
  //#endif
  //  }

  // blePeripheral.setAdvertisedServiceUuid(uartService.uuid());
  // add service, rx and tx characteristics.
  // blePeripheral.addAttribute(uartService);

  // blePeripheral.addAttribute(rxCharacteristic);
  // blePeripheral.addAttribute(txCharacteristic);
  // Assign event handlers for connected, disconnected to peripheral.
  // blePeripheral.setEventHandler(BLEConnected, blePeripheralConnectHandler);
  // blePeripheral.setEventHandler(BLEDisconnected,
  // blePeripheralDisconnectHandler);
  // rxCharacteristic.setEventHandler(BLEWritten, rxCharacteristicWritten);
  // wait(5000);

  outputTitle();

  // 888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888
  // Start the IMU and filter.
  Serial.println("Initializing IMU device...");

  // Verify connection.
  Serial.println("Testing device IMU connections...");
  if (CurieIMU.begin()) {
    Serial.println("CurieIMU connection successful.\n");
  } else {
    Serial.println("CurieIMU connection failed.\n");
    Serial.println("Exiting...");
    exit(0);
  }

  CurieIMU.setGyroRate(25);
  CurieIMU.setAccelerometerRate(25);
  // CurieIMU.setGyroRate(BMI160_GYRO_RATE_100HZ);
  // CurieIMU.setFullScaleAccelRange(BMI160_ACCEL_RANGE_2G);
  // CurieIMU.setFullScaleGyroRange(BMI160_GYRO_RANGE_250);

  filter.begin(25);

  // Set the accelerometer range to 2G.
  CurieIMU.setAccelerometerRange(2);
  // Set the gyroscope range to 250 degrees/second.
  CurieIMU.setGyroRange(250);

  // Serial.println("Enabling Gyroscope/Acceleration offset compensation");
  // * CurieIMU.setGyroOffsetEnabled(true);
  // * CurieIMU.setAccelOffsetEnabled(true);

  // IMU device must be resting in a horizontal position for the following
  // calibration procedure to work correctly!
  // Calibrate accel/gyro offset values.
  if (calibrateOffsets == 1) {
    SetupIMU();
  }

  // CurieIMU.attachInterrupt(crashCallback);

  // Enable Shock Detection.
  // CurieIMU.setDetectionThreshold(CURIE_IMU_SHOCK, 1500); // 1.5g = 1500 mg
  // CurieIMU.setDetectionDuration(CURIE_IMU_SHOCK, 50); // 50ms
  // CurieIMU.interrupts(CURIE_IMU_SHOCK);

  // Set up a function to update the IMU display once per second.
  timer.setInterval(1000L, updateIMUDisplay);

// wait(500);

// Set up a function to write updates to Blynk remote once per second.
// timer.setInterval(1000L, updateRemote);

#ifdef LCD
  lcd.clear();
  lcd.print("    Meca-ROV    ");
  wait(1000);

  for (int i = 0; i <= 16; i++) {
    lcd.setCursor(i, 1);
    lcd.print("âšª");
    wait(50);
  }

  breatheDisplay(REG_RED);
  wait(2000);
  lcd.clear();
  lcd.print("    Meca-ROV    ");
#endif

  AFMS.begin();

  initMotors();

  timer.setInterval(15000L, reconnectBlynk); // Check every 15 seconds if we are
                                             // connected to the server.

  // Pace updates to correct rate.
  microsPerReading = 1000000.0f / 25;
  microsPrevious = micros();

  Serial.println("Meca-ROV initialized.");
}

/**
   Loop.
*/
void loop() {
  previousMillis = millis(); // Reset counter
  state = 1;                 // Reset waiting state

  // check if it's time to read data and update the filter
  microsNow = micros();
  if ((microsNow - microsPrevious >= microsPerReading) && (roverActive)) {

    // read gyro measurements from device, scaled to the configured range
    // CurieIMU.readGyroScaled(gx, gy, gz);

    // read raw data from CurieIMU
    CurieIMU.readMotionSensor(aix, aiy, aiz, gix, giy, giz);

    // Convert from raw data to gravity and degrees/second units.
    ax = convertRawAcceleration(aix);
    ay = convertRawAcceleration(aiy);
    az = convertRawAcceleration(aiz);
    gx = convertRawGyro(gix);
    gy = convertRawGyro(giy);
    gz = convertRawGyro(giz);

    // Update the filter, which computes orientation.
    filter.updateIMU(gx / gFactor, gy / gFactor, gz / gFactor, ax, ay, az);

    // We want yaw to be tared at 0 degrees increasing clockwise.
    hpr[2] = filter.getRoll();
    hpr[1] = filter.getPitch();
    hpr[0] = (filter.getYaw() - 180) * -1;

    // Increment previous time, so we keep proper pace.
    microsPrevious = microsPrevious + microsPerReading;
  }

  // if (Blynk.connected())
  // {
  Blynk.run();
  // }
  timer.run();
  blePeripheral.poll();
}

/**
   Connection handler.
*/
// void blePeripheralConnectHandler(BLECentral & central) {
//  // Connected event handler.
//  Serial.println();
//  Serial.print("Connected: ");
//  Serial.println(central.address());
//#ifdef LCD
//  lcd.display();
//#endif
//  digitalWrite(LED_BUILTIN, HIGH);
//  connectionStatus = true;
//}

/**
   Disconnection handler.
*/
// void blePeripheralDisconnectHandler(BLECentral & central) {
//  // Disconnected event handler.
//  Serial.println();
//  Serial.print("Disconnected: ");
//  Serial.println(central.address());
//#ifdef LCD
//  lcd.noDisplay();
//#endif
//  digitalWrite(LED_BUILTIN, LOW);
//  connectionStatus = false;
//}
