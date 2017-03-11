#include <Arduino.h>

/**
  Name: BlynkUtils.ino

  Purpose: Meca-ROV utilities to support Blynk control.
  @version: 0.1
*/

//// Reconnect to server if disconnected.
//void reconnectBlynk() {
//  if (!Blynk.connected()) {
//    if (Blynk.connect()) {
//      Serial.println("Reconnected.");
//#ifdef LCD
//      lcd.clear();
//      lcd.setCursor(0, 0);
//      lcd.print("Reconnected.");
//#endif
//    }
//  }
//}

/**
   Reconnect to server if disconnected.
*/
void reconnectBlynk() {
  if (!Blynk.connected()) {
    if (Blynk.connect()) {
      Serial.println("Reconnected");
    } else {
      Serial.println("Not reconnected");
    }
  }
}

BLYNK_CONNECTED() {
  if (isFirstConnect) {
    Serial.println("Blynk client connected.");
    Blynk.syncAll();
    Blynk.syncVirtual(V9);
    isFirstConnect = false;
  }
  Serial.println("Connected...");
#ifdef LCD
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Connected.");
#endif
  // Blynk.virtualWrite(V9, true);
  // Blynk.virtualWrite(V0, 0);
}

// These functions are called every time the Blynk widgets write Virtual Pins.

// Drive Speed.
//BLYNK_WRITE(V0)
//{
//  int newDriveSpeed = param.asInt();
//  if (newDriveSpeed != driveSpeed)
//  {
//    driveSpeed = newDriveSpeed;
//    setDriveSpeed(newDriveSpeed);
//  }
//}

/**
   Set drive speed on all motors.
*/
void setDriveSpeed(int newDriveSpeed) {
  //  Serial.print("Drive Speed: ");
  //  Serial.println(newDriveSpeed);
  motorLF->setSpeed(newDriveSpeed);
  motorLR->setSpeed(newDriveSpeed);
  motorRF->setSpeed(newDriveSpeed);
  motorRR->setSpeed(newDriveSpeed);
}

/**
  Drive Forward.
*/
BLYNK_WRITE(V1)
{
  if (roverActive) {
    int pinValue = param.asInt(); // Assign incoming value from pin V1.
    if (pinValue == 1) {
      timer.restartTimer(0);
#ifdef LCD
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Drive Forward");
      lcd.setCursor(0, 1);
      lcd.print(driveSpeed);
      lcd.setCursor(8, 1);
      lcd.print(hpr[0]);
#endif
      Serial.println(" - Forward");
      setDriveSpeed(driveSpeed);
      motorLF->run(FORWARD);
      motorLR->run(FORWARD);
      motorRF->run(FORWARD);
      motorRR->run(FORWARD);
    }
    if (pinValue == 0) {
      releaseMotors();
    }
  }
}

/**
  Drive Backward.
*/
BLYNK_WRITE(V2)
{
  if (roverActive) {
    int pinValue = param.asInt(); // Assign incoming value from pin V2.
    if (pinValue == 1) {
      timer.restartTimer(0);
#ifdef LCD
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Drive Backward");
      lcd.setCursor(0, 1);
      lcd.print(driveSpeed);
      lcd.setCursor(8, 1);
      lcd.print(hpr[0]);
#endif
      Serial.println(" - Backward");
      setDriveSpeed(driveSpeed);
      motorLF->run(BACKWARD);
      motorLR->run(BACKWARD);
      motorRF->run(BACKWARD);
      motorRR->run(BACKWARD);
    }
    if (pinValue == 0) {
      releaseMotors();
    }
  }
}

/**
  Strafe Left.
*/
BLYNK_WRITE(V3)
{
  if (roverActive) {
    int pinValue = param.asInt(); // Assign incoming value from pin V3.
    if (pinValue == 1) {
      timer.restartTimer(0);
#ifdef LCD
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Strafe Left");
      lcd.setCursor(0, 1);
      lcd.print(driveSpeed);
      lcd.setCursor(8, 1);
      lcd.print(hpr[0]);
#endif
      Serial.println(" - Strafe Left");
      setDriveSpeed(driveSpeed);
      motorLF->run(BACKWARD);
      motorLR->run(FORWARD);
      motorRF->run(FORWARD);
      motorRR->run(BACKWARD);
    }
    if (pinValue == 0) {
      releaseMotors();
    }
  }
}

/**
  Strafe Right.
*/
BLYNK_WRITE(V4)
{
  if (roverActive) {
    int pinValue = param.asInt(); // Assign incoming value from pin V4.
    if (pinValue == 1) {
      timer.restartTimer(0);
#ifdef LCD
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Strafe Right");
      lcd.setCursor(0, 1);
      lcd.print(driveSpeed);
      lcd.setCursor(8, 1);
      lcd.print(hpr[0]);
#endif
      Serial.println(" - Strafe Right");
      setDriveSpeed(driveSpeed);
      motorLF->run(FORWARD);
      motorLR->run(BACKWARD);
      motorRF->run(BACKWARD);
      motorRR->run(FORWARD);
    }
    if (pinValue == 0) {
      releaseMotors();
    }
  }
}

/**
  Rotate Left.
*/
BLYNK_WRITE(V5)
{
  if (roverActive) {
    int pinValue = param.asInt(); // Assign incoming value from pin V5.
    if (pinValue == 1) {
      timer.restartTimer(0);
#ifdef LCD
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Rotate Left");
      lcd.setCursor(0, 1);
      lcd.print(driveSpeed);
      lcd.setCursor(8, 1);
      lcd.print(hpr[0]);
#endif
      Serial.println(" - Rotate Left");
      // Modify rotation rate.
      setDriveSpeed(driveSpeed * rotateSpeed);
      motorLF->run(BACKWARD);
      motorLR->run(BACKWARD);
      motorRF->run(FORWARD);
      motorRR->run(FORWARD);
    }
    if (pinValue == 0) {
      releaseMotors();
      setDriveSpeed(driveSpeed);
    }
  }
}

/**
  Rotate Right.
*/
BLYNK_WRITE(V6)
{
  if (roverActive) {
    int pinValue = param.asInt(); // assigning incoming value from pin V6.
    if (pinValue == 1) {
      timer.restartTimer(0);
#ifdef LCD
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Rotate Right");
      lcd.setCursor(0, 1);
      lcd.print(driveSpeed);
      lcd.setCursor(8, 1);
      lcd.print(hpr[0]);
#endif
      Serial.println(" - Rotate Right");
      // Modify rotation rate.
      setDriveSpeed(driveSpeed * rotateSpeed);
      motorLF->run(FORWARD);
      motorLR->run(FORWARD);
      motorRF->run(BACKWARD);
      motorRR->run(BACKWARD);
    }
    if (pinValue == 0) {
      releaseMotors();
      setDriveSpeed(driveSpeed);
    }
  }
}

/**
  All Halt. (Releases all motors.)
*/
BLYNK_WRITE(V7)
{
  int pinValue = param.asInt(); // Assign incoming value from pin V7.
  if (pinValue == 1) {
    timer.restartTimer(0);
#ifdef LCD
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("All Halt");
#endif
    Serial.println(" - All Halt");
  }
  if (pinValue == 0) {
    releaseMotors();
  }
}

/**
  Reset motors and LCD display.
*/
BLYNK_WRITE(V8)
{
  int pinValue = param.asInt(); // Assign incoming value from pin V8.
  if (pinValue == 1) {
    releaseMotors();
    timer.restartTimer(0);
    CurieIMU.autoCalibrateGyroOffset();
    CurieIMU.autoCalibrateAccelerometerOffset(X_AXIS, 0);
    CurieIMU.autoCalibrateAccelerometerOffset(Y_AXIS, 0);
    CurieIMU.autoCalibrateAccelerometerOffset(Z_AXIS, 1);
#ifdef LCD
    lcd.setRGB(BLBlue[0], BLBlue[1], BLBlue[2]);
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
    // Set timers.
    timer.enable(0);
    timer.enable(1);
  }
}

/**
  Toggle active state.
*/
BLYNK_WRITE(V9)
{
  int pinValue = param.asInt(); // Assign incoming value from pin V9.
  if (pinValue == 1) {
    roverActive = 1;
#ifdef LCD
    lcd.display();
    lcd.setRGB(BLBlue[0], BLBlue[1], BLBlue[2]);
    lcd.clear();
    lcd.print("    Meca-ROV    ");
#endif
    // Set timers.
    timer.enable(0);
    timer.enable(1);
  }
  else {
    roverActive = 0;
    releaseMotors();
#ifdef LCD
    lcd.clear();
    lcd.noDisplay();
#endif
  }
}

/**
  Left Joystick ("Sticks").
  This controls the x and y values for the speed the rover should drive in each direction.
*/
BLYNK_WRITE(V13)
{
  // We swap our x and y here so that the device may be used in landscape mode.
  // This should work natively within Blynk, but it doesn't seem to.
  x = param[1].asFloat();
  y = param[0].asFloat();

  xMapped = mapGeneric(x, 0, 255, -1, 1) * -1; // Invert this axis.
  yMapped = mapGeneric(y, 0, 255, -1, 1);
  rotMapped = mapGeneric(rotation, 0, 255, -1, 1);

  MecanumDriveCartesian(xMapped, yMapped, rotMapped, hpr[0]);

  //  DEBUG_PRINT("x: ");
  //  dtostrf(xMapped, 4, 2, xBuf);
  //  DEBUG_PRINTLN(xBuf);
  //  DEBUG_PRINT("y: ");
  //  dtostrf(yMapped, 4, 2, yBuf);
  //  DEBUG_PRINTLN(yBuf);
  //  DEBUG_PRINTLN();
}

/**
  Right Joystick ("Sticks").
  The x value represents the rotation of the rover.
*/
BLYNK_WRITE(V14)
{
  rotation = abs(param[1].asInt() - 255);

  float rot = mapGeneric(rotation, 0, 255, -1, 1);
  Serial.println("Rotation: " + (String)rot);
  MecanumDriveCartesian(xMapped, yMapped, rot, hpr[0]);
}

