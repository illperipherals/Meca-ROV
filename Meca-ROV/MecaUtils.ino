#include <Arduino.h>

/**
  Name: MecaUtils.ino

  Purpose: Meca-ROV utilities to support field-centric Mecanum control.
  @version: 0.1
*/

/**
  Set initial state of all motors.
*/
void initMotors() {
  Serial.println("Initializing motors...");
  // Set starting speed, 0 (off) to 255 (max speed).
  motorLF->setSpeed(255);
  motorLR->setSpeed(255);
  motorRF->setSpeed(255);
  motorRR->setSpeed(255);
  releaseMotors();

  Blynk.syncAll();
  Blynk.syncVirtual(V9);
}

/**
  Set all motors to release.
*/
void releaseMotors() {
  motorLF->run(RELEASE);
  motorLR->run(RELEASE);
  motorRF->run(RELEASE);
  motorRR->run(RELEASE);
}

/**
  Get the 6-axis accel/gyro calibrated.
*/
void SetupIMU() {
  Serial.println("Accelerometer offsets BEFORE calibration...");
  Serial.print(CurieIMU.getAccelerometerOffset(X_AXIS));
  Serial.print("\t");
  Serial.print(CurieIMU.getAccelerometerOffset(Y_AXIS));
  Serial.print("\t");
  Serial.println(CurieIMU.getAccelerometerOffset(Z_AXIS));
  Serial.print("\n");

#ifdef LCD
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Accel. PRE:");
  lcd.setCursor(0, 1);
  lcd.print(CurieIMU.getAccelerometerOffset(X_AXIS), 1);
  lcd.setCursor(5, 1);
  lcd.print(CurieIMU.getAccelerometerOffset(Y_AXIS), 1);
  lcd.setCursor(10, 1);
  lcd.print(CurieIMU.getAccelerometerOffset(Z_AXIS), 1);
#endif

  wait(2000);
  Serial.println("Gyro offsets BEFORE calibration...");
  Serial.print(CurieIMU.getGyroOffset(X_AXIS));
  Serial.print("\t");
  Serial.print(CurieIMU.getGyroOffset(Y_AXIS));
  Serial.print("\t");
  Serial.println(CurieIMU.getGyroOffset(Z_AXIS));
  Serial.print("\n");

#ifdef LCD
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Gyro. PRE:");
  lcd.setCursor(0, 1);
  lcd.print((CurieIMU.getGyroOffset(X_AXIS)), 1);
  lcd.setCursor(6, 1);
  lcd.print((CurieIMU.getGyroOffset(Y_AXIS)), 1);
  lcd.setCursor(11, 1);
  lcd.print((CurieIMU.getGyroOffset(Z_AXIS)), 1);
#endif

  wait(2000);
  Serial.println(
    "About to calibrate. Make sure your board is stable and level.");

#ifdef LCD
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(" * HOLD FAST! * ");
#endif

  wait(2000);
  // The board must be resting in a horizontal position for
  // the following calibration procedure to work correctly.
  Serial.println(
    "Starting Gyroscope calibration and enabling offset compensation...");
  CurieIMU.autoCalibrateGyroOffset();
  Serial.println(" Done.");

  Serial.println(
    "Starting Acceleration calibration and enabling offset compensation...");
  CurieIMU.autoCalibrateAccelerometerOffset(X_AXIS, 0);
  CurieIMU.autoCalibrateAccelerometerOffset(Y_AXIS, 0);
  CurieIMU.autoCalibrateAccelerometerOffset(Z_AXIS, 1);
  Serial.println(" Done.");
  Serial.print("\n");

  Serial.println("Accelerometer offsets AFTER calibration...");
  Serial.print(CurieIMU.getAccelerometerOffset(X_AXIS));
  Serial.print("\t");
  Serial.print(CurieIMU.getAccelerometerOffset(Y_AXIS));
  Serial.print("\t");
  Serial.println(CurieIMU.getAccelerometerOffset(Z_AXIS));
  Serial.print("\n");

#ifdef LCD
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Accel. POST:");
  lcd.setCursor(0, 1);
  lcd.print((CurieIMU.getAccelerometerOffset(X_AXIS)), 1);
  lcd.setCursor(6, 1);
  lcd.print((CurieIMU.getAccelerometerOffset(Y_AXIS)), 1);
  lcd.setCursor(11, 1);
  lcd.print((CurieIMU.getAccelerometerOffset(Z_AXIS)), 1);
#endif

  wait(1000);
  Serial.println("Gyro offsets AFTER calibration...");
  Serial.print(CurieIMU.getGyroOffset(X_AXIS));
  Serial.print("\t");
  Serial.print(CurieIMU.getGyroOffset(Y_AXIS));
  Serial.print("\t");
  Serial.print(CurieIMU.getGyroOffset(Z_AXIS));
  Serial.println("\n");

#ifdef LCD
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Gyro. POST:");
  lcd.setCursor(0, 1);
  lcd.print((CurieIMU.getGyroOffset(X_AXIS)), 1);
  lcd.setCursor(6, 1);
  lcd.print((CurieIMU.getGyroOffset(Y_AXIS)), 1);
  lcd.setCursor(11, 1);
  lcd.print((CurieIMU.getGyroOffset(Z_AXIS)), 1);
#endif
  wait(1000);
}

/**
   Rotate vector in Cartesian space.
*/
// if "angle" is measured CLOCKWISE from the zero reference:
// temp = y*cos(angle) + x*sin(angle);
// x = -y*sin(angle) + x*cos(angle);
// y = temp;
void RotateVector(double &x, double &y, double angle) {
  double cosA = cos(angle * (M_PI / 180.0));
  double sinA = sin(angle * (M_PI / 180.0));
  double xOut = x * cosA - y * sinA;
  double yOut = x * sinA + y * cosA;
  x = xOut;
  y = yOut;
}

/**
   Normalize wheel speed values.
*/
void Normalize(double *wheelSpeeds) {
  double maxMagnitude = fabs(wheelSpeeds[0]);
  int32_t i;
  for (i = 1; i < 4; i++) {
    double temp = fabs(wheelSpeeds[i]);
    if (maxMagnitude < temp)
      maxMagnitude = temp;
  }
  if (maxMagnitude > 1.0) {
    for (i = 0; i < 4; i++) {
      wheelSpeeds[i] = wheelSpeeds[i] / maxMagnitude;
    }
  }
}

/**
  Limit motor values to the -1.0 to +1.0 range.
*/
double Limit(double num) {
  if (num > 1.0) {
    return 1.0;
  }
  if (num < -1.0) {
    return -1.0;
  }
  return num;
}

/**
   Field-centric Mecanum drive.
*/
void MecanumDriveCartesian(float x, float y, float rotation, float gyroAngle) {
  double xIn = x;
  double yIn = y;

  // Negate y for the joystick.
  // yIn = -yIn;

  // Compensate for gyro angle.
  RotateVector(xIn, yIn, gyroAngle);

  double wheelSpeeds[4];
  wheelSpeeds[0] = yIn + xIn + rotation; // Left Front
  wheelSpeeds[1] = yIn - xIn + rotation; // Left Rear
  wheelSpeeds[2] = yIn - xIn - rotation; // Right Front
  wheelSpeeds[3] = yIn + xIn - rotation; // Right Rear

  Normalize(wheelSpeeds);

  double f = 1;
  if (abs(wheelSpeeds[0]) > f)
    f = wheelSpeeds[0];
  if (abs(wheelSpeeds[1]) > f)
    f = wheelSpeeds[1];
  if (abs(wheelSpeeds[2]) > f)
    f = wheelSpeeds[2];
  if (abs(wheelSpeeds[3]) > f)
    f = wheelSpeeds[3];

  motorLF->setSpeed(abs(wheelSpeeds[0]) * maxPower / f); // Left Front
  motorLR->setSpeed(abs(wheelSpeeds[1]) * maxPower / f); // Left Rear
  motorRF->setSpeed(abs(wheelSpeeds[2]) * maxPower / f); // Right Front
  motorRR->setSpeed(abs(wheelSpeeds[3]) * maxPower / f); // Right Rear

  //  DEBUG_PRINT("LF: ");
  //  DEBUG_PRINTLN(wheelSpeeds[0] * maxPower / f);
  //  DEBUG_PRINT("LR: ");
  //  DEBUG_PRINTLN(wheelSpeeds[1] * maxPower / f);
  //  DEBUG_PRINT("RF: ");
  //  DEBUG_PRINTLN(wheelSpeeds[2] * maxPower / f);
  //  DEBUG_PRINT("RR: ");
  //  DEBUG_PRINTLN(wheelSpeeds[3] * maxPower / f);

  //  Serial.print("LF: ");
  //  Serial.println(wheelSpeeds[0] * maxPower / f);
  //  Serial.print("LR: ");
  //  Serial.println(wheelSpeeds[1] * maxPower / f);
  //  Serial.print("RF: ");
  //  Serial.println(wheelSpeeds[2] * maxPower / f);
  //  Serial.print("RR: ");
  //  Serial.println(wheelSpeeds[3] * maxPower / f);

  // DEBUG_PRINT("Gyro Angle: ");
  // DEBUG_PRINTLN(gyroAngle);

  //  Serial.print("Gyro Angle: ");
  //  Serial.println(gyroAngle);

  if (wheelSpeeds[0] < 0) {
    motorLF->run(BACKWARD);
    DEBUG_PRINTLN("LF BACKWARD");
  }
  else {
    motorLF->run(FORWARD);
    DEBUG_PRINTLN("LF FORWARD");
  }
  if (wheelSpeeds[1] < 0) {
    motorLR->run(BACKWARD);
    DEBUG_PRINTLN("LR BACKWARD");
  }
  else {
    motorLR->run(FORWARD);
    DEBUG_PRINTLN("LR FORWARD");
  }
  if (wheelSpeeds[2] < 0) {
    motorRF->run(BACKWARD);
    DEBUG_PRINTLN("RF BACKWARD");
  }
  else {
    motorRF->run(FORWARD);
    DEBUG_PRINTLN("RF FORWARD");
  }
  if (wheelSpeeds[3] < 0) {
    motorRR->run(BACKWARD);
    DEBUG_PRINTLN("RR BACKWARD");
  }
  else {
    motorRR->run(FORWARD);
    DEBUG_PRINTLN("RR FORWARD");
  }
  DEBUG_PRINT(" LF: ");
  DEBUG_PRINTLN(wheelSpeeds[0]);
  DEBUG_PRINT(" LR: ");
  DEBUG_PRINTLN(wheelSpeeds[1]);
  DEBUG_PRINT(" RF: ");
  DEBUG_PRINTLN(wheelSpeeds[2]);
  DEBUG_PRINT(" RR: ");
  DEBUG_PRINTLN(wheelSpeeds[3]);
}

