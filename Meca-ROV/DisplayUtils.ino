#include <Arduino.h>

/**
  Name: DisplayUtils.ino

  Purpose: Meca-ROV utilities to support Grove Seeed RGB LCD control.
  @version: 0.1
*/

void outputTitle() {
  Serial.println("   _____                                __________ ____________   ____");
  Serial.println("  /     \\   ____   ____ _____           \\______   \\\\_____  \\   \\ /   /");
  Serial.println(" /  \\ /  \\_/ __ \\_/ ___\\\\__  \\    ______ |       _/ /   |   \\   |   / ");
  Serial.println("/    |    \\  ___/\\  \\___ / __ \\_ /_____/ |    |   \\/    |    \\     /  ");
  Serial.println("\\____|____/\\_____\\\\______\\_____/         |____|___/\\_________/\\___/   ");
}

/**
  Refresh LCD display.
*/
void updateIMUDisplay() {
  DEBUG_PRINT("Orientation: ");
  DEBUG_PRINT(hpr[0]);
  DEBUG_PRINT(" ");
  DEBUG_PRINT(hpr[1]);
  DEBUG_PRINT(" ");
  DEBUG_PRINTLN(hpr[2]);
#ifdef LCD
  lcd.clear();
  lcd.print("h:");
  lcd.print(hpr[0]);
  lcd.print((char)223);
  lcd.setCursor(0, 1);
  lcd.print("p:");
  lcd.print(hpr[1]);
  lcd.setCursor(9, 1);
  lcd.print("r:");
  lcd.print(hpr[2]);
#endif
}

#ifdef LCD
void sendToLCDDisplay(const String& outputMsg)
{
  // Print a message to the Grove RGB LCD.
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(outputMsg);
}

/**
  Fade in and out the Grove LCD display using a particular color.
*/
void breatheDisplay(unsigned char color)
{
  for (int i = 0; i < 255; i++)
  {
    lcd.setPWM(color, i);
    wait(1);
  }
  wait(30);
  for (int i = 255; i >= 0; i--)
  {
    lcd.setPWM(color, i);
    wait(3);
  }
  wait(10);
}
#endif

