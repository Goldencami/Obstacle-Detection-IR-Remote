#include <LiquidCrystal.h>
#include <IRremote.h>
#include <EEPROM.h>
#define yellow_LED 12
#define red_LED 11
#define green_LED 10
#define BTN_PIN 2
#define PHOTORESISTOR_PIN A0

#define ECHO_PIN 3
#define TRIGGER_PIN 4
#define IR_RECEIVE_PIN 5

#define LCD_RS_PIN A5
#define LCD_E_PIN A4
#define LCD_D4_PIN 6
#define LCD_D5_PIN 7
#define LCD_D6_PIN 8
#define LCD_D7_PIN 9

#define MEASUREMENT_TYPE_ADDRESS 0

bool isCM = true;
bool isLocked = false;
double distance = 0;
int currentScreen = 0; // 0, 1, 2

LiquidCrystal lcd(
  LCD_RS_PIN,
  LCD_E_PIN,
  LCD_D4_PIN,
  LCD_D5_PIN,
  LCD_D6_PIN,
  LCD_D7_PIN
);

byte redLEDState = LOW;
unsigned long lastBlinkRedLEDdelay = millis();
unsigned long blinkRedLEDdelay = 500;

unsigned long lastLuminosityDelay = millis();
unsigned long luminosityDelay = 100;

byte LEDsState = LOW;
unsigned long lastLockedLEDdelay = millis();
unsigned long lockedLEDdelay = 300;
 
byte btnState = LOW;
unsigned int debounceDelay = 50;
unsigned long lastBtnDebounce = millis();

unsigned long lastIRCommandTime = 0;
unsigned long irDebounceDelay = 300; // 300 ms debounce for IR commands

unsigned long lastUltrasonicTrigger = millis();
unsigned long ultrasonicTriggerDelay = 60;

volatile unsigned long pulseInTimeBegin;
volatile unsigned long pulseInTimeEnd;
volatile bool newDistanceAvailable = false;

double getUltrasonicDistance() {
  double durationMicros = pulseInTimeEnd - pulseInTimeBegin; 
  return durationMicros/58.0; //cm
}

void triggerUltrasonicSensor() {
  digitalWrite(TRIGGER_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN, LOW);
}

void echoPinInterrupt() {
  if(digitalRead(ECHO_PIN) == HIGH) { // start measuring
    // when signal is rising
    pulseInTimeBegin = micros();
  }
  else {
    // when signal is falling
    pulseInTimeEnd = micros();
    newDistanceAvailable = true;
  }
}

void displayScreen() {
  if(currentScreen == 0) {
    displayDistance();
  }
  else if(currentScreen == 1) {
    displayLuminosity();
  }
  else if(currentScreen == 2) {
    displayReset();
  }
}

void displayDistance() {
  eraseLCDline();
  lcd.setCursor(0, 0);
  lcd.print("Dist: ");
  String unit = isCM ? "cm" : "in";

  if(isCM) {
    lcd.print(distance);
  }
  else {
    lcd.print(distance*0.393701);
  }

  lcd.print(" ");
  lcd.print(unit);

  if(distance >= 10 && !isLocked) {
    eraseLCDline();
    lcd.setCursor(0, 1);

    if(distance > 50) {
      lcd.print("No obstacle.");
    }
    else {
      lcd.print("!! Warning !!");
    }
  }
}

void detectDistance() {
  if(distance >= 10 && !isLocked) {
    blinkRedLED();
  }
  else if(distance < 10) {
    isLocked = true;
  }
}

void blinkRedLED() {
  // rate between 0 and 1600ms for 0-400cm range
  unsigned long timeNow = millis();
  blinkRedLEDdelay = distance * 4;

  if(timeNow - lastBlinkRedLEDdelay > blinkRedLEDdelay) {
    if(redLEDState == LOW) {
      redLEDState = HIGH;
    }
    else {
      redLEDState = LOW;
    }

    digitalWrite(red_LED, redLEDState);
    lastBlinkRedLEDdelay = timeNow;
  }
}

void lockScreen() {
    eraseLCDline();
    lcd.setCursor(0, 0);
    lcd.print("!!! Obstacle !!!");

    eraseLCDline();
    lcd.setCursor(0, 1);
    lcd.print("Press to unlock.");

    blinkLockedLEDs();
}

void blinkLockedLEDs() {
  unsigned long timeNow = millis();

  if(timeNow - lastLockedLEDdelay > lockedLEDdelay) {
    if(LEDsState == LOW) {
      LEDsState = HIGH;
    }
    else {
      LEDsState = LOW;
    }

    digitalWrite(yellow_LED, LEDsState);
    digitalWrite(red_LED, LEDsState);
    digitalWrite(green_LED, LEDsState);

    lastLockedLEDdelay = timeNow;
  }
}

void displayLuminosity() {
  lcd.setCursor(0, 0);
  eraseLCDline();
  lcd.setCursor(0, 0);

  lcd.print("Luminosity: ");
  lcd.print(analogRead(PHOTORESISTOR_PIN));

  lcd.setCursor(0, 1);
  eraseLCDline();
}

void nightRoomLED() {
  unsigned long timeNow = millis();
  if(timeNow - lastLuminosityDelay > luminosityDelay) {
    //0 to 1023
    long intensity = 255 - (analogRead(PHOTORESISTOR_PIN)/4);
    analogWrite(green_LED, intensity);

    lastLuminosityDelay = timeNow;
  }
}

void displayReset() {
  eraseLCDline();
  lcd.setCursor(0, 0);
  lcd.print("Press on OFF to");

  eraseLCDline();
  lcd.setCursor(0, 1);
  lcd.print("reset settings.");
}

void unlockSystem() {
  unsigned long timeNow = millis();

  if(timeNow - lastBtnDebounce > debounceDelay) {
    byte newBtnState = digitalRead(BTN_PIN);

    if(newBtnState != btnState) {
      btnState = newBtnState;
      lastBtnDebounce = timeNow;

      if(btnState == HIGH) {
        Serial.println("Button pressed");
      }
      else {
        Serial.println("Button released");
        isLocked = false;
        digitalWrite(yellow_LED, LOW);
        digitalWrite(red_LED, LOW);
        digitalWrite(green_LED, LOW);
      }
    }
  }
}

void remoteCommand(long code) {
  switch(code) {
    case 64: //play
      isLocked = false;
      digitalWrite(yellow_LED, LOW);
      digitalWrite(red_LED, LOW);
      digitalWrite(green_LED, LOW);
      break;
    case 7: //toggle down
      if(currentScreen > 0) {
        --currentScreen;
      }
      break;
    case 9: //toggle up
      if(currentScreen < 2) {
        ++currentScreen;
      }
      break;
    case 25: //EQ
      isCM = !isCM;
      saveMeasurementType();
      break;
    case 69: // power off
      isCM = true;
      saveMeasurementType();
      break;
  }
}

void eraseLCDline() {
  for(int i=0; i < 16; i++) {
    lcd.print(" ");
  }
}

void saveMeasurementType() {
  String current = getMeasurementType();
  String unit = isCM ? "cm" : "in"; 

  if (current != unit) {
    for (int i = 0; i < unit.length(); i++) {
      EEPROM.write(MEASUREMENT_TYPE_ADDRESS + 1 + i, unit[i]);
    }
    EEPROM.write(MEASUREMENT_TYPE_ADDRESS + 1 + unit.length(), '\0');
  }
}

void loadMeasurementType() {
  String unit = getMeasurementType();
  if (unit == "in") {
    isCM = false;
  } else {
    isCM = true; // default to cm
  }
}

String getMeasurementType() {
  char c;
  String result = "";
  int addr = MEASUREMENT_TYPE_ADDRESS + 1;

  while ((c = EEPROM.read(addr++)) != '\0') {
    result += c;
  }

  return result;
}

void setup() {
  Serial.begin(115200);
  Serial.println("Starting Arduino!");

  pinMode(yellow_LED, OUTPUT);
  pinMode(red_LED, OUTPUT);
  pinMode(green_LED, OUTPUT);

  pinMode(ECHO_PIN, INPUT);
  pinMode(TRIGGER_PIN, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(ECHO_PIN), echoPinInterrupt, CHANGE);

  IrReceiver.begin(IR_RECEIVE_PIN);

  loadMeasurementType();
  lcd.begin(16, 2); // (columns, rows)
}

void loop() {
  unsigned long timeNow = millis();

  if (IrReceiver.decode()) {
    int code = IrReceiver.decodedIRData.command;
    if (timeNow - lastIRCommandTime > irDebounceDelay) {
      Serial.println(code);
      remoteCommand(code);
      lastIRCommandTime = timeNow;
    }

    IrReceiver.resume();  // Must call this to get the next signal
  }
  
  if(timeNow - lastUltrasonicTrigger > ultrasonicTriggerDelay) {
    lastUltrasonicTrigger += ultrasonicTriggerDelay;
    triggerUltrasonicSensor();
  }

  if(newDistanceAvailable) {
    newDistanceAvailable = false;
    distance = getUltrasonicDistance();
  }

  if(isLocked) {
    lockScreen();
    unlockSystem();
  }
  else {
    detectDistance();
    displayScreen();
    nightRoomLED();
  }
}