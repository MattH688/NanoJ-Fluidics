/*

  This is the firmware for an Arduino Uno to drive DC motor pumps through
  the Adafruit Motor Shield v2, while under control of our MicroManager
  plugins.

  © Pedro Almada, UCL, 2015
  © Matthew Hockely, UoK, 2020

  Commands:
  g = get status of all pumps
  a = stop all pumps
  p = get number of pumps
  axy = stop pump xy
  sxynnn = for pump xy set speed nnn
  rxydttttt - Start pump xy in direction d for ttttt seconds

  The commands use this notation:
  d = 1 is forward, d = 2 is backwards
  x = shield address {1-nShields}
  y = motor/pump address {1-nMotorsPerShield}
  nnn = speed {000-255}
  ttttt = duration {00001-99999}

  Our testing with 1 ml syringe tells us the speed will correspond to:
  255 - 2.3 microliters/second

*/

//// Imports
#include <EEPROM.h>
// Adafruit Motor Shield libraries
#include <Wire.h>
#include <Adafruit_MotorShield.h>
// Numpad and LCD
#include <Keypad.h>
#include <LiquidCrystal_I2C.h>

//// Settings

// Setup for LCD screen
//I2C pins declaration
LiquidCrystal_I2C lcd(0x27, 16, 2);

byte mlChar[] = {
  B01010,
  B10101,
  B10101,
  B00000,
  B01000,
  B01000,
  B01000,
  B00110
};

byte ulChar[] = {
  B10010,
  B10010,
  B01100,
  B00000,
  B01000,
  B01000,
  B01000,
  B00110
};

// Number of shields (MAX: 9)
const int nShields = 1;
// Number of DC motors per shield (MAX on Adafruit Motor Shield v2: 4)
const int nMotorsPerShield = 2;
// Character which signals end of message from PC
char endMessage = '.';
bool motorOn = 0; // By default, motors are off

/*4x4 Matrix Keypad connected to Arduino
  This code prints the key pressed on the keypad to the serial port*/

const byte numRows = 4; //number of rows on the keypad
const byte numCols = 4; //number of columns on the keypad

//keymap defines the key pressed according to the row and columns just as appears on the keypad
char keymap[numRows][numCols] =
{
  {'1', '2', '3', 'A'},
  {'4', '5', '6', 'B'},
  {'7', '8', '9', 'C'},
  {'*', '0', '#', 'D'}
};

//Code that shows the the keypad connections to the arduino terminals // Digital Pins
byte rowPins[numRows] = {9, 8, 7, 6}; //Rows 0 to 4
byte colPins[numCols] = {5, 4, 3, 2}; //Columns 0 to 4
Keypad myKeypad = Keypad(makeKeymap(keymap), rowPins, colPins, numRows, numCols);

// Misc. initializations
byte currentShield;
byte currentMotor;
byte currentSpeed;
int eeAddress = 0;
const int maxMessageLength = 9;
unsigned long startTime = 0; // in ms - 3000 = 3 seconds
unsigned long elapsedTime = 0;
unsigned long targetTime = 0;
unsigned long timeRemaining = 0;
boolean timeCounter = false;
Adafruit_MotorShield afms[nShields];
Adafruit_DCMotor *motor[nShields][nMotorsPerShield];

// The Arduino should keep track of what the motors have been told to do.
// We can address each motor's status by accessing the values on motorStatus
// Format is: [shield][motor]
// Speed can vary from 0 to 255
// States are: 0 - stop; 1 - forward; 2 - backward.
// motor is which current motor is decided based on flow rate only used in [1][1]

// Hard coded calibration at the bottom of the script
// Min Range - ul/sec
// Max Range - ul/sec
int motorStatus[nShields][nMotorsPerShield][1] = {{}}; // Rate of movement ([int rate][int currentMotorState])
int flowStatus[2] = {{}}; // [int flowUnit][int withdraw/infuse][int motor])
float floatVar[2] = {{}}; // [rate][dia][diaScale] -  Too large to convert to int
// floatVar[0] rate
// floatVar[1] diameter
// floatVar[2] diameter scale
int volFloat; // [volume] - Small enough to be int
bool InfWith = flowStatus[1]; // 0 is Inf; 1 is With

void setup() {
  // Start serial communication and set baud-rate
  Serial.begin(57600);
  Serial.println(F("Connected!"));

  // We now create a Motor Shield object for each shield. Their address
  // (96-128) is set by soldering a few jumpers.
  // see: https://learn.adafruit.com/adafruit-motor-shield-v2-for-arduino/stacking-shields
  // Note here I'm using decimal values for the address which is simpler
  // but their tutorial uses hex.

  unsigned char address = 96;

  // Create array of shield objects

  for (int i = 0; i < nShields; i++) {
    afms[i] = Adafruit_MotorShield(address);
    address = ++address;
    // Initialize shield objects with the default frequency 1.6KHz
    afms[i].begin();
  }

  // For each shield we then get each of the their motors.
  EEPROM.get(eeAddress, motorStatus);
  for (int i = 0; i < nShields; i++) {
    for (int j = 0; j < nMotorsPerShield; j++) {
      motor[i][j] = afms[i].getMotor(j + 1);
      motor[i][j]->setSpeed(motorStatus[i][j][0]);
    }
  }

  EEPROM.get(1, floatVar);
  if (floatVar[0] != NULL) { // Setup speed variable if no memory
    // Remember these will be divided by 100 to add decimal point
    floatVar[0] = 37.00; // ul or ml or pct value
    floatVar[1] = 14.50; // mm diameter of syringe
    floatVar[2] = 01.00; // 1x scale factor for 14.5mm diameter
    EEPROM.put(1, floatVar);
  }

  // Setup LCD screen
  lcd.init();
  // lcd.begin(0x27, 16, 2); //Initializing display
  lcd.backlight();//To Power ON the back light
  //lcd.backlight();// To Power OFF the back light
  lcd.createChar(0, mlChar);
  lcd.createChar(1, ulChar);

  EEPROM.get(2, flowStatus);
  if (flowStatus[0] != NULL) { // Setup speed variable if no memory
    //syringeInts[0] = 100; // Stored as pct '%'
    flowStatus[0] = 0; // Flow units - %
    flowStatus[1] = 0; // Withdraw/Infuse
    flowStatus[2] = 0; // First motor
    EEPROM.put(2, flowStatus);
  }

  EEPROM.get(3, volFloat);
  if (volFloat != NULL) {
    volFloat = 0; 
    EEPROM.put(3,volFloat);
  }
  LCDhome();

  // Check EEPROM for values
  //EEPROM.get(3, flowStatus);
  //[Unit][With/Inf][Motor]
}

void loop() {
  char incoming[maxMessageLength];
  char keypressed = myKeypad.getKey(); // Scan the numpad for input

  // If a pump has been started, we stop all pumps after a targetTime amount of time
  if (timeCounter == true) {
    elapsedTime = millis() - startTime;
    timeRemaining = targetTime - elapsedTime;

    lcd.clear();
    lcd.setCursor(1, 0); //Defining positon to write from first row,first column .
    lcd.print(F("Time remaining"));  //You can write 16 Characters per line .
    lcd.setCursor(0, 1); //Defining positon to write from second row,first column .
    lcd.print(timeRemaining);
    if (elapsedTime >= targetTime) {
      for (int i = 0; i < nShields; i++) {
        for (int j = 0; j < nMotorsPerShield; j++) {
          motorStatus[i][j][1] = 0; // Set the pump to stop
          EEPROM.put(eeAddress, motorStatus);
          motor[i][j]->run(RELEASE);
        }
      }
      elapsedTime = 0;
      targetTime = 0;
      timeCounter = false;
    }
  }

  /* else { // Use variable for selected pump to start
    if (motorStatus[currentShield][currentMotor][1] == 1) {
      motor[currentShield][currentMotor]->run(FORWARD);
      startTime = millis();
      elapsedTime = 0;
      timeCounter = true;
    } else {
      motor[currentShield][currentMotor]->run(FORWARD);
      startTime = millis();
      elapsedTime = 0;
      timeCounter = true;
    }
    }
  */
  // Press select button (A) to enter menu

  if (keypressed == 'A') {
    MenuUI();
  }
  /*
    // If there is a message from the serial port or start/stop is pressed
    if (Serial.available() > 0 || (keypressed == 'D')) {

      // Read buffer until end character is reached or maxMessageLength characters are read.
      int lengthOfMessage = Serial.readBytesUntil(endMessage, incoming, maxMessageLength);

      if (keypressed == 'D' && motorOn == 0) {
        // Make "fake" incoming message
        // rxydttttt - Start pump xy in direction d for ttttt seconds
        // d = 1 is forward, d = 2 is backwards
        incoming[0] = 'r';
        incoming[1] = flowStatus[2]; // Which pump (1 or 2)
        incoming[2] = flowStatus[1]; // Direction
        // In theory we have already fed time in via targetTime from the "void setVolume" function
      }

      // The incoming message will be char array and we're using element 0
      // to determine what is the action to be performed by the pump.
      // The next characters will determine optional parameters such as
      // what pump to start, what speed to set, etc...

      // If incoming message is:
      // g = get status of all pumps
      if (incoming[0] == 'g') {
        for (int i = 0; i < nShields; i++) {
          for (int j = 0; j < nMotorsPerShield; j++) {
            Serial.print("S");
            Serial.print(i + 1);
            Serial.print("M");
            Serial.print(j + 1);
            Serial.print(":");
            Serial.print(motorStatus[i][j][0]); // Current Speed
            Serial.print(",");
            Serial.print(motorStatus[i][j][1]); // Current State (0 = Still, 1 = infusin, 2 = withdraw)
            Serial.print(";");
          }
        }
        Serial.println();
      }

      // a = stop all pumps
      else if (incoming[0] == 'a' && lengthOfMessage == 1  || (keypressed == 'D' && motorOn == 1)) {
        lcd.clear();
        lcd.setCursor(0, 0); //Defining positon to write from first row,first column .
        //lcd.println("Pumps Stopped");
        for (int i = 0; i < nShields; i++) {
          for (int j = 0; j < nMotorsPerShield; j++) {
            motorStatus[i][j][0] = 255;
            motorStatus[i][j][1] = 0;
            EEPROM.put(eeAddress, motorStatus);
            motor[i][j]->run(RELEASE);
          }
        }
        elapsedTime = 0;
        timeCounter = false;
        Serial.println("Stopped all pumps!");
        //Serial.println();

      }

      // axy = stop pump xy
      else if (incoming[0] == 'a' && lengthOfMessage != 1) {
        lcd.clear();
        lcd.setCursor(0, 0); //Defining positon to write from first row,first column .
        lcd.println("Switching Pumps");
        currentShield = incoming[1] - '0' - 1;
        currentMotor  = incoming[2] - '0' - 1;
        motorStatus[currentShield][currentMotor][1] = 0;
        EEPROM.put(eeAddress, motorStatus);
        motor[currentShield][currentMotor]->run(RELEASE);

        elapsedTime = 0;
        timeCounter = false;
        Serial.print("Stopped pump: ");
        Serial.print(incoming[1]);
        Serial.print(",");
        Serial.print(incoming[2]);
        Serial.println("!");
      }

      //p = get number of pumps
      else if (incoming[0] == 'p') {
        String s = String(nShields, DEC);
        String m = String(nMotorsPerShield, DEC);
        String result = String(s + "." + m);
        Serial.println(result);
      }

      // sxynnn = for pump xy set speed nnn
      else if (incoming[0] == 's') {
        currentShield = incoming[1] - '0' - 1;
        currentMotor  = incoming[2] - '0' - 1;
        byte a = incoming[3] - '0';
        byte b = incoming[4] - '0';
        byte c = incoming[5] - '0';
        currentSpeed = (a * 100) + (b * 10) + c;

        motorStatus[currentShield][currentMotor][0] = currentSpeed;
        EEPROM.put(eeAddress, motorStatus);
        motor[currentShield][currentMotor]->setSpeed(currentSpeed);

        Serial.print("Set speed of pump: ");
        Serial.print(incoming[1]);
        Serial.print(",");
        Serial.print(incoming[2]);
        Serial.print(" to ");
        Serial.println(currentSpeed);

        lcd.clear();
        lcd.setCursor(0, 0); //Defining positon to write from first row,first column .
        lcd.print("     Rate:     ");
        lcd.setCursor(0, 1); //row,col
        lcd.print(currentSpeed);
        lcd.setCursor(6, 1);
        lcd.write(1);
        lcd.print("/min");
        // Debugging via serial commands if no number pad available using LCD screen
      } else if (incoming[0] == '1' || incoming[0] == '2' || incoming[0] == '3' || incoming[0] == '4' || incoming[0] == '5' || incoming[0] == '6' || incoming[0] == '7' || incoming[0] == '8' || incoming[0] == '9' || incoming[0] == '*' || incoming[0] == 'A' || incoming[0] == 'B' || incoming[0] == 'C' || incoming[0] == 'D') {
        keypressed = incoming[0];
      }

      // rxydttttt - Start pump xy in direction d for ttttt seconds
      // d = 1 is forward, d = 2 is backwards
      else if (incoming[0] == 'r' && lengthOfMessage == maxMessageLength) {
        char targetTimeInput[5] = {incoming[4], incoming[5], incoming[6], incoming[7], incoming[8]};
        targetTime = atol(targetTimeInput) * 1000;
        currentShield = incoming[1] - '0' - 1;
        currentMotor  = incoming[2] - '0' - 1;
        motorStatus[currentShield][currentMotor][1] = incoming[3] - '0';
        EEPROM.put(eeAddress, motorStatus);

        if (incoming[3] == '1') {
          motor[currentShield][currentMotor]->run(FORWARD);
          startTime = millis();
          elapsedTime = 0;
          timeCounter = true;
          Serial.print("Started pump: ");
          Serial.print(incoming[1]);
          Serial.print(",");
          Serial.print(incoming[2]);
          Serial.println(" in the forward direction.");

          //lcd.clear();
          //lcd.setCursor(0, 0); //Defining positon to write from first row,first column .
          lcd.print("Infusing pump ");
          //        lcd.print(incoming[1]);
          //        lcd.print(",");
          //        lcd.print(incoming[2]);
          //        lcd.setCursor(0, 1); //row,col
          //        lcd.write(currentSpeed);
          //        lcd.setCursor(6, 1);
          //        lcd.print("ul/min");
        }
        else if (incoming[3] == '2') {
          startTime = millis();
          elapsedTime = 0;
          timeCounter = true;
          motor[currentShield][currentMotor]->run(BACKWARD);
          Serial.print("Started pump: ");
          Serial.print(incoming[1]);
          Serial.print(",");
          Serial.print(incoming[2]);
          Serial.println(" in the backward direction.");

          lcd.print("Withdrawing pump");
        }

        lcd.clear();
        lcd.setCursor(0, 0); //Defining positon to write from first row,first column .
        lcd.print(incoming[1]);
        lcd.print(",");
        lcd.print(incoming[2]);
        lcd.setCursor(0, 1); //row,col
        lcd.write(currentSpeed);
        lcd.setCursor(6, 1);
        lcd.write(1);
        lcd.print("/min");
      }

      else {
        Serial.println();
      }
    }*/
}

void LCDhome() {
  //Serial.println(F("1"));
  //EEPROM.get(1, floatVar);
  // LCD Home display
  float tempVol;
  lcd.clear();
  lcd.print(F("V:"));
  Serial.println(F("Volume:"));
  Serial.println(volFloat);
  tempVol = (float) volFloat / 100;
  Serial.print("tempVol ");
  Serial.println(tempVol);
  if (tempVol < 0.01) {
    lcd.print(F("Inf"));
  } else {
    lcd.print(tempVol, 1);
  }
  lcd.setCursor(5, 0);
  lcd.write(0);
  lcd.setCursor(7, 0);
  lcd.print(F("R:"));
  lcd.print(floatVar[0], 2);
  /*char ratePrint[6];
    dtostrf(floatVar[0], 3, 2, ratePrint);
    char * p;
    p = strstr (ratePrint, ".00");
    if (p) {
    int decimalPos = p - ratePrint;
    ratePrint[decimalPos] = NULL;
    ratePrint[decimalPos + 1] = NULL;
    ratePrint[decimalPos + 2] = NULL;
    }
    //Serial.println(ratePrint);
    lcd.print(ratePrint);*/

  lcd.setCursor(12, 0);
  if (flowStatus[0] == 0) { //pct

    lcd.print(F("%Pct"));

  } else if (flowStatus[0] == 1) { //ul/sec
    lcd.write(1);
    lcd.print(F("/sec"));

  } else if (flowStatus[0] == 2) { //ul/min
    lcd.write(1);
    lcd.print(F("/min"));

  } else {
    lcd.write(0);
    lcd.print(F("/min"));
  }

  lcd.setCursor(0, 1);
  lcd.print(F("Pump:"));
  if (flowStatus[1] == 1) {
    lcd.print(F("Infuse"));
    lcd.write(126);
  } else if (flowStatus[1] == 2) {
    lcd.print(F("Withdraw"));
    lcd.write(127);
  }  else {
    lcd.write(88);
    if (InfWith == 0) {
      lcd.setCursor(8, 1);
      lcd.print(F("Infuse"));
      lcd.setCursor(15, 1);
      lcd.write(126);
    } else { // if (InfWith == 0) {
      lcd.setCursor(7, 1);
      lcd.print(F("Withdraw"));
      lcd.write(127);
    }
  }
}

void MenuUI() {
  bool Menu = 0;
  int MenuChoice = 0;
  char keypressed = myKeypad.getKey(); // Scan the numpad for input

  lcd.clear();

  // B and C to move up and down menu; A to exit
  while (Menu != 1) {
    if (keypressed == 'B') {
      lcd.clear();
      MenuChoice = MenuChoice + 1;
      if (MenuChoice > 3) {
        MenuChoice = 0;
      }
    } else if (keypressed == 'C') {
      lcd.clear();
      MenuChoice = MenuChoice - 1;
      if (MenuChoice < 0) {
        MenuChoice = 3;
      }
    }

    switch (MenuChoice) {
      case 0:
        lcd.setCursor(3, 0);
        lcd.print(F("Set Rate"));
        if (keypressed == 'A') {
          SetRate();
          Menu = 1;
        }
        break;
      case 1:
        lcd.setCursor(1, 0);
        lcd.print(F("Set Diameter"));
        if (keypressed == 'A') {
          SetDiameter();
          Menu = 1;
        }
        break;
      case 2:
        lcd.setCursor(1, 0);
        lcd.print(F("Set Inf/With"));
        if (keypressed == 'A') {
          SetInfWith();
          Menu = 1;
        }
        break;
      case 3:
        lcd.setCursor(2, 0);
        lcd.print(F("Set Volume"));
        if (keypressed == 'A') {
          SetVolume();
          Menu = 1;
        }
        break;
    }
    if (keypressed == '#' || keypressed == 'D') {
      Menu = 1;
    }
    keypressed = myKeypad.getKey();
  }
  LCDhome(); // Reset screen to default
}

void SetRate() {
  int cursorPos = 0;
  //int cursorPos1 = 0;
  bool Menu = 0;
  char keypressed = myKeypad.getKey(); // Scan the numpad for input
  int motorSel = flowStatus[2]; // Find current motor selceted
  int motorVal = motorStatus[0][motorSel][0]; // motorShield/motorSelection - motorValue
  int rateInt = 0;
  int *motorStruct;
  int gearRatio = 0;
  int *structLCDVal;
  char stringLCDVal[] = {};
  int unitVal = flowStatus[0];
  int decimalPos = 0;
  int rateValArray[3];

  // Convert float to char string
  // - Easier for LCD print and changing individual values
  dtostrf(floatVar[0], 2, 2, stringLCDVal);



  // Header on
  lcd.setCursor(3, 0);
  lcd.print(F("Set Rate"));

  // Important for printing the correct units
  keypressed = 'A';
  unitVal = unitVal - 1;
  lcd.setCursor(0, 1);
  lcd.print(stringLCDVal);

  // Shows cursor
  lcd.cursor();

  // A and # changes units; B and C moves cursor; Numbers for value; * for decimal point; D exit to main menu
  while (Menu != 1) {

    // Set rate either as percentage or flow rate
    if (keypressed == 'A' || keypressed == '#') {
      unitVal = unitVal + 1;
      if (unitVal > 3) {
        unitVal = 0;
      }
      //Serial.print(unitVal);
      // Generic LCD setup to insert flow rate
      //lcd.noCursor();
      lcd.setCursor(10, 1);
      switch (unitVal) {
        case 0:
          lcd.print(F("% perc"));
          break;
        case 1:
          lcd.write(1);
          lcd.print(F("/sec"));
          break;
        case 2:
          lcd.write(1);
          lcd.print(F("/min"));
          break;
        case 3:
          lcd.write(0);
          lcd.print(F("/min"));
          break;
      }
      lcd.setCursor(cursorPos, 1);
    } else if (keypressed == 'D') { // Exit menu
      Menu = 1;
    } else if (isDigit(keypressed) || keypressed == '*' || keypressed == 'C' || keypressed == 'B') {

      // 
      LCDinput(keypressed, &decimalPos, stringLCDVal, &rateInt, &cursorPos,3); // input numbers
      
      floatVar[0] = atof(stringLCDVal); // Convert string to float

//      if ((unitVal == 0) && (floatVar[0] > 100)) { // if pct(%) and rateInt is max (99%)
//        rateValArray[0] = 1; rateValArray[1] = 0; rateValArray[2] = 0;
//      }
    }
    // check for key presses
    keypressed = myKeypad.getKey();
  }
  lcd.setCursor(cursorPos, 1);
  lcd.noCursor();

  // Requires conversion to pump power 0-255
  if (unitVal  == 0) { // if percentage

    if (floatVar[0] > 100) {
      floatVar[0] = 100;
    }

    motorVal = motorVal * 2.55;

  } else { // if ul/sec
    //if rateInt is between 2ul/sec to 5ul/sec or ml/sec, etc
    //set LCD screen to show which gear layout is required - Also choose motor to drive
    motorStruct = Units2Pct(unitVal, rateValArray, rateInt, decimalPos);
    motorVal = motorStruct[1]; gearRatio = motorStruct[2];
  }

  // Save output to EEPROM
  motorStatus[1][motorSel][0] = motorVal; // Format is: [shield][motor] -- [speed][state][rate] int
  flowStatus[0] = unitVal; // Stored unit

  // Calculate target time if volume in place
  setTargetTime();

  lcd.clear();
  EEPROM.put(0, motorStatus);
  EEPROM.put(1, floatVar);
  EEPROM.put(2, flowStatus);
}

// Simple conversion from units to percentage
int Units2Pct(int unitVal, int rateVal, int rateInt, int decimalPos) {
  float tempVal = floatVar[0];
  int motorVal = 0;
  int gearRatio = 0;
  
  if (unitVal == 1) { // ul/min

    tempVal = tempVal * 0.06;

  } else if (unitVal == 2) { // ul/sec

    tempVal = tempVal * 0.001;

  } else { // ml/min
    tempVal = tempVal;
  }

  // 10 different gear combinations in this version
  // Below is an optimised table which will attempt to use the gears based on their optimal power usage
  // floatVar is DiaScaling factor

  if ((37 * floatVar[2]) > tempVal > (16 * floatVar[2])) {//(16 * floatVar[2] < tempVal < 37 * floatVar[2]) {
    gearRatio = 1;
    motorVal = ((tempVal - (9.2849 * floatVar[2])) / (27.855 * floatVar[2])) * 100;
  } else if ((16 * floatVar[2]) > tempVal > (9 * floatVar[2])) {
    gearRatio = 2;
    motorVal = ((tempVal - (4.1078 * floatVar[2])) / (12.323 * floatVar[2])) * 100;
  } else if ((9 * floatVar[2]) > tempVal > (3 * floatVar[2])) {
    gearRatio = 3;
    motorVal = ((tempVal - (2.4206 * floatVar[2])) / (7.2617 * floatVar[2])) * 100;
  } else if ((3 * floatVar[2]) > tempVal > (2 * floatVar[2])) {
    gearRatio = 4;
    motorVal = ((tempVal - (0.8456 * floatVar[2])) / (2.5367 * floatVar[2])) * 100;
  } else if ((2 * floatVar[2]) > tempVal > (0.6 * floatVar[2])) {
    gearRatio = 5;
    motorVal = ((tempVal - (0.5366 * floatVar[2])) / (1.6099 * floatVar[2])) * 100;
  } else if ((0.6 * floatVar[2]) > tempVal > (0.3 * floatVar[2])) {
    gearRatio = 6;
    motorVal = ((tempVal - (0.1704 * floatVar[2])) / (0.5112 * floatVar[2])) * 100;
  } else if ((0.3 * floatVar[0]) > tempVal > (0.1 * floatVar[2])) {
    gearRatio = 7;
    motorVal = ((tempVal - (0.0956 * floatVar[2])) / (0.2867 * floatVar[2])) * 100;
  } else if ((0.1 * floatVar[0]) > tempVal > (0.08 * floatVar[2])) {
    gearRatio = 8;
    motorVal = (tempVal - (0.0331 * floatVar[2]) / (0.0992 * floatVar[2])) * 100;
  } else if ((0.08 * floatVar[0]) > tempVal > (0.025 * floatVar[2])) {
    gearRatio = 9;
    motorVal = ((tempVal - (0.021 * floatVar[2])) / (0.0629 * floatVar[2])) * 100;
  } else if ((0.025 * floatVar[0]) > tempVal > (0.006 * floatVar[2])) {
    gearRatio = 10;
    motorVal = (tempVal - (0.0066 * floatVar[2]) / (0.0199 * floatVar[2])) * 100;
  } else if  (tempVal > (37 * floatVar[2])) { //&& tempVal < 0.007) {
    gearRatio = -1;
    motorVal = 255; // Max speed
    float maxTemp = (27.855 * floatVar[2]);
    lcd.setCursor(0, 0);
    lcd.print("Max flow rate");
    lcd.setCursor(0, 1);
    delay(3000);

    if (unitVal == 1) { //ul/sec
      lcd.print(maxTemp * 600); // Max flow rate
      lcd.setCursor(10, 1);
      lcd.write(1);
      lcd.print(F("/sec"));
    } else if (unitVal == 2) {// ul/min
      lcd.print(maxTemp * 1000); // Max flow rate
      lcd.setCursor(10, 1);
      lcd.write(1);
      lcd.print(F("/min"));
    } else if (unitVal == 3) {
      lcd.print(maxTemp); // Max flow rate
      lcd.setCursor(10, 1);
      lcd.write(0);
      lcd.print(F("/min"));
    }
    floatVar[0] = maxTemp;
    delay(3000); // Wait 3 seconds
    lcd.clear();

    lcd.setCursor(1, 0);
    lcd.print(F("Use Gear Ratio"));
    lcd.setCursor(6, 1);
    lcd.print(F("10"));
  } else if (tempVal < 0.0066 * floatVar[2]) {
    gearRatio = -2;
    motorVal = 64; // 25% of motor power (as slow as possible)
    float minTemp = (0.0066 * floatVar[2]);
    lcd.setCursor(0, 0);
    lcd.print(F("Max flow rate"));
    lcd.setCursor(0, 1);
    delay(3000);

    if (unitVal == 1) { //ul/sec
      lcd.print(minTemp * 600); // Max flow rate
      lcd.setCursor(10, 1);
      lcd.write(1);
      lcd.print(F("/sec"));
    } else if (unitVal == 2) { // ul/min
      lcd.print(minTemp * 1000); // Max flow rate
      lcd.setCursor(10, 1);
      lcd.write(1);
      lcd.print(F("/min"));
    } else if (unitVal == 3) {
      lcd.print(minTemp); // Max flow rate
      lcd.setCursor(10, 1);
      lcd.write(0);
      lcd.print(F("/min"));
    }

    delay(3000); // Wait 3 seconds
    lcd.clear();

    lcd.setCursor(0, 0);
    lcd.print(F("Use Gear Ratio 1"));
  }
  // LCD Print gear ratio
  if (gearRatio != -1 && gearRatio != -2 && gearRatio < 10) {
    lcd.clear();
    lcd.setCursor(1, 0);

    lcd.print(F("Use Gear Ratio "));
    lcd.print(gearRatio);
  } else if (gearRatio != -1 && gearRatio != -2 && gearRatio > 9) {
    lcd.clear();
    lcd.setCursor(1, 0);

    lcd.print(F("Use Gear Ratio"));
    lcd.setCursor(6, 1);
    lcd.print(gearRatio);
  }

  delay(3000); // Wait 3 seconds
  lcd.clear();



  //  if (12 < tempVal < 30) {
  //    gearRatio = 1;
  //    motorVal = ((tempVal - 7.6756) / 23.026) * 100;
  //  } else if (7 < tempVal < 12) {
  //    gearRatio = 2;
  //    motorVal = ((tempVal - 3.0699) / 9.2097) * 100;
  //  } else if (3 < tempVal < 7) {
  //    gearRatio = 3;
  //    motorVal = ((tempVal - 2.6206) / 7.8618) * 100;
  //  } else if (1.5 < tempVal < 3) {
  //    gearRatio = 4;
  //    motorVal = ((tempVal - 0.8494) / 2.5483) * 100;
  //  } else if (0.5 < tempVal < 1.5) {
  //    gearRatio = 5;
  //    motorVal = ((tempVal - 0.417) / 1.2511) * 100;
  //  } else if (0.3 < tempVal < 0.5) {
  //    gearRatio = 6;
  //    motorVal = ((tempVal - 0.1415) / 0.4244) * 100;
  //  } else if (0.11 < tempVal < 0.3) {
  //    gearRatio = 7;
  //    motorVal = ((tempVal - 0.1067) / 0.3201) * 100;
  //  } else if (0.06 < tempVal < 0.11) {
  //    gearRatio = 8;
  //    motorVal = ((tempVal - 0.0303) / 0.0908) * 100;
  //  } else if (0.02 < tempVal < 0.06) {
  //    gearRatio = 9;
  //    motorVal = ((tempVal - 0.0171) / 0.0513) * 100;
  //  } else if (0.006 < tempVal < 0.02) {
  //    gearRatio = 10;
  //    motorVal = (tempVal - 0.0056 / 0.01675) * 100;
  //  } else { //  (tempVal > 37 && tempVal < 0.007) {
  //    gearRatio = 0; // Warning, outside gear ratio range
  //  }

  // Create structure of variables
  int structMotor[2] = {motorVal, gearRatio};

  return structMotor; // Return structure to loop()
}

// Purpose is to set diameter which changes usable rates
// V = pi*(r^2)*h - equation for the cylinder
void SetDiameter() {
  //char structLCDVal;
  char structLCDVal[] = {};
  int decimalPos = 0;
  char *DiaVal;
  int DiaInt;
  bool Menu = 0;
  int cursorPos = 0;
  int unitVal = -1;
  char keypressed = myKeypad.getKey(); // Scan the numpad for input
  //float *diaStore = motorStatus[1][1][6]; // storage array for EEPROM
  float diaScale = floatVar[2]; // Scaling factor to change flow rates available
  float tempVal = floatVar[1]; // temporary float for the diameter

  lcd.clear();
  keypressed = 'B'; // To print first menu option


  while (Menu != 1) {
    // Set rate either as percentage or flow rate
    if (keypressed == 'B' || keypressed == '#') {
      lcd.clear();
      unitVal = unitVal + 1;
      if (unitVal > 2) {
        unitVal = 0;
      }
      // Generic LCD setup to insert flow rate
      lcd.cursor();
      lcd.setCursor(0, 0);
      switch (unitVal) {
        case 0:
          lcd.setCursor(5, 0);
          lcd.print("Custom");
          lcd.setCursor(0, 1);


          lcd.print(DiaVal);
          break;
        case 1:
          lcd.setCursor(3, 0);
          lcd.print("BD 10");
          lcd.write(0);
          DiaVal = "14.5";
          lcd.setCursor(0, 1);
          lcd.print(DiaVal);
          break;
        case 2:
          lcd.setCursor(3, 0);
          lcd.print("BD 20");
          lcd.write(0);
          DiaVal = "19.13";
          lcd.setCursor(0, 1);
          lcd.print(DiaVal);
          break;
      }
      lcd.setCursor(13, 1);
      lcd.print("mm");
      lcd.setCursor(cursorPos, 1);
      lcd.cursor();
    } else if ((unitVal == 0) && (isDigit(keypressed) || keypressed == '*')) {
      LCDinput(keypressed, &decimalPos, structLCDVal, &DiaInt, &cursorPos, 2); // input numbers
      //char stringLCDVal[] = LCDinput(keypressed, &decimalPos, stringLCDVal, &rateInt, &cursorPos); // input numbers
      DiaVal = structLCDVal;
      lcd.setCursor(0, 1);
      lcd.print(DiaVal);
      
    } else if (keypressed == 'D') {
      Menu = 1;
    }
    /*else if (isDigit(keypressed) || keypressed == '*' || keypressed == 'C' || keypressed == 'B') {

      LCDinput(keypressed, &decimalPos, structLCDVal, &DiaInt, &cursorPos); // input numbers
      //char stringLCDVal[] = LCDinput(keypressed, &decimalPos, stringLCDVal, &rateInt, &cursorPos); // input numbers
      }*/
    keypressed = myKeypad.getKey();
  }
  // Convert int array to float (number with decimal places)
  //tempVal = Int2Float(DiaVal, DiaInt, decimalPos);
  tempVal = atof(DiaVal);

  // Convert the diameter to a scaling factor for calibration
  diaScale = DiaConv(DiaVal);

  floatVar[1] = tempVal; floatVar[2] = diaScale;
  // No need to store diameter units as it will always be "mm"
  lcd.clear();
  EEPROM.put(1, floatVar);
}

// Diameter conversion value
// Get value to convert flow rates if syringe not BD 10ml
int DiaConv(int r) {
  int h0;
  float diaScale;

  // These must be changed if using different calibration
  int h1 = 23.713; // mm - "h" value which is used to compare different syringe sizes - known calibration
  float v = 27.855; // ml/min yellow/red pump calibration

  // Rearrangement of cylinder volume
  h0 = (PI * (r ^ 2)) / v; // v = pi*(r^2)*h

  // Scaling factor for values
  diaScale = h0 / h1; // h1 is known calibration at the volume 27.855 for diameter 14.5 mm

  return diaScale;
}

//// Purpose is to save recoding the numpad LCD input for diameter, rate, etc.
//
// keypressed - Input character (Should only be numbers, '*', 'B' and 'C')
// *decimalPos - Where the current decimalPos is
// *printVal - Printed number in array form
// *printInt - Maximum number of digits on display
// *cursorPos - Used to change single number/shift decimal
// sigFig - How many significant figures before decimal point
//
char LCDinput(char keypressed, int *decimalPos, char *printVal, int *printInt, int *cursorPos, int sigFig) {
  lcd.setCursor(0, 1);

  Serial.println("Print int");
  Serial.println(*printInt);

  Serial.println("decimal pos ");
  Serial.println(*decimalPos);

  Serial.println("Sig fig ");
  Serial.println(sigFig);

  Serial.println("Cursor Pos ");
  Serial.println(*cursorPos);

  //Serial.println("Imported number");
  int arrLen = sizeof(printVal) / sizeof(printVal[0]);
  for (int i = 0; i < arrLen; i++) {
    lcd.print(printVal[i]);
  }

  lcd.setCursor(*cursorPos, 1);
  // Move cusor
  if (keypressed == 'B') { //&& (*cursorPos + 1 > arrLen)) {
    // +1 to cursor position
    *cursorPos = *cursorPos + 1;
    if (*cursorPos > *printInt) { // 2
      *cursorPos = 0;
    }
  } else if (keypressed == 'C') {
    *cursorPos = *cursorPos - 1;
    if (*cursorPos < 0) {
      *cursorPos = *printInt;
    }
  } else if (isDigit(keypressed) || keypressed == '*') { 
    //lcd.setCursor(*cursorPos, 1);

    if (keypressed == '*') {

      // If cursorPos is first digit, move to second digit after "0"
      if (*cursorPos == 0) {
      
      // If decimalPoint exists and is less than cursorPos
      //  move numbers to left until *cursorPos
      } else if (*decimalPos < *cursorPos) {
        for (int i = *decimalPos; i < *cursorPos; i++) {
            printVal[i]=printVal[*decimalPos+i];
        }
        printVal[*cursorPos] = '.';
        *decimalPos = *cursorPos; // Store where the Decimal point is
        
      // Else - 
      } else if (*decimalPos > *cursorPos) {
          // Shift number to right
          for (int i = *decimalPos; i < arrLen+1; i++) {
          
            printVal[i]=printVal[i+1];
            
          }

        printVal[*cursorPos] = '.';
        *decimalPos = *cursorPos; // Store where the Decimal point is

        // if the decimal point is first
        // printVal[0] = 0
        // printVal[1] = '.'
        // printVal[0] = shift values across by 2
        if (*cursorPos == 0) {

          for (int i = 0; i < arrLen-1; i++) {
            printVal[arrLen-i+1] = printVal[arrLen-i];
          }
          
          printVal[1]='.';
          printVal[0]=0;
        }
      }
    } else { // Any digit entered
        //
        // If print number exceeds significant figure before decimal position
        //  && no decimal position exists...
        //
        // Automatically place decimal point followed by number
        //
        //    decimalPos = -1 if no decimal position present
        if ((*cursorPos >= sigFig) && (*cursorPos == *decimalPos || *decimalPos == -1)){ //
          printVal[*cursorPos] = '.';
          *cursorPos = *cursorPos + 1;
          *decimalPos = sigFig;
        }
      lcd.print(keypressed);
      printVal[*cursorPos] = keypressed;

      // If decimal position is replaced with number but at significant figure limit...
      if ((*decimalPos == *cursorPos) && (*cursorPos == sigFig-1)){
        printVal[sigFig] = '.';
        *decimalPos = sigFig;
      // if decimal point is replaced
      } else if (*decimalPos == *cursorPos) {
          *decimalPos = -1;
      }

        // If SigNum or 3 dp numbers haven't been placed (Max number of placements)
      if ((*cursorPos > 1 && (*decimalPos == 0 || *decimalPos == sigFig)) || *cursorPos > 2) { // If max numbers placed 3 digits (Remember 0 is a digit)
        //Serial.println("Cursor set 0");
        *cursorPos = 0;
        *printInt = 3;
      } else {
        *cursorPos = *cursorPos + 1;
      }
    }
  }

  
  //lcd.setCursor(*cursorPos, 1);
  lcd.setCursor(0, 1);
  lcd.print(printVal);
}

// Purpose is to set diameter which changes usable rates
// V = pi*(r^2)*h - equation for the cylinder
void SetVolume() {
  char structLCDVal[] = {};
  //char *VolVal;
  int VolInt = 4;
  bool Menu = 0;
  int cursorPos = 0;
  char keypressed = myKeypad.getKey(); // Scan the numpad for input
  float tempVal = (float) volFloat/100; // temporary float for the volume
  dtostrf(tempVal, 2, 2, structLCDVal); // split number into 2 decimal places
  //  Serial.println(floatVar[3]);
  //p = strstr('.', structLCDVal);
  int decimalPos = (strcmp(structLCDVal,".")-1); //index begins at 0 so -1
  int sigFig = 2;

  lcd.clear();
  lcd.setCursor(2, 0);
  lcd.print("Set Volume");
  lcd.setCursor(0, 1);
  lcd.print(tempVal);
  lcd.setCursor(13, 1);
  //motorStatus[1][1][2] = unitVal; // Stored unit
  if (flowStatus[0] = 0) { // if pct
    lcd.print("min");
  } else {
    lcd.write(0);
  }

  while (Menu != 1) {
    // Set rate either as percentage or flow rate
    lcd.setCursor(cursorPos,1);
    lcd.cursor();
    if (keypressed == 'D') {
      Menu = 1;
    } else if (isDigit(keypressed) || keypressed == '*' || keypressed == 'B' || keypressed == 'C')  {

      LCDinput(keypressed, &decimalPos, structLCDVal, &VolInt, &cursorPos, sigFig); // input numbers
      //VolVal = structLCDVal;
      lcd.setCursor(0, 1);
      lcd.print(structLCDVal);
    }
    keypressed = myKeypad.getKey();
  }
  
  tempVal = atof(structLCDVal);
  volFloat = tempVal*100; // Convert to int - Lower memory storage

  EEPROM.put(3, volFloat); // Export int
  setTargetTime(); // Calculate time required for volume

  lcd.clear();
}

// Simple void to set target time when flow rate or volume is changed
void setTargetTime() {
  //EEPROM.get(1, floatVar);
  //EEPROM.get(2, flowStatus);

  float tempVal = volFloat/100; // Convert int to float (2 decimal places) - Saves memory

  if (flowStatus[0] != 0) {
    // Convert ul to ml
    // [rate][dia][diaScale][volume]
    tempVal = tempVal * 1000;
    if (flowStatus[0] == 1) {
      tempVal = tempVal / 60; // Convert seconds to minutes of rate (changed here earlier to save coding within startTime)
    }
  }
  targetTime = ((tempVal / floatVar[0]) * 60000); // ( volume (minutes) / rate (converted to minutes) ) ¦¦ * from minutes to milliseconds
}


float Int2Float(int *intVal, int sizeInt, int decimalPos) {
  // intVal is distint array to be converted to float
  // sizeInt is the size of the array
  // tempInt is the temporary int for storing the float value
  // tempVal is tempInt convert to decimal
  int tempInt;
  float tempVal;

  // Convert to float than storing in distint arrays
  const int arrLen = sizeof(intVal) / sizeof(intVal[0]);
  for (int i = 0; i < arrLen; i++) {
    tempInt = ((intVal[i] * pow(10, ((arrLen - 1) - (i)))) + tempInt); // Simple conversion for 3 inputs from numpad - convert from single digits to one number
  }

  // Placing decimal point requires division by a desired factor determined by number length - decimalPos
  //    Remember indexing starts at 0, so no need for +1
  if (decimalPos != 1) {
    tempVal = tempInt / (10 * (sizeInt - decimalPos));
  }
  return tempVal;
}

//// simple int void to calculate the number of digits
int sizeInt(int x) {
  int valLen;

  if (x > 9999)
    valLen = 5;
  else if (x > 999)
    valLen = 4;
  else if (x > 99)
    valLen = 3;
  else if (x > 9)
    valLen = 2;
  else
    valLen = 1;

  return valLen;
}

//// void for assigning infusion or withdrawal
void SetInfWith() {
  bool Menu = 0;
  char keypressed = myKeypad.getKey();

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Infuse/Withdraw");
  keypressed = 'B';

  while (Menu != 1) {
    lcd.setCursor(3, 1);
    if (keypressed == 'B' || keypressed == 'C') {
      if (InfWith == 1) {
        InfWith = 0;
        lcd.print(" Infuse ");
        lcd.write(126);
      } else if (InfWith == 0) {
        InfWith = 1;
        lcd.print("Withdraw");
        lcd.write(127);
      }
    } else if (keypressed == 'D') {
      Menu = 1;
    }
    keypressed = myKeypad.getKey(); // Refresh keypressed
  }
}
