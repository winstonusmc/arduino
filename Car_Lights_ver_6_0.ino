/* Version 6.0 -  * Changing the brake lights to remove the turn signal function.   
 */
#include <LowPower.h> 
#include <mcp_can.h>
#include <SPI.h>

//  Outputs
const int buzzerPin = 13;
const int domeLight = 11;
const int lightPower = 10;
const int turnRight = 7; 
const int turnLeft = 6;
const int brakeLight = 8;


// Analog Inputs
const int turnSignalSwitch = A0;
const int headlightSwitch = A1;
const int wiperSwitch = A2;
const int lightSensor = A3;
const int ignSwitch = A5;
const int accSwitch = A6; 

// Digital Inputs
const int lockSensor = 2; //INT 0
const int keySwitch = 3; // INT 1
const int washSwitch = 4; 
const int brakeIn = 5; 
const int fogSwitch = 14;
const int seatBelt = 15;
const int driverDoorSwitch = 18; // INT 5
const int passDoorSwitch = 19; // INT 4
const int hatchSwitch = 20; // INT 3

// Interupts
const int driverInterupt = 5;
const int passInterupt = 4;
const int lockInterupt = 0;
 
// Constants
const int brightness = 0; //sets brightness of turn signals as Parking Lights, 
const int signalInterval = 333; //Turn signal interval in milliseconds, SAE states 90 times a minute (333ms)
const int hazardInterval = 333; //Hazard interval in milliseconds, SAE states 90 times a minute (333ms)
const int lightTime = 300; //Interior light timeout in seconds
const int headlightTime = 290; //Headlight time out
const int sleepTime = 301; //Sleep time out
const int secondInterval = 1000; //sets speed of counters to per second
const int onInterval = 6; //milliseconds for 0 to 255 to occur in 1.5 second
const int offInterval = 6; //milliseconds for 255 to 0 to occur in 1.5 second
const int factor = 3; //sets severity of log curve for dimout
const int domeTime = 30; //Dome light timeout in seconds
const int washTime = 4; //windshield washer wipe time in seconds
const int debounceDelay = 200; //Debounce time
const int dayLight = 630; //Daylight level for auto lights
const int autoLightDelay = 30; //Delay when the lights turn off from daylight or ign off
const int wiperDelay = 4; //Delay for wiper headlight turn-on
const int flashRate = 660; //sets locking/unlocking flash speed
const int flashInterval = 333; //sets locking/unlocking flash duration in milliseconds
const int hornRate = 333; //sets horn chirp speed
const int hornInterval = 100; //length of horn chirp in milliseconds
const int canXmitInterval = 100; // The interval of Can Xmit

unsigned char Flag_Recv = 0;
unsigned char len = 0;
unsigned char buf[8];
INT32U canId = 0x000;
char str[20];

//State change detection (Values are the static state of the Switches)
int lastDriverDoorState = 1;
int lastPassDoorState = 1;
int lastIgnState = 0;
int lastWashState = 1;
int lastLockState = 0;
int lastRightState = 0;
int lastLeftState = 0;
int lastHazardState = 0;
int lastAutoState = 0;
int lastFlashState = 0;
int lastButton1State = 0;
int lastParkingState = 0;


//Debounce (not currently used)
int lastDriverReading = 1;
int lastPassReading = 1;
int driverReadingState = 0;
int passReadingState = 0;
unsigned long lastDriverDebounce, lastPassDebounce; 


//Countdown timers
int domeCount, lightCount, washCount, autoCount, wiperCount, flashCount, hornCount, headlightCount, canCount,
    sleepCount;

//Timer variables
unsigned long previousMillis, previousXmit, previousLight, previousDome, previousDim, previousWash,
              previousAuto, previousWiper, previousBlink, previousFlash, previousHorn, previousChirp,
              logDim, dimCount, previousHead, previousCanXmit, previousCanTimeout, previousSleepTime; 

//CAN bytes
unsigned char byteF_5fa, byteG_5fa, byteA_60d, byteB_60d, byteA_1f9, byteA_35d, byteB_35d, byteC_35d, byteA_358,
              byteA_625, byteB_625, byteC_625, byteD_625, byteA_1F9;

//State variables
int leftState,rightState, hazardState, brakeState, fogState, parkingState, parkingSwitchState, lowBeamState, highBeamState, 
    parkingBrightness, wiperOn, wiperHigh, washState, fanState, acState, ignState, accState, lightLevel, 
    driverDoorState, passDoorState, lightPowerState, domeLightState, keySwitchState, washTimeout, 
    domeTimeout, lockState, unlockState, autoLightState, autoLightStatus, ignSwi, wiperState, flashState, 
    doubleFlash, doubleHorn, singleFlash, singleState, doubleState, hazardSwi, rightSwi, leftSwi, hornChirp, 
    headlightTimeout, canAwakeState, canSleepState, canTimeoutState, buttonState, tailFeedback, lowFeedback,
    highFeedback;

MCP_CAN CAN(49); // Set CS to pin 49 from the Mega

void wakeUp() {
  
}

void setup() {

  Serial.begin(115200); //Because I like that Baud
  pinMode(lightPower, OUTPUT);
  pinMode(domeLight, OUTPUT);
  pinMode(turnRight, OUTPUT);
  pinMode(turnLeft, OUTPUT);
    pinMode(brakeLight, OUTPUT);
  pinMode(buzzerPin, OUTPUT);

  CAN.begin(CAN_500KBPS); // init can bus : baudrate = 500k
    

sleepCount = sleepTime;
headlightCount = headlightTime;


}


void loop() {
  hazardSwi = analogRead(turnSignalSwitch);
  int lightValue = analogRead(lightSensor);
  //Serial.print ("Turn Signal: ");
  //Serial.println (turnSignalSwitch);
  
      
  ign();
  acc();
  interior();
  if (lightValue > 10) { //Auto light function
     autoLightStatus = HIGH;
     autoLights();
  }
  else {
     autoLightStatus = LOW;
     autoCount = 0;
     headlight();
  }
  wiper();
  wash();
  if (hazardSwi < 250) { //splits turn signal and hazard commands, hazard over ride
    hazard();
  }
  else {
    turn();
  }
  canRcv();
  rcvBytes();
  canBits();
  serialXmit();
  canTimeout();
  canXmit();
 
  

}

void ign() {
  ignSwi = analogRead(ignSwitch);
  if (ignSwi > 250) {
    ignState = HIGH;
  }
  if (ignSwi < 100) { //Hysteresis to add stability to the ignState, caused funny resets without it
    ignState = LOW;
  }
}

void acc() {
  int switchIn = analogRead(accSwitch);
  if (switchIn > 250) {
    accState = HIGH;
  }
  if (switchIn < 100) { //Hysteresis to add stability to the accState, caused funny resets without it
    accState = LOW;
  }
}

void interior() {
  
  driverDoorState = digitalRead(driverDoorSwitch);
  passDoorState = digitalRead(passDoorSwitch);
  keySwitchState = digitalRead(keySwitch);
  lockState = digitalRead(lockSensor);
    
  
  //--Timer resets--
  
  //Drivers door is the only door that the dome timeout functions
  if (driverDoorState != lastDriverDoorState) { //When the drivers door is either opened or closed
    lastDriverDoorState = driverDoorState;
    lightCount = lightTime; //reset light power timeout
    domeCount = domeTime; //reset dome timeout delay 
    sleepCount = sleepTime; //reset sleep count  
  }
  
  if (passDoorState != lastPassDoorState) { //When the passengers door is either opened or closed
    lastPassDoorState = passDoorState; 
    lightCount = lightTime; //reset light power timeout
    sleepCount = sleepTime; //reset sleep count 
       
  }

  if (ignState != lastIgnState) { // ign switched off
    lastIgnState = ignState;
    headlightCount = headlightTime; //Reset headlight timeout
    if (ignState == LOW) {
      domeCount = domeTime; //reset dome timeout delay
      lastLockState = lockState;
      lightCount = lightTime; //reset light power timeout
      sleepCount = sleepTime; //reset sleep count 
      
    }
  }

  // when doors are unlocked when the ignition is off, the dome light turns on for the delay time
  if (lockState != lastLockState && ignState == LOW && driverDoorState == HIGH) { //Ignition off and door closed
    lastLockState = lockState;
    sleepCount = sleepTime;
    canCount =0;
    sleepCount = sleepTime; //reset sleep count 
    if (lockState == LOW) { //Door Locks are pull down when unlocked
      lightCount = lightTime; //reset light power timeout
       domeCount = domeTime; //reset dome timeout delay
      previousFlash = millis(); // Unlock flash single
      previousChirp = millis(); // unlock chirp single
      
    }
    else {
      domeCount = 0; // Cancels dome delay time when locking
      autoCount = 0; //turns off auto lights after ign off when locking
      doubleFlash = 1; // Lock double flash
      doubleHorn = 1; 
    }
  }
  else {
    doubleFlash = 0; 
    doubleHorn = 0;
  }

  if (driverDoorState == HIGH && ignState == HIGH) { // cancels dome delay time when ign on
    domeCount = 0;
  }
 
  //--Light Power Time Out--
  if (millis() - previousLight >= secondInterval) {
    previousLight = millis();
    if (lightCount > 0  && accState == LOW) { //only counts when acc is off
      lightCount --;
      }
  }
  
  if (lightCount >= 1 || accState == HIGH) {    
    lightPowerState = HIGH;
  }
  else {
    lightPowerState = LOW;
  }

  //--Dome Light Time Out--
  if (millis() - previousDome >= secondInterval) {
    previousDome = millis();
    if (domeCount >= 1 && driverDoorState == HIGH) { //only counts when driver door is closed
      domeCount --;
      }
  }  
  
  if (domeCount >= 1) {    
    domeTimeout = HIGH;
  }
  else {
    domeTimeout = LOW;    
  }
  
  //--Dome Light State Update--
  if ((driverDoorState == LOW || passDoorState == LOW || domeTimeout == HIGH) && sleepCount > 3) {
    
    domeLightState = HIGH;
  }
  else {
    domeLightState = LOW;
  }

  //---Dome light Dim---
  //dim on
  if (domeLightState == HIGH) { // count up to 16 when high
    if (millis() - previousDim >= onInterval) {
      previousDim = millis();
      if (dimCount < 256) { //Using 16 instead of 255 to provide a logarithimic curve when squared
        dimCount++;
      }
    }
  }
  //dim off
  else {
    if (millis() - previousDim >= offInterval) { // count down to 0 when low
      previousDim = millis();
      if (dimCount > 0) {
        dimCount--;
      }
    }
  }

  long factor2 = pow(256, (factor - 1)); // Allows for changing of log curve severity
  logDim = pow(dimCount, factor)/(factor2); // remaps the dimming for more linear appearance
  

  //--Lock Unlock Flashing--

  if ((flashCount > 0 && flashCount < 2) || doubleFlash == 1) {
    if (millis() - previousBlink >= flashRate) { 
        previousBlink = millis();
        previousFlash = millis(); 
        flashCount++;
    }
  }

  if (flashCount >= 2) flashCount = 0;

  if (millis() - previousFlash < flashInterval && millis() > flashInterval) flashState = HIGH;
  else flashState = LOW;

  //--Horn Chirp--
   if ((hornCount > 0 && hornCount < 2) || doubleHorn == 1) {
    if (millis() - previousHorn >= hornRate) { 
        previousHorn = millis();
        previousChirp = millis(); 
        hornCount++;
    }
  }

  if (hornCount >=2) hornCount = 0;

  if (millis() - previousChirp < hornInterval && millis() > hornInterval) { //Previous time reset by flash on
    hornChirp = HIGH;
  }
  else {
    hornChirp = LOW;
  }
  
  
  //Write
  
  digitalWrite(lightPower, lightPowerState);
  analogWrite(domeLight, logDim);
}

void autoLights() {
  lightLevel = analogRead(lightSensor);
  int switchIn = analogRead(headlightSwitch);

  if (millis() - previousAuto >= secondInterval) {
    previousAuto = millis();
    if (autoCount >= 1 && autoLightState == LOW) {
      autoCount --;
      }
  }
  
  if (lightLevel > dayLight && ignState == HIGH) { //ign on and ambient light low
    autoLightState = HIGH;
  }
  
  if (lightLevel < (dayLight - 50) && ignState == HIGH) { //ign on and ambient light high
    autoLightState = LOW;
  }
  if (ignState == LOW) { //ign off
    autoLightState = LOW;
  }
  
  if (autoLightState != lastAutoState) { //Timer to prevent quick turn off after light goes below desired level
    lastAutoState = autoLightState;
    autoCount = autoLightDelay; //reset auto timer to preset value
  }
  
  if (autoLightState == HIGH || autoCount > 0 || wiperState == HIGH) { //
    parkingState = HIGH;
    if (switchIn > 275) { // if headlight switch is on, then the high beam select is functional
      lowBeamState = HIGH;
      highBeamState = LOW;
    }
    else  {
      highBeamState = HIGH;
      lowBeamState = LOW;
    }
    if (wiperState == HIGH) {
      fogState = HIGH;
    }
    else {
      fogState = LOW;
    }
  }

  else {
    parkingState = LOW;
    lowBeamState = LOW;
    highBeamState = LOW;
  }
}

void headlight() {

  int switchIn = analogRead(headlightSwitch); 

  //Serial.print(switchIn); //on for headlight switch debug
  
  //--Head light Time Out--
  if (millis() - previousHead >= secondInterval) {
    previousHead = millis();
    if (headlightCount > 0  && ignState == LOW && parkingSwitchState == HIGH) { //only counts when ign is off and lights are on
      headlightCount --;
      }
  }
  
  if (headlightCount == 0 && ignState == LOW) { //Either counting has ended or ignition is off   
    headlightTimeout = HIGH;
  }
  else {
    headlightTimeout = LOW;
  }

  
  //**Parking Lights**
  if (switchIn < 900) {
    parkingSwitchState = HIGH;
    if (headlightTimeout == LOW) {
      parkingState = HIGH; 
      //fogState=HIGH;
      parkingBrightness = brightness;
    }
    else {
      parkingState = LOW; 
      //fogState = LOW;
      parkingBrightness = 0;
      //headlightCount = headlightTime; //once lights are turned off, then reset headlight count
    }
    
  }
  else {
    parkingSwitchState = LOW;
    parkingState = LOW;    
  }
  

  if (parkingSwitchState == LOW) headlightCount = headlightTime;
  
  //**Low Beams**
  if ((switchIn > 275) && (switchIn < 575) && headlightTimeout == LOW) {
    lowBeamState = HIGH;
    
  } 
  else {
    lowBeamState = LOW;
    
  }
  //**High Beams**
  if (switchIn < 275 && headlightTimeout == LOW) {
    highBeamState = HIGH;
  }
  else  {
    highBeamState = LOW;
  }
}

void wiper() {

  int switchIn = analogRead(wiperSwitch);
  //Serial.print("  Wiper Switch: ");
  //Serial.println(switchIn); //for debug

// **Windshield Wipers**  
  if ((switchIn < 850) || washTimeout == HIGH) { //Wiper On signal
    wiperOn = HIGH;
  }
  else {
    wiperOn = LOW;
  }    
  if (switchIn < 500)  { //Wiper Fast Signal
    wiperHigh = HIGH;
  }
  else {
    wiperHigh = LOW;
  }

  //--Wiper delay for autolights--
  if (millis() - previousWiper >= secondInterval) {
    previousWiper = millis();
    if (wiperCount < wiperDelay) {
      if (wiperOn == HIGH && washTimeout == LOW) {
        wiperCount ++;
      }
    }
  }
  if (wiperOn == LOW) {
    wiperCount = 0;
  }
  if (wiperCount == wiperDelay) {
    wiperState = HIGH;
  }
  else {
    wiperState = LOW;
  }
}

void wash() {
  washState = digitalRead(washSwitch);

  if (washState != lastWashState && millis() > 4000) {
    lastWashState = washState;
    if (washState == HIGH) {
      washCount = washTime;
    }
  }
   
  
  if (millis() - previousWash >= secondInterval) {
    previousWash = millis();
    if (washCount >= 1 && washState == HIGH) {
      washCount --;
      }
  }  
  
  if ((washCount > 0 || washState == LOW) && wiperHigh == LOW) {
    washTimeout = HIGH;
  }
  else {
    washTimeout = LOW;
  }
  
}

void turn() {

  int switchIn = analogRead(turnSignalSwitch);
  brakeState = digitalRead(brakeIn);
  if ((switchIn > 250) && (switchIn < 500)) {
    rightSwi = HIGH;
  }
  else {
    rightSwi = LOW;
  }
  if ((switchIn > 500) && (switchIn < 850)) {
    leftSwi = HIGH;
  }
  else {
    leftSwi = LOW;
  }
  unsigned long currentMillis = millis();

  //Serial.print("  Turn Switch: ");
  //Serial.print(switchIn); //for debug

// **Turn Signals**  
  if (rightSwi == HIGH && ignState == HIGH) { //Right Turn Signal
    if (currentMillis - previousMillis >= signalInterval) { //if time elapsed is greater than the signal interval
      previousMillis = currentMillis; //then reset time
      if (rightState == LOW) { //if right state was low
        rightState = HIGH; //make highe
      }
      else {
        rightState = LOW; // if it was high, make low
      }
    }
  }
  else {
    rightState = LOW;
  }  

  if (rightState != lastRightState) {
    lastRightState = rightState;
    if (rightState == HIGH){
      tone(buzzerPin,2400,4);
    }
    else {
      tone(buzzerPin,2100,4);
    }
  }
    
  if (leftSwi == HIGH && ignState == HIGH)  { //Left Turn Signal
    if (currentMillis - previousMillis >= signalInterval) { //if time elapsed is greater than the signal interval
      previousMillis = currentMillis; //then reset time
      if (leftState == LOW) { //if right state was low
        leftState = HIGH;
      }
      else {
        leftState = LOW;
      }
    }
  }
  else {
    leftState = LOW;  
  }

  if (leftState != lastLeftState) {
    lastLeftState = leftState;
    if (leftState == HIGH){
      tone(buzzerPin,2400,4);
    }
    else {
      tone(buzzerPin,2100,4);
    }
  }
  
  if (rightState == HIGH || flashState == HIGH) {
    analogWrite(turnRight, 255);
  }
  else {
    analogWrite(turnRight, parkingBrightness);
  } 
  if (leftState == HIGH || flashState == HIGH) {
    analogWrite(turnLeft, 255);
  }
  else {
    analogWrite(turnLeft, parkingBrightness);
  }  
//**Brakes**  
  
  if (brakeState == LOW) { //right rear will operate as a brake light
    analogWrite(brakeLight, 255); 
  }
  else {
    analogWrite(brakeLight, parkingBrightness);
  }
}
  
void hazard() {
  
  int switchIn = analogRead(turnSignalSwitch);
  unsigned long currentMillis = millis();

  if (switchIn < 250) { //Hazard on
    if (currentMillis - previousMillis >= hazardInterval) { //if time elapsed is greater than the signal interval
      previousMillis = currentMillis; //then reset time
      if (hazardState == LOW) { //if right state was low
        hazardState = HIGH;
      }     
      else {
        hazardState = LOW;
      }

      if (hazardState == LOW) { //updates for CAN BUS dash signal lights
        rightState = LOW;
        leftState = LOW;
      }
      else {
        rightState = HIGH;
        leftState = HIGH;
      }
    }

  if (hazardState != lastHazardState) {
    lastHazardState = hazardState;
    if (hazardState == HIGH){
      tone(buzzerPin,2400,4);
    }
    else {
      tone(buzzerPin,2100,4);
    }
  }
        
  digitalWrite(turnRight, hazardState);
  digitalWrite(turnLeft, hazardState);
      
  }
}

// **Receive Can Message** 
void canRcv() {
   if(CAN_MSGAVAIL == CAN.checkReceive()) {         
    CAN.readMsgBuf(&len, buf);    
    //canId = CAN.getCanId();
    //Serial.print("ID: ");
    //Serial.print(canId, HEX);
    //Serial.print(" ");
    /*    
    for(int i = 0; i<len; i++) {    // print the data
        Serial.print(buf[i]);Serial.print("\t");
    }
    Serial.println();*/
    switch (CAN.getCanId()) {
    case 0x1F9:
      byteA_1F9= buf[1];
      //byteG_5fa = buf[7];
      break; 
    
    
    case 0x625:  // Group 1 
      byteA_625 = buf[1];
      byteB_625 = buf[2];
      byteC_625 = buf[3];
      byteD_625 = buf[4];
      break;
      
    
    /* 
    case 0x5f2:  // Group 2 
      //baro = word(buf[0] , buf[1]);
      //MAP = word(buf[2] , buf[3]);
      //mat = word(buf[4] , buf[5]);
      //clt = word(buf[6] , buf[7]);
      break;
    
    case 0x5f3:   // Group 3 
      //tps = ((buf[0] * 256) + buf[1]);
      //bat = ((buf[2] * 256) + buf[3]);
      //afr1 = ((buf[4] * 256) + buf[5]);
      //afr2 = (((buf[6] *256) + buf[7]); 
      break;*/
    }
  }
}
void rcvBytes() {
  Serial.print(" byteA_1F9: ");
  Serial.println(byteA_1F9, BIN);
  //Serial.print(" Tail Feedback: ");
  //Serial.print(tailFeedback);
  //Serial.print(" High Beam Feedback: ");
  //Serial.print(highFeedback);  
  //Serial.print(" Low Beam Feedback: ");
  //Serial.println(lowFeedback);  
  //Serial.print(" byteB_1F9: ");
  //Serial.print(byteB_625, BIN);
  //Serial.print(" byteC_625: ");
  //Serial.print(byteC_625, BIN);
  //Serial.print(" byteD_625: ");
  //Serial.println(byteD_625, BIN);
    
  if ((byteG_5fa & 32) && (ignState == HIGH)){  //bitmask for A/C bit from Megasquirt
  acState = HIGH;
  }
  else acState = LOW;
  if (((byteF_5fa & 64)/64) && (ignState == HIGH) || acState == HIGH) { //bitmask for fan bit from Megasquirt
  fanState = HIGH;
  }
  else fanState = LOW;
  if (byteA_625 & 64) tailFeedback = 1; else tailFeedback = 0;  
  if (byteA_625 & 32) lowFeedback = 1; else lowFeedback = 0;
  if (byteA_625 & 16) highFeedback = 1; else highFeedback = 0;

}

void canBits() {

  byteA_1f9 =   (fanState * 64) //Cooling Fan
              + (acState * 8); // A/C Compressor request
  byteA_358 =   (hornChirp * 64); //Horn Chirp request
  byteA_35d =   (ignState * 128); //IGN on
  byteB_35d =   (canAwakeState * 64) //wake 
              + (canAwakeState * 16) //wake 
              + (canSleepState * 4) //sleep 
              + (canSleepState * 1) //sleep 
              + (0);
  byteC_35d =   (wiperOn * 128) //wiper continuous 
              + (wiperOn * 64) //wiper on 
              + (wiperHigh * 32) //wiper high
              + (0);
  byteA_60d =   (parkingState *4) //praking lights
              + (lowBeamState * 2) //low beams
              + (0);
  byteB_60d =   (rightState * 64) //right signal indicator
              + (leftState * 32) //left signal indicator
              + (highBeamState * 8) //high beams
              + (ignState * 4) //Ign on
              + (accState * 2) //ACC on
              + (fogState * 1); //Fog Lights
    
              
  

}
void serialXmit() {
  
  const int interval = 100; //Transmit interval in milliseconds
  unsigned long currentXmit = millis();
  //refresh rate
  if (currentXmit - previousXmit >= interval) { 
    previousXmit = currentXmit; 
    
    //Serial.print(" ID: ");
    //Serial.print("0x5fa");
    //Serial.print("\t");
    //Serial.print(byteF_5fa);
    //Serial.print("\t");
    //Serial.print(" ID: ");
    //Serial.print("0x5fa");
    //Serial.print("\t");
    //Serial.print(byteB_35d, BIN);
    //Serial.print("\t");
    //Serial.print(byteC_35d);
    //Serial.print("\t");
    //Serial.print(" Can Count: ");
    //Serial.print(canCount);
    //Serial.print(" State: ");
    //Serial.print(canTimeoutState);
    //Serial.print(" Sleep Count: ");
    //Serial.print(sleepCount);
    //Serial.print(" Button State: ");
    //Serial.print(buttonState);
    
    
    //Serial.println(" ");
    

    //Serial.print (" Key Switch: ");
    //Serial.print (keySwitchState);
    //Serial.print (" ACC: ");
    //Serial.print (accState);
    //Serial.print (" IGN: ");
    //Serial.print (ignState);
    //Serial.print ("  Parking State: ");
    //Serial.print (parkingState);
    //Serial.print (" Low Beam State: ");
    //Serial.print (lowBeamState);
    //Serial.print (" High Beam State: ");
    //Serial.print (highBeamState);
    //Serial.print (" Wiper On: ");
    //Serial.print (wiperOn);
    //Serial.print (" Washer: ");
    //Serial.print (washState);
    //Serial.print (" Wiper High: ");
    //Serial.print (wiperHigh);
    //Serial.print (" Left Turn: ");
    //Serial.print (leftState);
    //Serial.print (" Right turn: ");
    //Serial.print (rightState);
    //Serial.print (" Brakes: ");
    //Serial.print (brakeState);
    //Serial.print (" Lock: ");
    //Serial.print (lockState);
    //Serial.print (" Dome: ");
    //Serial.print (domeLightState);
    //Serial.print (" Driver Door: ");
    //Serial.print (driverDoorState);
    //Serial.print (" Pass Door: ");
    //Serial.print (passDoorState);
    //Serial.print (" Fan: ");
    //Serial.print (fanState);
    //Serial.print (" Light Level: ");
    //Serial.print (lightLevel);
    //Serial.print (" Wiper Count: ");
    //Serial.print (wiperCount);
    //Serial.print (" Wiper State: ");
    //Serial.print (wiperState);
    //Serial.print (" Horn Chirp: ");
    //Serial.print (hornChirp);
    //Serial.print (" Double Flash: ");
    //Serial.print (doubleFlash);
    //Serial.print (" Flash Count: ");
    //Serial.print (flashCount);
    //Serial.print (" Horn Count: ");
    //Serial.print (hornCount);
    //Serial.print (" Headlight Count: ");
    //Serial.print (headlightCount);
    //Serial.print (" Headlight Timeout: ");
    //Serial.print (headlightTimeout);
    //Serial.print (" domeCount: ");
    //Serial.print (domeCount);
    //Serial.print (" dimCount: ");
    //Serial.print (dimCount);
    //Serial.print (" lightCount: ");
    //Serial.print (lightCount);
    //Serial.print (" washCount: ");
    //Serial.print (washCount);
    //Serial.print (" ignSwi: ");
    //Serial.print (ignSwi);
    //Serial.print (" fanState: ");
    //Serial.print (fanState);
    //Serial.print (" acState: ");
    //Serial.print (acState);
    
    
  
    
    //Serial.println(" ");
    

    
  } 
}

void canTimeout() {
  
  if (hornChirp == HIGH || ignState == HIGH || parkingState == HIGH || lowBeamState == HIGH || highBeamState == HIGH || rightState == HIGH || leftState == HIGH || fogState == HIGH) {
    canCount = 0;
    sleepCount = sleepTime;
    canTimeoutState =  1;
  }
  else canTimeoutState = 0;
  

  //buttonState = digitalRead(buttonPin);
  //canTimeoutState = 0;
  
  if (canCount < 10 && canTimeoutState == 0) {
    if(millis() - previousCanTimeout > 1000) {
      previousCanTimeout = millis();
      canCount ++;
    }
  }

  if (canCount < 8) {  //Sets the CAN message to the IPDM to wake
    canAwakeState = 1;
    canSleepState = 0;  
  }
  else if (canCount >= 8) { //Sets the CAN message to the IPDM to sleep
    canAwakeState = 0;
    canSleepState = 1;
  }


  // **SLEEP**
  
  if (canTimeoutState == 0 && sleepCount > 0) {
    if (millis() - previousSleepTime > secondInterval) {
      previousSleepTime = millis();
      sleepCount --;
    }
  }

  if (sleepCount < 4) CAN.setSleep(1); // Puts the MCP2515 to sleep, custom library
  else CAN.setSleep(0); // Puts the MCP2515 to normal mode
  
  if (sleepCount < 3) {
    domeCount = 0;
    lightCount = 0;
  }
  
  if (sleepCount == 0) {
     
    
    attachInterrupt(driverInterupt, wakeUp, CHANGE);
    attachInterrupt(passInterupt, wakeUp, CHANGE);
    attachInterrupt(lockInterupt, wakeUp, CHANGE);
    //LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF); //Powers down controller

    CAN.begin(CAN_500KBPS); //Resets CAN bus
    detachInterrupt(driverInterupt);
    detachInterrupt(passInterupt); 
    detachInterrupt(lockInterupt);
     
  }
  
}
  
void canXmit() {  
  
  
  if (canCount < 10) {
    if (millis() - previousCanXmit >= canXmitInterval) { 
      previousCanXmit = millis();
    
      unsigned char msg_60d[8] = {byteA_60d, byteB_60d, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      unsigned char msg_1f9[2] = {byteA_1f9, 0x00};
      unsigned char msg_358[8] = {byteA_358, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      unsigned char msg_35d[6] = {byteA_35d, byteB_35d, byteC_35d, 0x00, 0x00, 0x00};
    
      // send data:  
      CAN.sendMsgBuf(0x60d, 0, 8, msg_60d);
      //CAN.sendMsgBuf(0x1f9, 0, 2, msg_1f9);
      CAN.sendMsgBuf(0x35d, 0, 6, msg_35d);
      CAN.sendMsgBuf(0x358, 0, 8, msg_358);
    
    }
  }
}

