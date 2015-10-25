#include <mcp_can.h>
#include <SPI.h>

const int clicker = 10; //analog output
const int frontRight = 9; //analog output
const int lowBeamOut = 8;
const int parkingOut = 7;
const int rearRight = 6; //analog output
const int rearLeft = 5; //analog output
const int highBeamOut = 4;
const int frontLeft = 3; //analog output

const int turnSignalSwitch = A0;
const int headlightSwitch = A1;
const int brakeIn = A2;

const int brightness = 30; //sets brightness of Parking Lights, 
const int signalInterval = 700; //Turn signal interval in milliseconds, SAE states 90 times a second (700ms)
const int hazardInterval = 300;

int leftState = HIGH;
int rightState = HIGH;
int hazardState = HIGH;
int brakeState = LOW;
int parkingState = LOW;
int lowBeamState = HIGH;
int highBeamState = HIGH;
int parkingBrightness = 0;

unsigned long previousMillis = 0;

MCP_CAN CAN(10); // Set CS to pin 10

void setup() {

  Serial.begin(115200);
  pinMode(parkingOut, OUTPUT);
  pinMode(lowBeamOut, OUTPUT);
  pinMode(highBeamOut, OUTPUT);
  pinMode(brakeIn, INPUT);
  pinMode(clicker, OUTPUT);
  pinMode(frontRight, OUTPUT);
  pinMode(frontLeft, OUTPUT);
  pinMode(rearRight, OUTPUT);
  pinMode(rearLeft, OUTPUT);

START_INIT:

    if(CAN_OK == CAN.begin(CAN_500KBPS))                   // init can bus : baudrate = 500k
    {
        Serial.println("CAN BUS Shield init ok!");
    }
    else
    {
        Serial.println("CAN BUS Shield init fail");
        Serial.println("Init CAN BUS Shield again");
        delay(100);
        goto START_INIT;
    }  
}

void loop() {
  int hazardSwi = 0;
  hazardSwi = analogRead(turnSignalSwitch);
    
  headlight();
  if (hazardSwi < 250) { //splits turn signal and hazard commands, hazard over ride
    hazard();
  }
  else {
    turn();
  }
  Serial.print(" Left: ");
  Serial.print(leftState);
  Serial.print(" Right: ");
  Serial.print(rightState);
  Serial.print(" Brakes: ");
  Serial.print(brakeState);
  Serial.print(" High Beam: ");
  Serial.print(highBeamState);
  Serial.print(" "); 
  
  //unsigned char msg2[8] = {lightValue, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  unsigned char msg[4] = {leftState, rightState, brakeState, highBeamState};
  // send data:  id = 0x00, standrad frame, data len = 8, stmp: data buf
  CAN.sendMsgBuf(0x01, 0, 4, msg);
  delay(20); 

}

void headlight() {

  int switchIn = 0;
  

  switchIn = analogRead(headlightSwitch);
  Serial.println(switchIn); //on for headlight switch debug
  
  //**Parking Lights**
  if (switchIn < 900) {
    parkingState = HIGH; 
    parkingBrightness = brightness;
  }    
  else {
    parkingState = LOW;
    parkingBrightness = 0;
  }
  
  //**Low Beams**
  if ((switchIn > 475) && (switchIn < 575)) {
    lowBeamState = HIGH;
  } 
  else {
    lowBeamState = LOW;
  }
  //**High Beams**
  if (switchIn < 470) {
    highBeamState = HIGH;
  }
  else  {
    highBeamState = LOW;
  }
  digitalWrite (parkingOut, parkingState);
  digitalWrite (lowBeamOut, lowBeamState);
  digitalWrite (highBeamOut, highBeamState);      
}

void turn() {

  int switchIn = 0;
  int brakeSwi = 0;
  unsigned long currentMillis = millis();

  switchIn = analogRead(turnSignalSwitch);
  brakeSwi = analogRead(brakeIn);

  //Serial.print("  Turn Switch: ");
  //Serial.print(switchIn); //for debug

// **Turn Signals**  
  if ((switchIn > 500) && (switchIn < 850)) { //Right Turn Signal
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
  if ((switchIn > 250) && (switchIn < 500))  { //Left Turn Signal
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
  if (rightState == LOW) {
    analogWrite(rearRight, parkingBrightness);
    analogWrite(frontRight, parkingBrightness);
    digitalWrite(clicker, LOW);
  }
  else {
    analogWrite(rearRight, 255);
    analogWrite(frontRight, 255);
    digitalWrite(clicker, HIGH);
  } 
  if (leftState == LOW) {
      analogWrite(rearLeft, parkingBrightness);
      analogWrite(frontLeft, parkingBrightness);
      digitalWrite(clicker, LOW);
    }
    else {
      analogWrite(rearLeft, 255);
      analogWrite(frontLeft, 255);
      digitalWrite(clicker, HIGH);
    }  
//**Brakes**  
  if (brakeSwi < 500) {
    brakeState = HIGH;
  }
  else {
    brakeState = LOW;
  }
  
  if (switchIn > 1000) { // Switch in center
    if (brakeState == HIGH) { //brake lights when signals are off, both light
      analogWrite(rearLeft, 255);
      analogWrite(rearRight, 255);
    }
    else {
      analogWrite(rearLeft, parkingBrightness);
      analogWrite(rearRight, parkingBrightness);
    }
  }
  if ((switchIn > 500) && (switchIn < 750)) { // Switch is right 
    if (brakeState == HIGH) { //brakes are press when right signal is on
      analogWrite(rearLeft, 255); //only light left brake light
    }
    else {
      analogWrite(rearLeft, parkingBrightness);
    }
  }
  if ((switchIn > 250) && (switchIn < 500))  { //Switch is left
    if (brakeState == HIGH) { //brakes are press when left signal is on
      analogWrite(rearRight, 255); //only light right brake light
    }
    else {
      analogWrite(rearRight, parkingBrightness);
    }
  }
}


void hazard() {
  
  int switchIn = 0;
  unsigned long currentMillis = millis();

  switchIn = analogRead(turnSignalSwitch);
    
  if (switchIn < 250) { //Hazard on
    if (currentMillis - previousMillis >= hazardInterval) { //if time elapsed is greater than the signal interval
      previousMillis = currentMillis; //then reset time
      if (hazardState == LOW) { //if right state was low
        hazardState = HIGH;
      }     
      else {
        hazardState = LOW;
      }

      if (hazardState == LOW) {
        rightState = LOW;
        leftState = LOW;
      }
      else {
        rightState = HIGH;
        leftState = HIGH;
      }
    }
  digitalWrite(frontRight, rightState);
  digitalWrite(frontLeft, rightState);
  digitalWrite(rearRight, leftState);
  digitalWrite(rearLeft, leftState);    
  }
}
