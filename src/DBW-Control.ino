#include <FastCRC.h>
#include <FastCRC_tables.h>
#include <FastCRC_cpu.h>
#include <MultiMap.h>
#include <EnableInterrupt.h>
#include <Arduino.h>
#include <PID_v1.h>
#include "ETC.h"


//AlphaX Throttle Control
//Licensed under the GPL v3
/*
Alpha X ThrttoleController
Copyright (C) Norman Paulino

This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License
as published by the Free Software Foundation; either version 2
of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful,la
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
*/
void loadCalibration();
void idleRequestPWM();
void serialRead(byte);

// Inputs
const int POT_THROTTLE = A0; // Servo Position Input
const int POT_THROTTLE2 = A1; // Servo Position Input2
const int POT_PEDAL = A3; // Pedal Sensor Input
const int POT_PEDAL2 = A2; // Pedal Sensor Input
const int POT_IDLE = A4; //Requested Idle Input
const byte idlePWMPin = 2;

// Pins H bridge
const byte HB_DIRECTION = 11; // H bridge Direction
const byte HB_PWM = 10;     // H bridge PWM (speed)

// Throttle constraints, max voltages in both direction
 int MinTPS;
 int MinTPS2;
 int HalfTPS2;
 int MaxTPS;
 int MaxTPS2;
 int MaxAPP = 1023;
 int MaxAPP2 = 1023;


 int HalfAPP2;
 int MinAPP = 0;
 int MinAPP2 = 0;
 int inAPP = 0;
 unsigned int throttleRest;
 int calMode = 0;
 int calLevel1 = 0;
 float calVal1 = 0.00;
 int calLevel2 = 0;
 float calVal2 = 0.00;
 int calSuccessCount = 0;
 int calSuccessCountSec = 0;
 int rawTPS2 = 0;
 int rawAPP2 = 0;
 int TPSerror = 0;
 int APPerror = 0;
 int secondAPP2 = 500;
 int secondOutAPP2 = 650;
 int secondTPS2 = 327;
 int secondOutTPS2 = 691;
 int thirdAPP2 = 562;
 int thirdOutAPP2 = 715;
 int thirdTPS2 = 602;
 int thirdOutTPS2 = 1016;

 int tpsPercent = 0;
 int appPercent = 0;

 int expectAPP2 = 0;
 int expectTPS2 = 0;
 

int tenthirdPointAPP;
int tenthirdPointTPS;
float thirdpPointAPPsafety = 0;
float thirdpPointTPSsafety = 0;
float thirdpPointAPP2safety = 0;
float thirdpPointTPS2safety = 0;

float thirdPointAPP = 0;
float thirdPointTPS = 0;
float appIn[] = {0, 0, 0};
float appOut[] = {0, 0, 0};

float app2In[] = {0, 0, 0, 0};
float app2Out[] = {0, 0, 0, 0};

float tps2In[] = {0, 0, 0, 0};
float tps2Out[] = {0, 0, 0, 0};

//PID Control variables
double kP = 2.3;
double kI = 1;
double kD = .9;
//Idle constants
double idlekP = 1.8;
double idlekI = .08;
double idlekD = 0.9;

//customvars
unsigned long timeDiag = 500;
bool diagEnabled = false;
bool calFlag = 0;
int throttleCorr = 0;
byte idleSetPoint = 0;
unsigned long idlePWMtime = 0;
bool idleCount = true;
byte dutyCyclein = 0;
byte throttleMode = 0;
bool safetyCheck = true;
byte safetyCount = 0;
byte safetyMode = 0;

FastCRC8 CRC8;

//Define the PID controller
double Input, Output, SetPoint;
PID throttlePID(&Input, &Output, &SetPoint, kP, kI, kD, DIRECT);
//double idleOutput, idleSetPoint;
//PID idlePID(&Input, &idleOutput,  = true&idleSetPoint, idlekP, idlekI, idlekD, DIRECT);

//Operating States flag 
char state = 's'; //s = safety, t = transient, i = idle, c = constant (steady state)


void setup() {
// put your setup code here, to run once:
 //Enable PID control
 pinMode(3, OUTPUT);
 digitalWrite(3, HIGH);
 Serial.begin(115200);
 Serial.println("Box OK!");
  throttlePID.SetMode(AUTOMATIC);
  throttlePID.SetSampleTime(1);
  //idlePID.SetMode(AUTOMATIC);
  calFlag = EEPROM.read(8);
  if (calFlag){
    uint8_t CRCbuf[37];
    for (int i = 0; i < 38; i++){
      CRCbuf[i] = EEPROM.read(i);
    }
    uint8_t CRCvalue = EEPROM.read(38);
    Serial.print("CRC buffer value = "); Serial.println(CRC8.smbus(CRCbuf, sizeof(CRCbuf)));
    Serial.print("CRC value = "); Serial.println(EEPROM.read(38));
    if (CRCvalue == CRC8.smbus(CRCbuf, sizeof(CRCbuf))){
      loadCalibration();
      Serial.println("Calibration Loaded");
      digitalWrite(3, LOW);
    }
    else{ 
      safetyMode = 3;
      Serial.println("Calibration Corrupted! No loading was done!");
    }
    
  }
  pinMode(HB_DIRECTION, OUTPUT);
  enableInterrupt(idlePWMPin, idleRequestPWM, CHANGE);
  
}

void loop() {
  if (Serial.available()){
    byte serialRead = Serial.read() - 48;
    Serial.println(serialRead);
    serialCommands(serialRead);
  }
  
  //Read all inputs
    int inTPS = analogRead(POT_THROTTLE); //read for example 48, meaning throttle at rest, max is 758
    int rawAPP = analogRead(POT_PEDAL); //read for example 70, app is 0 (no demand), max is 940
  
    int thirdAPP = MaxAPP * thirdPointAPP;
    int thirdTPS = MaxTPS * thirdPointTPS;
    appIn[0] = MinAPP;
    appIn[1] = thirdAPP + MinAPP;
    appIn[2] = MaxAPP;
    appOut[0] = MinTPS;
    appOut[1] = thirdTPS + MinTPS;
    appOut[2] = MaxTPS;
    float sampleSize = 3;

    
    app2In[0] = MinAPP;
    app2In[1] = secondAPP2; //500
    app2In[2] = thirdAPP2; //562
    app2In[3] = MaxAPP; //865
    app2Out[0] = MinAPP2;
    app2Out[1] = secondOutAPP2; //650
    app2Out[2] = thirdOutAPP2; //715
    app2Out[3] = MaxAPP2;

    tps2In[0] = MinTPS; //138
    tps2In[1] = secondTPS2;  //327
    tps2In[2] = thirdTPS2;  //602
    tps2In[3] = MaxTPS; //890
    tps2Out[0] = MinTPS2; //455
    tps2Out[1] = secondOutTPS2; //691
    tps2Out[2] = thirdOutTPS2;  //1016
    tps2Out[3] = MaxTPS2; //1019
    float safeSampleSize = 4;

    appPercent = map(rawAPP, MinAPP, MaxAPP, 0, 100);
    tpsPercent = map(inTPS, MinTPS, MaxTPS, 0, 100);

    idleSetPoint = 100 + dutyCyclein; //analogRead(POT_IDLE);
    
    switch(throttleMode){
      case 0:
       inAPP = multiMap<float>(rawAPP, appIn, appOut, sampleSize);
       break;
      case 1:
       inAPP = map(rawAPP, MinAPP, MaxAPP, MinTPS, MaxTPS * 0.5);
       break;
    }
    
    if (calMode == 0) {
      if (inAPP <= idleSetPoint){
        state = 'i';  //sets idle mode
      }
      else if (inAPP > idleSetPoint){
        state = 'c'; //Sets to steady state
      }
      if (abs(inTPS - inAPP) >= 60){
        //Serial.println(inTPS -inAPP);
        state = 't';  //sets to transient mode
      }
      if (safetyMode > 0){
        state = 's';
      }
    } else if(calMode == 1 || calMode == 2) {
      state = 'k';
      safetyMode = 0;
      safetyCount = 0;
  }

    if (((millis() % 200) == 0) && (safetyCheck == true)){
        rawTPS2 = analogRead(POT_THROTTLE2);
        rawAPP2 = analogRead(POT_PEDAL2);
        expectAPP2 = multiMap<float>(rawAPP, app2In, app2Out, safeSampleSize);
        expectTPS2 = multiMap<float>(inTPS, tps2In, tps2Out, safeSampleSize);
        TPSerror = abs(expectTPS2-rawTPS2);
        APPerror = abs(expectAPP2-rawAPP2);
        
        digitalWrite(3, safetyMode);

        if (state == 'i'){
           TPSerror = 0;
        }
        if ((TPSerror > 10) || (APPerror > 10)){
          safetyCount++;
        }
        else if (safetyCount != 0){
          safetyCount--;
        }
        if (safetyCount == 0){
          state = 'i';
          safetyCheck = true;
          safetyMode = 0;
          throttleMode = 0;
        }
        if(safetyCount > 10){
        //if ((safetyCount > 10) && (safetyCount <= 20)){
          //Serial.println("ETC SYSTEM ERROR!");
          safetyMode = 1;
          throttleMode = 1;
        }
        /*else if ((safetyCount > 20) && (safetyCount <= 30)){
         safetyMode = 2;
          Serial.println("ETC SYSTEM ERROR HIGH!");
        }
        else if (safetyCount > 30){
          safetyMode = 3;
          Serial.println("ETC SYSTEM ERROR CRITICAL!");
        }*/
        safetyCheck = false;
        if ((diagEnabled) && safetyMode > 0){
          Serial.print("TPS Error = "); Serial.println(TPSerror);
          Serial.print("TPS = "); Serial.println(inTPS);
          Serial.print("Expected TPS2  = "); Serial.println(expectTPS2);
          Serial.print("TPS2 = "); Serial.println(rawTPS2);
          Serial.print("APP Error = "); Serial.println(APPerror);
          Serial.print("APP = "); Serial.println(rawAPP);
          Serial.print("Expected APP2  = "); Serial.println(expectAPP2);
          Serial.print("APP2 = "); Serial.println(rawAPP2);
          Serial.print("safetyCount = "); Serial.println(safetyCount);
          Serial.print("safetyMode = "); Serial.println(safetyMode);
          Serial.print(app2In[0]);
          Serial.print(" ");
          Serial.print(app2In[1]);
          Serial.print(" ");
          Serial.print(app2In[2]);
          Serial.print(" ");
          Serial.print(app2In[3]);
          Serial.println(" ");
          Serial.print(app2Out[0]);
          Serial.print(" ");
          Serial.print(app2Out[1]);
          Serial.print(" ");
          Serial.print(app2Out[2]);
          Serial.print(" ");
          Serial.print(app2Out[3]);
          Serial.println(" ");
          Serial.print(tps2In[0]);
          Serial.print(" ");
          Serial.print(tps2In[1]);
          Serial.print(" ");
          Serial.print(tps2In[2]);
          Serial.print(" ");
          Serial.print(tps2In[3]);
          Serial.println(" ");
          Serial.print(tps2Out[0]);
          Serial.print(" ");
          Serial.print(tps2Out[1]);
          Serial.print(" ");
          Serial.print(tps2Out[2]);
          Serial.print(" ");
          Serial.print(tps2Out[3]);
          Serial.println(" ");
          
          /*Serial.print("MaxTPS % = "); Serial.println(MaxTPS);
          Serial.print("MinTPS % = "); Serial.println(MinTPS);
          Serial.print("MaxTPS2 % = "); Serial.println(MaxTPS2);
          Serial.print("MinTPS2 % = "); Serial.println(MinTPS2);
          Serial.print("APPerror = "); Serial.println(APPerror);*/
          
        }
    }
    else if((millis() % 200) != 0) { safetyCheck = true;}
     if ((millis() - timeDiag) > 600){
      if ((diagEnabled) && safetyMode == 0){
        /*Serial.print("TPS = "); Serial.println(inTPS);
        Serial.print("Raw APP% = "); Serial.println(appPercent);
        Serial.print("Raw TPS% = "); Serial.println(tpsPercent);
        //Serial.print("Calibrated Target% = "); Serial.println(inappPercent);
        Serial.print("State: "); Serial.println(state);
        Serial.print("Idle% = "); Serial.println(idleSetPoint);
        Serial.print("Input idle DC = ");Serial.println(dutyCyclein);
        Serial.print("PID Output = "); Serial.println(Output);
        Serial.print("APP Map In = "); Serial.print(appIn[0]); Serial.print(" "); Serial.print(appIn[1]); Serial.print(" "); Serial.println(appIn[2]);
        Serial.print("APP Map Out = "); Serial.print(appOut[0]); Serial.print(" "); Serial.print(appOut[1]); Serial.print(" "); Serial.println(appOut[2]);*/
        Serial.print("TPS Error = "); Serial.println(TPSerror);
        Serial.print("TPS = "); Serial.println(inTPS);
        Serial.print("Expected TPS2  = "); Serial.println(expectTPS2);
        Serial.print("TPS2 = "); Serial.println(rawTPS2);
        Serial.print("APP Error = "); Serial.println(APPerror);
        Serial.print("APP = "); Serial.println(rawAPP);
        Serial.print("Expected APP2  = "); Serial.println(expectAPP2);
        Serial.print("APP2 = "); Serial.println(rawAPP2);
        Serial.print("safetyCount = "); Serial.println(safetyCount);
        Serial.print("safetyMode = "); Serial.println(safetyMode);
        Serial.print(app2In[0]);
          Serial.print(" ");
          Serial.print(app2In[1]);
          Serial.print(" ");
          Serial.print(app2In[2]);
          Serial.print(" ");
          Serial.print(app2In[3]);
          Serial.println(" ");
          Serial.print(app2Out[0]);
          Serial.print(" ");
          Serial.print(app2Out[1]);
          Serial.print(" ");
          Serial.print(app2Out[2]);
          Serial.print(" ");
          Serial.print(app2Out[3]);
          Serial.println(" ");
          Serial.print(tps2In[0]);
          Serial.print(" ");
          Serial.print(tps2In[1]);
          Serial.print(" ");
          Serial.print(tps2In[2]);
          Serial.print(" ");
          Serial.print(tps2In[3]);
          Serial.println(" ");
          Serial.print(tps2Out[0]);
          Serial.print(" ");
          Serial.print(tps2Out[1]);
          Serial.print(" ");
          Serial.print(tps2Out[2]);
          Serial.print(" ");
          Serial.print(tps2Out[3]);
          Serial.println(" ");
      }
      //Serial.print("Idle% = "); Serial.println(idleSetPoint);
      timeDiag = millis();
    }
    
   
    //convert inputs to doubles
    Input = inTPS;
    SetPoint = inAPP;
    switch (state){
      case 'c':
      throttlePID.SetTunings(1.1, .07, 0.03);
      //  throttlePID.SetSampleTime(2);
      if (inTPS < inAPP){
          digitalWrite(HB_DIRECTION, 0);            //sets ETC Direction to forward
          throttlePID.SetControllerDirection(DIRECT);
          throttlePID.SetOutputLimits(50,200);
          //Serial.print("PID Output = "); Serial.println(Output);
          throttlePID.Compute();                    //compute PID based correction
          analogWrite(HB_PWM,(Output));  //uses the base duty and adds a correction factor with PID
          //Serial.println("Steady State Forward");
        }
      else {
           digitalWrite(HB_DIRECTION, 1);                       //set ETC Direction Backward
           /*throttlePID.SetTunings(1.4, 1, 0.7);
           throttlePID.SetControllerDirection(REVERSE);
           throttlePID.SetOutputLimits(20,120);
           throttlePID.Compute();   */
           if (abs(inTPS - inAPP) < 5){
            analogWrite(HB_PWM,20);  //makes throttle go backward with open loop values
            //Serial.println("Idle backslow");
           }
           else{
            analogWrite(HB_PWM,80);  //makes throttle go backward with open loop values
            //Serial.println("Idle back");
           }
           throttlePID.Compute();                    //compute PID based correction

          }
      break;
      case 't':
      //Transient function that forces the TP to open at max rate, and close at a relatively fast rate to avoid slamming
      if (inTPS < inAPP){ //opening
            digitalWrite(HB_DIRECTION, 0);
            analogWrite(HB_PWM,200);
            //Serial.println("Open Transient");
          }
        else {
           digitalWrite(HB_DIRECTION, 1); //closing
             if ((inTPS < throttleRest -50)) {
              analogWrite(HB_PWM, 20);
             //Serial.println("Closing transient - slow");
             }
             else{
               analogWrite(HB_PWM, 170);
               //Serial.println("Closing transient");
             }    
        }
        //Serial.println("Transient mode");
        break;
        case 'i':
           // throttlePID.SetSampleTime(1);
          SetPoint = idleSetPoint;
          if (inTPS < idleSetPoint){
            throttlePID.SetTunings(1.1, .07, 0.0);
            throttlePID.SetControllerDirection(DIRECT);
            digitalWrite(HB_DIRECTION, 0);                //sets ETC Direction to forward
            throttlePID.SetOutputLimits(5,60);
            throttlePID.Compute();                            //compute PID based correction
            analogWrite(HB_PWM,(Output));  //uses the base duty and adds a correction facttor with PID
            //Serial.println("Idle opening");
          }
          else {
           digitalWrite(HB_DIRECTION, 1);                       //set ETC Direction Backward
           /*throttlePID.SetTunings(1.4, 1, 0.7);
           throttlePID.SetControllerDirection(REVERSE);
           throttlePID.SetOutputLimits(20,120);
           throttlePID.Compute();   */
           if (abs(inTPS - idleSetPoint) < 5){
            analogWrite(HB_PWM,20);  //makes throttle go backward with open loop values
            //Serial.println("Idle backslow");
           }
           else{
            analogWrite(HB_PWM,105);  //makes throttle go backward with open loop values
            //Serial.println("Idle back");
           }
           //Serial.println("Idle closing");
           //Serial.println(Output);
          throttlePID.Compute();
          }
        //Serial.println("Idle control");
        break;
        case 's': // safetymodes
          if (safetyMode == 1){ 
            throttlePID.SetTunings(1.9, .07, 0.03);
            SetPoint = constrain(SetPoint, 160, 980);
            
            if (inTPS < inAPP){
              digitalWrite(HB_DIRECTION, 0);            //sets ETC Direction to forward
              throttlePID.SetControllerDirection(DIRECT);
              throttlePID.SetOutputLimits(20,180);
              //Serial.print("PID Output = "); Serial.println(Output);
              throttlePID.Compute();                    //compute PID based correction
              analogWrite(HB_PWM,(Output));  //uses the base duty and adds a correction factor with PID
              //Serial.println("Steady State Forward");
            }
            else {
             digitalWrite(HB_DIRECTION, 1);                       //set ETC Direction Backward
             /*throttlePID.SetTunings(1.4, 1, 0.7);
             throttlePID.SetControllerDirection(REVERSE);
             throttlePID.SetOutputLimits(20,120);
             throttlePID.Compute();   */
             if (abs(inTPS - inAPP) < 5){
              analogWrite(HB_PWM,20);  //makes throttle go backward with open loop values
              //Serial.println("Idle backslow");
             }
             else{
              analogWrite(HB_PWM,80);  //makes throttle go backward with open loop values
              //Serial.println("Idle back");
             }
             throttlePID.Compute();                    //compute PID based correction
  
            }
          }
          else if (safetyMode == 2){
            digitalWrite(HB_DIRECTION, 1);
            analogWrite(HB_PWM,60);
          }
          else {
            digitalWrite(HB_DIRECTION, 1);
            analogWrite(HB_PWM,0);
            safetyCheck = false;
          }
         break;
        case 'k': //calibration mode
          
          switch(calMode){
            case 1:
              /*if(calSuccessCount == 0){
                SetPoint = calLevel1;
              }else if(calSuccessCount == 1 && calSuccessCountSec == 0){
                SetPoint = calLevel2;
              }*/
              switch(calSuccessCountSec){
                case 0:
                  switch(calSuccessCount){
                    case 0:
                      SetPoint = calLevel1;
                      break;
                    case 1:
                      SetPoint = calLevel2;
                      break;
                  }
                  break;
                  case 1:
                    break;
                  default:
                    break;
              }


              throttlePID.SetTunings(1.3, .07, 0.02);
              if (abs(SetPoint - inTPS) > 40){
                if (inTPS < SetPoint){ //opening
                  digitalWrite(HB_DIRECTION, 0);
                  analogWrite(HB_PWM,220);
                  //Serial.println("Open Transient");
                }
                if(inTPS > SetPoint){
                  digitalWrite(HB_DIRECTION, 1);
                  analogWrite(HB_PWM,50);
                }    
              
              } else if (inTPS < SetPoint){
                  digitalWrite(HB_DIRECTION, 0);            //sets ETC Direction to forward
                  throttlePID.SetControllerDirection(DIRECT);
                  throttlePID.SetOutputLimits(20,230);
                  throttlePID.Compute();                    //compute PID based correction
                  analogWrite(HB_PWM,(Output));  //uses the base duty and adds a correction factor with PID
                  
                }
              else {
                  digitalWrite(HB_DIRECTION, 1);                       //set ETC Direction Backward
                  /*throttlePID.SetTunings(1.4, 1, 0.7);
                  throttlePID.SetControllerDirection(REVERSE);
                  throttlePID.SetOutputLimits(20,120);
                  throttlePID.Compute();   */
                  if (abs(inTPS - SetPoint) < 5){
                    analogWrite(HB_PWM,5);  //makes throttle go backward with open loop values
                    //Serial.println("Idle backslow");
                  }
                  else{
                    analogWrite(HB_PWM,20);  //makes throttle go backward with open loop values
                    //Serial.println("Idle back");
                  }
                  throttlePID.Compute();                    //compute PID based correction

                  }
                  if(calSuccessCount == 0){
                    if(inTPS == calLevel1){
                      secondTPS2 = analogRead(POT_THROTTLE);
                      secondOutTPS2 = analogRead(POT_THROTTLE2);
                      Serial.print("Calibration success TPS: ");
                      Serial.println(secondTPS2);
                      Serial.print("Calibration success TPS2: ");
                      Serial.println(secondOutTPS2);
                      calSuccessCount = 1;
                    }
                  }else if(calSuccessCount == 1 && calSuccessCountSec == 0){
                    if(inTPS == calLevel2){
                      thirdTPS2 = analogRead(POT_THROTTLE);
                      thirdOutTPS2 = analogRead(POT_THROTTLE2);
                      Serial.print("Calibration success TPS: ");
                      Serial.println(thirdTPS2);
                      Serial.print("Calibration success TPS2: ");
                      Serial.println(thirdOutTPS2);
                      calSuccessCountSec = 1;
                    }                  
                  }else if(calSuccessCount == 1 && calSuccessCountSec == 1){
                    calMode = 0;
                    calSuccessCount = 0;
                    calSuccessCountSec = 0;
                  }
                  break;
                case 2:    
                  if(rawAPP == calLevel1){ 
                    secondAPP2 = analogRead(POT_PEDAL);
                    secondOutAPP2 = analogRead(POT_PEDAL2);
                    Serial.print("Calibration success APP: ");
                    Serial.print(secondAPP2);
                    Serial.print(" APP2: ");
                    Serial.println(secondOutAPP2); 
                    
                    calSuccessCount = 1;
                  
                  }
                  if(rawAPP == calLevel2){ 
                    thirdAPP2 = analogRead(POT_PEDAL);
                    thirdOutAPP2 = analogRead(POT_PEDAL2);
                    Serial.print("Calibration success APP: ");
                    Serial.print(thirdAPP2);
                    Serial.print(" APP2: ");
                    Serial.println(thirdOutAPP2);
                    calSuccessCountSec = 1;
                    
                  }
                  if(calSuccessCount == 1 && calSuccessCountSec == 1){
                    calMode = 0;
                    calSuccessCount = 0;
                    calSuccessCountSec = 0;
                  }
                  break;
              }
          break;
        default: //Assumes an error in the char assignation and closes the throttle
          digitalWrite(HB_DIRECTION, 1);
          analogWrite(HB_PWM,0);
         break;
    }
}

void serialCommands(byte serialRead){
  switch(serialRead){
      case 1:
        //Calibrate TPS
        MaxTPS = FindMaxTPS(HB_PWM, HB_DIRECTION, POT_THROTTLE);
        MaxTPS2 = FindMaxTPS(HB_PWM, HB_DIRECTION, POT_THROTTLE2);
        MinTPS = FindMinTPS(HB_PWM, HB_DIRECTION, POT_THROTTLE);
        MinTPS2 = FindMinTPS(HB_PWM, HB_DIRECTION, POT_THROTTLE2);
        throttleRest = FindRestingTPS(HB_PWM, HB_DIRECTION, POT_THROTTLE);
        break;
        
      case 2:
      //CalibrateAPP
        MaxAPP = FindMaxAPP(POT_PEDAL);
        MaxAPP2 = FindMaxAPP(POT_PEDAL2);
        MinAPP = FindMinAPP(POT_PEDAL);
        MinAPP2 = FindMinAPP(POT_PEDAL2);
        break;
      case 3:
        diagEnabled = !diagEnabled;
        break;
      case 4:
        calFlag = burnCalibration(MinTPS, MaxTPS, MinAPP, MaxAPP, MinTPS2, MaxTPS2, MinAPP2, MaxAPP2,throttleRest,throttleMode,tenthirdPointAPP,tenthirdPointTPS, secondAPP2, secondTPS2, secondOutAPP2, secondOutTPS2, thirdAPP2, thirdTPS2, thirdOutAPP2, thirdOutTPS2);
        break;
      case 5:
        calFlag = clearCalibration();
        break;
      case 6:
        throttleMode++;
        if (throttleMode > 1){ throttleMode = 0;}
        Serial.print("Throttlemode changed to = ");Serial.println(throttleMode);
        break;
      case 7:
        Serial.setTimeout(2500);
        calMode = 0;
       
        Serial.println("0 - TPS calibration, 1 - APP calibration");
        calMode = Serial.parseInt();
        switch(calMode){
          case 0:
            Serial.println("0 - multiplier of maximum TPS1, 1 - absolute TPS1");
            calMode = Serial.parseInt();
            switch(calMode){
            case 0:
                Serial.println("Enter calibration offset multiplier for first TPS point");
                calVal1 = Serial.parseFloat();
                Serial.println(String(calVal1));
                Serial.println("Enter calibration offset multiplier for second TPS point");
                calVal2 = Serial.parseFloat();
                Serial.println(String(calVal2));
                calLevel1 = calVal1 * (MaxTPS - MinTPS) + MinTPS;
                calLevel2 = calVal2 * (MaxTPS - MinTPS) + MinTPS;
                Serial.println(String(calLevel1));
                Serial.println(String(calLevel2));
                break;
              case 1:
                Serial.println("Enter calibration offset multiplier for first TPS point");
                calLevel1 = Serial.parseFloat();
                Serial.println(String(calLevel1));
                Serial.println("Enter calibration offset multiplier for second TPS point");
                calLevel2 = Serial.parseFloat();
                Serial.println(String(calLevel2));
                break;
              

            }
            calMode = 1;
            break;
         
          case 1:
            Serial.println("0 - multiplier of maximum APP1, 1 - absolute APP1");
            calMode = Serial.parseInt();
            switch(calMode){
            case 0:
                Serial.println("Enter calibration offset multiplier for first APP point");
                calVal1 = Serial.parseFloat();
                Serial.println(String(calVal1));
                Serial.println("Enter calibration offset multiplier for second APP point");
                calVal2 = Serial.parseFloat();
                Serial.println(String(calVal2));
                calLevel1 = calVal1 * (MaxAPP - MinAPP) + MinAPP;
                calLevel2 = calVal2 * (MaxAPP - MinAPP) + MinAPP;
                Serial.println(String(calLevel1));
                Serial.println(String(calLevel2));
                break;
              case 1:
                Serial.println("Enter calibration offset multiplier for first APP point");
                calLevel1 = Serial.parseFloat();
                Serial.println(String(calLevel1));
                Serial.println("Enter calibration offset multiplier for second APP point");
                calLevel2 = Serial.parseFloat();
                Serial.println(String(calLevel2));
                break;
              

            }
            calMode = 2;
            break;
        }
        

        break;
      case 8: 
        Serial.setTimeout(2000);
        Serial.println("Enter APP mid point calibration");
        thirdPointAPP = Serial.parseFloat();
        Serial.print("Entered calibration: "); Serial.println(thirdPointAPP);
        delay(200);
        Serial.println("Enter TPS mid point calibration");
        thirdPointTPS = Serial.parseFloat();
        Serial.print("Entered calibration: "); Serial.println(thirdPointTPS);
        tenthirdPointAPP = thirdPointAPP * 100.0;
        tenthirdPointTPS = thirdPointTPS * 100.0;
        break;
      case 9:
        uint8_t CRCbuf[37];
        for (int i = 0; i < 38; i++){
          CRCbuf[i] = EEPROM.read(i);
        }
        //uint8_t CRCvalue2 = EEPROM.read(20);
        Serial.print("CRC buffer value = "); Serial.println(CRC8.smbus(CRCbuf, sizeof(CRCbuf)));
        Serial.print("CRC value = "); Serial.println(EEPROM.read(38));
        break;
      default:
        break;
    }
}

void loadCalibration(){
  byte high = EEPROM.read(0);
  byte low = EEPROM.read(1);
  MinTPS = word(high,low);
  high = EEPROM.read(2);
  low = EEPROM.read(3);
  MaxTPS = word(high,low);
  high = EEPROM.read(4);
  low = EEPROM.read(5);
  MinAPP = word(high,low);
  high = EEPROM.read(6);
  low = EEPROM.read(7);
  MaxAPP = word(high,low);
  high = EEPROM.read(9);
  low = EEPROM.read(10);
  throttleRest = word(high,low);
  calFlag = EEPROM.read(8);
  
  high = EEPROM.read(11);
  low = EEPROM.read(12);
  MinTPS2 = word(high,low);
  high = EEPROM.read(13);
  low = EEPROM.read(14);
  MaxTPS2 = word(high,low);
  high = EEPROM.read(15);
  low = EEPROM.read(16);
  MinAPP2 = word(high,low);
  high = EEPROM.read(17);
  low = EEPROM.read(18);
  MaxAPP2 = word(high,low);
  throttleMode = EEPROM.read(19);
  thirdPointAPP = EEPROM.read(20) / 100.0;
  thirdPointTPS = EEPROM.read(21) / 100.0;
  high = EEPROM.read(22);
  low = EEPROM.read(23);
  secondAPP2 = word(high,low);
  high = EEPROM.read(24);
  low = EEPROM.read(25);
  secondOutAPP2 = word(high,low);
  high = EEPROM.read(26);
  low = EEPROM.read(27);
  secondTPS2 = word(high,low);
  high = EEPROM.read(28);
  low = EEPROM.read(29);
  secondOutTPS2 = word(high,low);
  high = EEPROM.read(30);
  low = EEPROM.read(31);
  thirdAPP2 = word(high,low);
  high = EEPROM.read(32);
  low = EEPROM.read(33);
  thirdOutAPP2 = word(high,low);
  high = EEPROM.read(34);
  low = EEPROM.read(35);
  thirdTPS2 = word(high,low);
  high = EEPROM.read(36);
  low = EEPROM.read(37);
  thirdOutTPS2 = word(high,low);
}

void idleRequestPWM(){
  //Serial.println("INTERRUPT! ");
  if (digitalRead(idlePWMPin)){
    if (idleCount){
      idlePWMtime = millis();
      idleCount = false;
    }
  }
  else{
    if (!idleCount){
      dutyCyclein = millis() - idlePWMtime;
      idleCount = true;
    }
  }
}
